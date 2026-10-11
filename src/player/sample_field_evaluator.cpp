// -*-c++-*-

/*
 *Copyright:

 Cyrus2D
 Modified by Omid Amini, Nader Zare
 
 Gliders2d
 Modified by Mikhail Prokopenko, Peter Wang

 Copyright (C) Hiroki SHIMORA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code; see the file COPYING.  If not, write to
 the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 *EndCopyright:
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "sample_field_evaluator.h"

#include "field_analyzer.h"
#include "simple_pass_checker.h"

#include <rcsc/player/player_evaluator.h>
#include <rcsc/common/server_param.h>
#include <rcsc/common/logger.h>
#include <rcsc/math_util.h>

#include <rcsc/player/world_model.h>

#include <rcsc/geom/voronoi_diagram.h>

#include <iostream>
#include <algorithm>
#include <cmath>
#include <cfloat>

// #define DEBUG_PRINT

using namespace rcsc;

static const int VALID_PLAYER_THRESHOLD = 8;

//
// 传球收益参数（可调）
//

//! 链条最后把球交给位置不比我方现有球位更靠后的队友、且接球点有空间时的奖励值。
//! 单位与 point 的前进项一致（1.0 约等于推进 1 米的价值）。设为 0.0 即关闭本项。
static const double PASS_POSSESSION_BONUS = 8.0;

//! 接球点与最近对手的最小距离，低于该值视为被紧逼，不给奖励。
static const double PASS_BONUS_MIN_SPACE = 2.0;

//! 允许的最大「向后传球」幅度（米），超过则认为不如就地控制。
static const double PASS_BONUS_MAX_BACKWARD_X = 5.0;


namespace {

/*!
  \brief Voronoi「空当点」候选（只依赖 world model，按 cycle 缓存）
 */
struct VoronoiCandidate {
    rcsc::Vector2D pos_;
    double back_offset_;   //!< 相对当前球位置允许向后的偏移（顶点 5.0 / 线段交点 25.0）
    double opp_min_dist_;  //!< 到最近对手的距离
    double our_dist_;      //!< 到最近队友（含自己）的距离
};

/*!
  \brief 计算空当点候选集合，每个 cycle 只计算一次。

  \param wm world model

  原实现把 Voronoi 构造以及对所有对手/队友的距离扫描写在 evaluate_state() 内部，
  而 evaluate_state() 每个 cycle 会被 ActionChainGraph 调用数百次
  （DEFAULT_MAX_EVALUATE_LIMIT = 500 次评估）。这些量与传入的 state 无关，
  因此按 cycle 缓存（同一 cycle 内 world model 不再变化）：
  结果与原逐次计算一致，只是省掉重复计算。
 */
const std::vector< VoronoiCandidate > &
get_voronoi_candidates( const rcsc::WorldModel & wm )
{
    static rcsc::GameTime s_update_time( -1, 0 );
    static std::vector< VoronoiCandidate > s_candidates;

    if ( s_update_time == wm.time() )
    {
        return s_candidates;
    }

    s_update_time = wm.time();
    s_candidates.clear();

    VoronoiDiagram vd;

    for ( auto o = wm.opponentsFromSelf().begin(), o_end = wm.opponentsFromSelf().end();
          o != o_end;
          ++o )
    {
        vd.addPoint( (*o)->pos() );
    }

    vd.compute();

    //
    // 收集候选点（顺序必须与原来两个循环一致：先顶点，再线段交点）
    //
    struct RawPoint {
        Vector2D pos_;
        double offset_;
        RawPoint( const Vector2D & p, const double & o )
            : pos_( p ), offset_( o )
          { }
    };

    std::vector< RawPoint > points;

    for ( auto p = vd.vertices().begin(), p_end = vd.vertices().end();
          p != p_end;
          ++p )
    {
        points.push_back( RawPoint( *p, 5.0 ) );
    }

    Vector2D y1( wm.offsideLineX(), -34.0 );
    Vector2D y2( wm.offsideLineX(), 34.0 );
    Vector2D z1( wm.offsideLineX(), -34.0 );
    Vector2D z2( wm.offsideLineX(), 34.0 );

    if ( wm.ball().pos().x > 25.0 )
    {
        if ( wm.ball().pos().y < 0.0 )
        {
            y2.y = 20.0;
        }
        if ( wm.ball().pos().y > 0.0 )
        {
            y1.y = -20.0;
        }
    }

    if ( wm.ball().pos().x > 36.0 )
    {
        if ( wm.ball().pos().y < 0.0 )
        {
            y2.y = 8.0;
        }
        if ( wm.ball().pos().y > 0.0 )
        {
            y1.y = -8.0;
        }
    }

    z1.x = y1.x + 6.0;
    if ( z1.x > 52.5 )
    {
        z1.x = 52.0;
    }

    z2.x = y2.x + 6.0;
    if ( z2.x > 52.5 )
    {
        z2.x = 52.0;
    }

    z1.y = y1.y;
    z2.y = y2.y;

    Line2D offside_line( y1, y2 );
    Line2D forward_line( z1, z2 );

    for ( auto seg = vd.segments().begin(), seg_end = vd.segments().end();
          seg != seg_end;
          ++seg )
    {
        const Vector2D si = (*seg).intersection( offside_line );
        const Vector2D fi = (*seg).intersection( forward_line );

        if ( si.isValid()
             && std::fabs( si.y ) < 34.0
             && std::fabs( si.x ) < 52.5 )
        {
            points.push_back( RawPoint( si, 25.0 ) );
        }

        if ( fi.isValid()
             && std::fabs( fi.y ) < 34.0
             && std::fabs( fi.x ) < 52.5
             && wm.ball().pos().x < 37.0 )
        {
            points.push_back( RawPoint( fi, 25.0 ) );
        }
    }

    //
    // 计算每个候选点到最近对手 / 最近队友的距离（只依赖 world model）
    //
    s_candidates.reserve( points.size() );

    for ( auto it = points.begin(); it != points.end(); ++it )
    {
        VoronoiCandidate candidate;
        candidate.pos_ = it->pos_;
        candidate.back_offset_ = it->offset_;
        candidate.opp_min_dist_ = 1000.0;
        candidate.our_dist_ = 1000.0;

        for ( auto o = wm.opponentsFromSelf().begin(), o_end = wm.opponentsFromSelf().end();
              o != o_end;
              ++o )
        {
            const double d = ( (*o)->pos() - it->pos_ ).length();
            if ( candidate.opp_min_dist_ > d )
            {
                candidate.opp_min_dist_ = d;
            }
        }

        for ( auto t = wm.teammatesFromSelf().begin(), t_end = wm.teammatesFromSelf().end();
              t != t_end;
              ++t )
        {
            if ( (*t)->pos().x > wm.offsideLineX() + 1.0 )
            {
                continue;
            }

            const double d = ( (*t)->pos() - it->pos_ ).length();
            if ( candidate.our_dist_ > d )
            {
                candidate.our_dist_ = d;
            }
        }

        const double self_dist = ( wm.self().pos() - it->pos_ ).length();
        if ( wm.self().pos().x < it->pos_.x
             && self_dist > 7.0
             && candidate.our_dist_ > self_dist )
        {
            candidate.our_dist_ = self_dist;
        }

        s_candidates.push_back( candidate );
    }

    return s_candidates;
}

}


/*-------------------------------------------------------------------*/
/*!

 */
static double evaluate_state( const PredictState & state , const rcsc::WorldModel & wm );


/*-------------------------------------------------------------------*/
/*!

 */
SampleFieldEvaluator::SampleFieldEvaluator()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
SampleFieldEvaluator::~SampleFieldEvaluator()
{

}

/*-------------------------------------------------------------------*/
/*!

 */
double
SampleFieldEvaluator::operator()(const PredictState &state,
                                 const std::vector<ActionStatePair> & /*path*/,
                                 const rcsc::WorldModel &wm) const
{
    const double final_state_evaluation = evaluate_state( state , wm);

    //
    // ???
    //

    double result = final_state_evaluation;

    return result;
}


/*-------------------------------------------------------------------*/
/*!

 */
static
double
evaluate_state( const PredictState & state, const rcsc::WorldModel & wm )
{
    const ServerParam & SP = ServerParam::i();

    const AbstractPlayerObject * holder = state.ballHolder();

#ifdef DEBUG_PRINT
    dlog.addText( Logger::ACTION_CHAIN,
                  "========= (evaluate_state) ==========" );
#endif

    //
    // if holder is invalid, return bad evaluation
    //
    if ( ! holder )
    {
#ifdef DEBUG_PRINT
        dlog.addText( Logger::ACTION_CHAIN,
                      "(eval) XXX null holder" );
#endif
        return - DBL_MAX / 2.0;
    }

    const int holder_unum = holder->unum();


    //
    // ball is in opponent goal
    //
    if ( state.ball().pos().x > + ( SP.pitchHalfLength() - 0.1 )
         && state.ball().pos().absY() < SP.goalHalfWidth() + 2.0 )
    {
#ifdef DEBUG_PRINT
        dlog.addText( Logger::ACTION_CHAIN,
                      "(eval) *** in opponent goal" );
#endif
        return +1.0e+7;
    }

    //
    // ball is in our goal
    //
    if ( state.ball().pos().x < - ( SP.pitchHalfLength() - 0.1 )
         && state.ball().pos().absY() < SP.goalHalfWidth() )
    {
#ifdef DEBUG_PRINT
        dlog.addText( Logger::ACTION_CHAIN,
                      "(eval) XXX in our goal" );
#endif

        return -1.0e+7;
    }


    //
    // out of pitch
    //
    if ( state.ball().pos().absX() > SP.pitchHalfLength()
         || state.ball().pos().absY() > SP.pitchHalfWidth() )
    {
#ifdef DEBUG_PRINT
        dlog.addText( Logger::ACTION_CHAIN,
                      "(eval) XXX out of pitch" );
#endif

        return - DBL_MAX / 2.0;
    }


    //
    // set basic evaluation
    //

    // G2d: to retrieve opp team name 
    // C2D: Helios 18 Tune removed -> replace with BNN
    // bool heliosbase = false;
    // if (wm.opponentTeamName().find("HELIOS_base") != std::string::npos)
    //     heliosbase = true;

    // G2d: number of direct opponents
        int opp_forward = 0;

        Vector2D egl (52.5, -8.0);
        Vector2D egr (52.5, 8.0);
        Vector2D left = egl - wm.self().pos();
        Vector2D right = egr - wm.self().pos();

        Sector2D sector(wm.self().pos(), 0.0, 10000.0, left.th(), right.th());

        for (auto of = wm.opponentsFromSelf().begin();
             of != wm.opponentsFromSelf().end();
             ++of)
        {
                if ( sector.contains( (*of)->pos() ) && !((*of)->goalie()) )
                opp_forward++;
        }

        double weight = 1.0;
        if (wm.ball().pos().x > 35.0)
            weight = 0.3;

	double depth = 10.0;
    // C2D: Helios 18 Tune removed -> replace with BNN
	// if (heliosbase)
	// 	depth = 0.0;

    double point = state.ball().pos().x * weight;

        Vector2D best_point = ServerParam::i().theirTeamGoalPos();

    // G2d: new eval function
    // C2D: replace PlayerPtrCont::const_iterator with auto
        if ( wm.ball().pos().x < depth || opp_forward == 0 )
	{
		// stay with best point = opp goal
	}
	else
	{

		if ( wm.ball().pos().x < 35.0 &&  state.ball().pos().x > 5.0 )
		{
                        //
                        // 取「最优空当点」(best_point)。
                        // 候选点（Voronoi 顶点 + 越位线/前插线与 Voronoi 边的交点）以及它们
                        // 到对手/队友的距离都只依赖 world model，因此按 cycle 缓存一次
                        // （见 get_voronoi_candidates()）。原来这部分在每个被评估的 state 里
                        // 重算一次，而本函数一个 cycle 会被 ActionChainGraph 调用数百次。
                        //
                        const std::vector< VoronoiCandidate > & candidates = get_voronoi_candidates( wm );

                        double max_dist = -1000.0;

                        for ( std::vector< VoronoiCandidate >::const_iterator c = candidates.begin(),
                                  end = candidates.end();
                              c != end;
                              ++c )
                        {
                            // back_offset_ = 5.0 (Voronoi 顶点) / 25.0 (线段交点)
                            if ( c->pos_.x < state.ball().pos().x - c->back_offset_
                                 || c->pos_.x > 52.5
                                 || std::fabs( c->pos_.y ) > 34.0 )
                            {
                                continue;
                            }

                            if ( ( c->pos_ - state.ball().pos() ).length() > 34.0 )
                            {
                                continue;
                            }

                            if ( max_dist < c->opp_min_dist_ - c->our_dist_ )
                            {
                                max_dist = c->opp_min_dist_ - c->our_dist_;
                                best_point = c->pos_;
                            }
                        }
		}
	}


    dlog.addText( Logger::TEAM, __FILE__": best point=(%.1f %.1f)", best_point.x, best_point.y);


    point += std::max( 0.0, 40.0 - best_point.dist( state.ball().pos() ) );

    //
    // 传球收益
    // 若本链条结束时球权在队友脚下（即最近一次球权变化来自传球/传中），
    // 且接球点有空间、位置不比当前球位更靠后，则给予小幅奖励。
    // 目的：在「带球推进」与「传给位置更好的队友」价值接近时更倾向于传球。
    // PASS_POSSESSION_BONUS 设为 0.0 即完全关闭本项（便于 A/B 对比）。
    //
    if ( holder_unum != state.self().unum()
         && state.ball().pos().x >= wm.ball().pos().x - PASS_BONUS_MAX_BACKWARD_X )
    {
        double nearest_opp_dist = 1000.0;

        for ( auto o = wm.opponentsFromSelf().begin(), o_end = wm.opponentsFromSelf().end();
              o != o_end;
              ++o )
        {
            const double d = ( (*o)->pos() - state.ball().pos() ).length();
            if ( nearest_opp_dist > d )
            {
                nearest_opp_dist = d;
            }
        }

        if ( nearest_opp_dist > PASS_BONUS_MIN_SPACE )
        {
            point += PASS_POSSESSION_BONUS;
#ifdef DEBUG_PRINT
            dlog.addText( Logger::ACTION_CHAIN,
                          "(eval) pass possession bonus %f (%f) receiver=%d space=%.1f",
                          PASS_POSSESSION_BONUS, point, holder_unum, nearest_opp_dist );
#endif
        }
    }

//  point += std::max( 0.0, 40.0 - ServerParam::i().theirTeamGoalPos().dist( state.ball().pos() ) );

#ifdef DEBUG_PRINT
    dlog.addText( Logger::ACTION_CHAIN,
                  "(eval) eval-center (%d) state ball pos (%f, %f)",
                  evalcenter, state.ball().pos().x, state.ball().pos().y );

    dlog.addText( Logger::ACTION_CHAIN,
                  "(eval) initial value (%f)", point );
#endif

    //
    // add bonus for goal, free situation near offside line
    //
    if ( FieldAnalyzer::can_shoot_from( holder->unum() == state.self().unum(),
                                        holder->pos(),
                                        state.getPlayers( new OpponentOrUnknownPlayerPredicate( state.ourSide() ) ),
                                        VALID_PLAYER_THRESHOLD ) )
    {
        point += 1.0e+6;
#ifdef DEBUG_PRINT
        dlog.addText( Logger::ACTION_CHAIN,
                      "(eval) bonus for goal %f (%f)", 1.0e+6, point );
#endif

        if ( holder_unum == state.self().unum() )
        {
            point += 5.0e+5;
#ifdef DEBUG_PRINT
            dlog.addText( Logger::ACTION_CHAIN,
                          "(eval) bonus for goal self %f (%f)", 5.0e+5, point );
#endif
        }
    }

    return point;
}
