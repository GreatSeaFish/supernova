// Shared, dependency-free policy for pass planning and evaluation.
#ifndef SUPERNOVA_PASS_QUALITY_H
#define SUPERNOVA_PASS_QUALITY_H

#include <algorithm>
#include <cmath>

namespace pass_quality {

inline int preparation_cycles( const int kick_count )
{
    return std::max( 0, kick_count - 1 );
}

// Offside is judged at release, against both the ball and the defensive line.
inline bool offside_at_release( const double receiver_x,
                                const double ball_x,
                                const double offside_line_x )
{
    return receiver_x > std::max( 0.0, std::max( ball_x, offside_line_x ) );
}

// Continuous [0,1] reward: pressure and long waits reduce possession value.
inline double possession_factor( const double space, const unsigned long cycles )
{
    const double space_factor = std::max( 0.0, std::min( 1.0, ( space - 2.0 ) / 4.0 ) );
    return space_factor / ( 1.0 + 0.04 * cycles );
}

} // namespace pass_quality

#endif
