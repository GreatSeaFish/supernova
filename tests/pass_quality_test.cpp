#include "planner/pass_quality.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
int failures = 0;
void check( const bool ok, const char * scenario )
{
    if ( ! ok )
    {
        std::cerr << "FAIL: " << scenario << '\n';
        ++failures;
    }
}
}

int main()
{
    using namespace pass_quality;
    // Legal cutbacks beyond the last defender and level receivers stay available.
    check( ! offside_at_release( 40.0, 42.0, 35.0 ), "cutback behind ball" );
    check( ! offside_at_release( 35.0, 20.0, 35.0 ), "level with defensive line" );
    check( ! offside_at_release( -5.0, -20.0, -10.0 ), "receiver in own half" );
    check( offside_at_release( 35.1, 20.0, 35.0 ), "beyond defender and ball" );
    check( offside_at_release( 42.1, 42.0, 35.0 ), "beyond ball on cutback" );

    // A running receiver is onside now, but crosses before a three-kick release.
    const double receiver_x = 34.5;
    const double velocity_x = 0.8;
    const double decay = 0.4;
    check( ! offside_at_release( receiver_x, 20.0, 35.0 ), "onside at decision" );
    check( offside_at_release( receiver_x + velocity_x * ( 1.0 + decay ),
                               20.0, 35.0 ), "crosses line during preparation" );
    check( preparation_cycles( 1 ) == 0 && preparation_cycles( 2 ) == 1
           && preparation_cycles( 3 ) == 2, "release timing for one/two/three kicks" );

    check( possession_factor( 2.0, 8 ) == 0.0, "marked receiver has no bonus" );
    check( possession_factor( 6.0, 8 ) > possession_factor( 3.0, 8 ),
           "open receiver outranks tightly marked receiver" );
    check( possession_factor( 6.0, 5 ) > possession_factor( 6.0, 20 ),
           "quick circulation outranks equal-space slow pass" );
    check( possession_factor( 2.001, 8 ) < 0.001,
           "no fixed bonus jump just above pressure threshold" );
    check( possession_factor( 1000.0, 0 ) <= 1.0,
           "missing opponents cannot create unbounded reward" );
    for ( int cycle = 0; cycle <= 100; ++cycle )
    {
        double previous = 0.0;
        for ( int cm = 0; cm <= 1000; ++cm )
        {
            const double value = possession_factor( cm / 100.0, cycle );
            check( std::isfinite( value ) && value >= previous && value <= 1.0,
                   "space reward stays finite, bounded and monotonic" );
            previous = value;
        }
    }
    if ( failures ) return EXIT_FAILURE;
    std::cout << "Pass quality scenarios passed\n";
    return EXIT_SUCCESS;
}
