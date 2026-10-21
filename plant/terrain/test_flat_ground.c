#include "plant_flat_ground.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    double r;
    const double down[] = {0, 0, 1};
    const double tilted[] = {0.6, 0, 0.8};
    const double scaled[] = {6, 0, 8};
    const double horizontal[] = {1, 0, 0};
    const double up[] = {0, 0, -1};
    const double bad[] = {NAN, 0, 1};
    assert(plant_flat_ground_down_m(1000, -400) == 0);
    assert(plant_flat_ground_agl_m(-100) == 100);
    assert(plant_flat_ground_agl_m(0.001) == -0.001);
    assert(plant_flat_ground_range_m(-100, down, 0.2, 150, &r) && r == 100);
    /* Known 3-4-5 triangle: slant range must exceed vertical clearance. */
    assert(plant_flat_ground_range_m(-100, tilted, 0.2, 150, &r) && fabs(r-125) < 1e-10);
    assert(plant_flat_ground_range_m(-100, scaled, 0.2, 150, &r) && fabs(r-125) < 1e-10);
    assert(!plant_flat_ground_range_m(-100, tilted, 0.2, 120, &r) && isnan(r));
    assert(plant_flat_ground_range_m(-100, down, 100, 100, &r));
    assert(!plant_flat_ground_range_m(0, down, 0.2, 150, &r) && isnan(r));
    assert(!plant_flat_ground_range_m(0.001, down, 0, 150, &r));
    assert(!plant_flat_ground_range_m(-100, horizontal, 0, 150, &r));
    assert(!plant_flat_ground_range_m(-100, up, 0, 150, &r));
    assert(!plant_flat_ground_range_m(-100, bad, 0, 150, &r));
    assert(!plant_flat_ground_range_m(-100, down, 150, 0, &r));
    assert(!plant_flat_ground_range_m(NAN, down, 0, 150, &r));
    puts("flat ground geometry: PASS");
    return 0;
}
