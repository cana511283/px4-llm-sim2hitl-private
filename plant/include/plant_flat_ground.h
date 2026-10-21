#ifndef PLANT_FLAT_GROUND_H
#define PLANT_FLAT_GROUND_H

#include <math.h>

/* Infinite horizontal plane at NED down=0, matching the generated contact model.
 * N/E and D are metres relative to the plant's ground datum, NOT FC Home.
 * Header-only geometry: no state, allocation, clocks or random numbers.
 */
static inline double plant_flat_ground_down_m(double north_m, double east_m)
{
    (void)north_m;
    (void)east_m;
    return 0.0;
}

/* Signed vertical clearance, including small contact-model penetration. */
static inline double plant_flat_ground_agl_m(double down_m)
{
    return -down_m;
}

/* Slant range along an arbitrary NED ray (need not be normalized).
 * Returns 1 only for a forward intersection within inclusive sensor limits.
 * On failure writes NAN, so callers cannot reuse a previous valid distance.
 * A sensor below the plane has no valid downward measurement.
 */
static inline int plant_flat_ground_range_m(double sensor_down_m,
        const double ray_ned[3], double min_range_m, double max_range_m,
        double *range_m)
{
    double norm, range;
    if (!range_m) return 0;
    *range_m = NAN;
    if (!ray_ned || !isfinite(sensor_down_m) || sensor_down_m > 0.0 ||
        !isfinite(min_range_m) || !isfinite(max_range_m) ||
        min_range_m < 0.0 || max_range_m < min_range_m ||
        !isfinite(ray_ned[0]) || !isfinite(ray_ned[1]) ||
        !isfinite(ray_ned[2]) || ray_ned[2] <= 0.0) return 0;
    norm = hypot(hypot(ray_ned[0], ray_ned[1]), ray_ned[2]);
    if (!isfinite(norm) || norm <= 0.0) return 0;
    range = (-sensor_down_m) * (norm / ray_ned[2]);
    if (!isfinite(range) || range < min_range_m || range > max_range_m) return 0;
    *range_m = range;
    return 1;
}
#endif
