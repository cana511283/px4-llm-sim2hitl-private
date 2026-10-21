/* Plant: one generated plant core (plant_core.h) plus the C Truth. See skysurfer_plant.h. */
#include "skysurfer_plant.h"

#include "plant_geo.h"
#include "plant_carrier.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Mass of the source model (Mass0 in model/FW_SkySurfer_InitialFunction.m). The core does not output
 * it, so the build passes it in (-DPLANT_MASS_KG from results/core_build.json, plant/core/toolchain.py
 * model_defines); plant/core/validate_plant.py checks |F_gravity| = mass * g every step. */
#ifndef PLANT_MASS_KG
#error "PLANT_MASS_KG comes from the model: build through plant/core/compile_core.py or hitl/update.py"
#endif
#define PLANT_GRAVITY_M_S2 9.80665

/* Aerodynamic table breakpoints (import_aerostates_avl.m). */
#define PLANT_TABLE_ALPHA_MIN_DEG -10.0
#define PLANT_TABLE_ALPHA_MAX_DEG 15.0
#define PLANT_TABLE_BETA_MAX_DEG 10.0
#define PLANT_TABLE_MACH_MIN 0.05
#define PLANT_TABLE_MACH_MAX 0.10

static const double kPi = 3.14159265358979323846;

#define PLANT_KTS_M_S (1852.0 / 3600.0) /* the source Velocity Conversion */

struct Plant {
  double origin_latitude_deg, origin_longitude_deg;
  PlantWindConfig wind;
  PlantCoreInitial initial;   /* kept to re-initialise the core when the turbulence seeds change */
  double last_height_m;       /* truth height of the previous step: decides "after airborne" */
  double wind_start_s;
  uint32_t ready, wind_started, stepped, wind_active; /* wind_active: some wind is in effect this step */
  double core[]; /* plant_core_size() bytes; double keeps pointer/double alignment */
};

static PlantCore *core_of(Plant *plant) { return (PlantCore *)plant->core; }

uint32_t plant_api_version(void) { return PLANT_API_VERSION; }

void plant_default_wind_config(PlantWindConfig *c) {
  if (c == NULL) return;
  memset(c, 0, sizeof(*c));
  c->start_mode = PLANT_WIND_AFTER_AIRBORNE;
  c->start_height_m = 5.0;
  c->ramp_s = 5.0;
  PlantCoreWindParameters d;
  plant_core_default_wind_parameters(&d);
  for (int k = 0; k < 3; k++) c->gust_length_m[k] = d.gust_length_m[k];
  for (int k = 0; k < 4; k++) c->turbulence_seed[k] = (uint32_t)d.turbulence_seed[k];
}

static int low_altitude_mode(const PlantWindConfig *c) {
  return c->gust_magnitude_m_s[0] != 0.0 || c->gust_magnitude_m_s[1] != 0.0 || c->gust_magnitude_m_s[2] != 0.0 ||
         c->shear_speed_6m_m_s > 0.0 || c->turbulence_speed_6m_m_s > 0.0;
}

/* Fraction of the configured wind in effect at time_s (0 before the start, then the ramp). */
static double wind_factor(const Plant *plant, double time_s) {
  if (!plant->wind_started) return 0.0;
  if (plant->wind.ramp_s <= 0.0) return 1.0;
  const double f = (time_s - plant->wind_start_s) / plant->wind.ramp_s;
  return f < 0.0 ? 0.0 : (f > 1.0 ? 1.0 : f);
}

/* This aircraft's gust/shear/turbulence settings into the generated core's shared parameters. */
static int32_t apply_wind_parameters(const Plant *plant, double factor) {
  const PlantWindConfig *c = &plant->wind;
  PlantCoreWindParameters p;
  plant_core_default_wind_parameters(&p);
  p.gust_start_s = plant->wind_started ? plant->wind_start_s + c->gust_start_s : 1e9; /* 1e9 s: never */
  for (int k = 0; k < 3; k++) {
    p.gust_length_m[k] = c->gust_length_m[k];
    p.gust_magnitude_m_s[k] = c->gust_magnitude_m_s[k];
  }
  p.shear_speed_6m_m_s = c->shear_speed_6m_m_s * factor;
  p.shear_from_deg = c->shear_from_deg;
  p.turbulence_from_deg = c->turbulence_from_deg;
  for (int k = 0; k < 4; k++) p.turbulence_seed[k] = (double)c->turbulence_seed[k];
  return plant_core_set_wind_parameters(&p);
}

static int32_t valid_wind_config(const PlantWindConfig *c) {
  const double numbers[] = {c->start_height_m, c->start_time_s, c->ramp_s, c->gust_start_s, c->gust_length_m[0],
                            c->gust_length_m[1], c->gust_length_m[2], c->gust_magnitude_m_s[0], c->gust_magnitude_m_s[1],
                            c->gust_magnitude_m_s[2], c->shear_speed_6m_m_s, c->shear_from_deg,
                            c->turbulence_speed_6m_m_s, c->turbulence_from_deg};
  for (size_t k = 0; k < sizeof(numbers) / sizeof(numbers[0]); k++) {
    if (!isfinite(numbers[k])) return 0;
  }
  if (c->start_mode > PLANT_WIND_FROM_START || c->ramp_s < 0.0 || c->gust_start_s < 0.0) return 0;
  if (c->shear_speed_6m_m_s < 0.0 || c->turbulence_speed_6m_m_s < 0.0) return 0;
  for (int k = 0; k < 3; k++) {
    if (c->gust_length_m[k] <= 0.0) return 0;
  }
  return 1;
}

size_t plant_size(void) { return offsetof(Plant, core) + plant_core_size(); }

int32_t plant_init(Plant *plant, const PlantInitialConditions *initial) {
  if (plant == NULL || initial == NULL) return PLANT_ERR_INPUT;
  for (int k = 0; k < 3; k++) {
    if (initial->rates_body_rad_s[k] != 0.0) return PLANT_ERR_INPUT; /* source supports zero only */
  }
  if (initial->site_msl_m != 0.0) return PLANT_ERR_INPUT;
  if (!isfinite(initial->latitude_deg) || !isfinite(initial->longitude_deg) ||
      fabs(initial->latitude_deg) > 90.0 || fabs(initial->longitude_deg) > 180.0) {
    return PLANT_ERR_INPUT;
  }
  PlantCoreInitial ic;
  for (int k = 0; k < 3; k++) {
    ic.velocity_body_m_s[k] = initial->velocity_body_m_s[k];
    ic.euler_deg[k] = initial->euler_rad[k] * (180.0 / kPi);
  }
  ic.height_m = initial->height_agl_m;
  plant->ready = 0;
  plant_default_wind_config(&plant->wind);
  plant->initial = ic;
  plant->last_height_m = ic.height_m;
  plant->wind_start_s = 0.0;
  plant->wind_started = plant->stepped = plant->wind_active = 0;
  if (apply_wind_parameters(plant, 0.0) != PLANT_CORE_OK) return PLANT_ERR_STATE; /* seeds read by init */
  int32_t code = plant_core_init(core_of(plant), &ic);
  if (code != PLANT_CORE_OK) return code == PLANT_CORE_ERR_INPUT ? PLANT_ERR_INPUT : PLANT_ERR_STATE;
  plant->origin_latitude_deg = initial->latitude_deg;
  plant->origin_longitude_deg = initial->longitude_deg;
  plant->ready = 1;
  return PLANT_OK;
}

int32_t plant_configure_wind(Plant *plant, const PlantWindConfig *config) {
  if (plant == NULL || config == NULL || !valid_wind_config(config)) return PLANT_ERR_INPUT;
  if (!plant->ready || plant->stepped) return PLANT_ERR_STATE;
  plant->wind = *config;
  plant->wind.reserved = 0;
  if (apply_wind_parameters(plant, 0.0) != PLANT_CORE_OK) return PLANT_ERR_INPUT;
  return plant_core_init(core_of(plant), &plant->initial) == PLANT_CORE_OK ? PLANT_OK : PLANT_ERR_STATE;
}

/* Core wind input for the step from time_s; starts the wind when its condition is met. */
static int32_t step_wind(Plant *plant, const double steady_ned[3], double time_s) {
  const PlantWindConfig *c = &plant->wind;
  if (!plant->wind_started) {
    const int start = c->start_mode == PLANT_WIND_FROM_START ||
                      (c->start_mode == PLANT_WIND_AT_TIME && time_s >= c->start_time_s) ||
                      (c->start_mode == PLANT_WIND_AFTER_AIRBORNE && plant->last_height_m > c->start_height_m);
    if (start) {
      plant->wind_started = 1;
      plant->wind_start_s = time_s;
    }
  }
  const double factor = wind_factor(plant, time_s);
  if (apply_wind_parameters(plant, factor) != PLANT_CORE_OK) return PLANT_ERR_INPUT;
  double wind[PLANT_CORE_WIND_COUNT] = {0.0, 0.0, 1.0, 0.0};
  plant->wind_active = plant->wind_started && (low_altitude_mode(c) || hypot(steady_ned[0], steady_ned[1]) > 0.0);
  if (plant->wind_active) {
    if (low_altitude_mode(c)) {
      wind[2] = 2.0;
      wind[3] = c->turbulence_speed_6m_m_s * factor;
    } else {
      const double speed = hypot(steady_ned[0], steady_ned[1]) * factor;
      double from_deg = atan2(-steady_ned[1], -steady_ned[0]) * (180.0 / kPi); /* air moves TO; wind comes FROM */
      if (from_deg < 0.0) from_deg += 360.0;
      wind[0] = speed / PLANT_KTS_M_S;
      wind[1] = speed > 0.0 ? from_deg : 0.0;
    }
  }
  return plant_core_set_wind(core_of(plant), wind) == PLANT_CORE_OK ? PLANT_OK : PLANT_ERR_INPUT;
}

static void fill_truth(const Plant *plant, const double *y, double time_s, PlantTruth *t) {
  const double deg = kPi / 180.0;
  memset(t, 0, sizeof(*t));
  t->time_s = time_s;
  for (int k = 0; k < 3; k++) {
    t->position_ned_m[k] = y[PLANT_CORE_POSITION_NED_M + k];
    t->velocity_ned_m_s[k] = y[PLANT_CORE_VELOCITY_NED_M_S + k];
    t->velocity_body_m_s[k] = y[PLANT_CORE_VELOCITY_BODY_M_S + k];
    t->euler_rad[k] = y[PLANT_CORE_EULER_RAD + k];
    t->rates_body_rad_s[k] = y[PLANT_CORE_RATES_BODY_RAD_S + k];
    t->angular_accel_body_rad_s2[k] = y[PLANT_CORE_ANGULAR_ACCEL_RAD_S2 + k];
    t->accel_ned_m_s2[k] = y[PLANT_CORE_ACCEL_NED_M_S2 + k];
    t->force_aero_body_N[k] = y[PLANT_CORE_FORCE_AERO_N + k];
    t->moment_aero_body_Nm[k] = y[PLANT_CORE_MOMENT_AERO_NM + k];
    t->force_thrust_body_N[k] = y[PLANT_CORE_FORCE_THRUST_N + k];
    t->moment_thrust_body_Nm[k] = y[PLANT_CORE_MOMENT_THRUST_NM + k];
    t->force_ground_body_N[k] = y[PLANT_CORE_FORCE_GROUND_N + k];
    t->force_gravity_body_N[k] = y[PLANT_CORE_FORCE_GRAVITY_N + k];
  }
  for (int k = 0; k < 4; k++) {
    t->quaternion_nb[k] = y[PLANT_CORE_QUATERNION + k];
    t->surface_deflection_rad[k] = y[PLANT_CORE_SURFACE_DEG + k] * deg;
  }
  /* Source block Flat Earth to LLA: WGS84, psi = 0, reference height 0 (plant_geo.h). */
  plant_geo_flat_to_lla(t->position_ned_m[0], t->position_ned_m[1], plant->origin_latitude_deg,
                      plant->origin_longitude_deg, &t->latitude_deg, &t->longitude_deg);
  t->height_agl_m = -t->position_ned_m[2];
  t->altitude_msl_m = t->height_agl_m; /* site_msl = 0 */
  t->ground_speed_m_s = hypot(t->velocity_ned_m_s[0], t->velocity_ned_m_s[1]);
  t->course_rad = atan2(t->velocity_ned_m_s[1], t->velocity_ned_m_s[0]);
  if (t->course_rad < 0.0) t->course_rad += 2.0 * kPi;
  if (t->course_rad >= 2.0 * kPi) t->course_rad = 0.0; /* -tiny + 2*pi rounds to 2*pi */

  /* Ideal accelerometer, as the source Measurement/acc: Ab - Vb x w - DCM*[0 0 g], where the
   * gravity term is F_gravity / mass (the source Gravity block is DCM*[0 0 g]*mass). */
  const double *v = t->velocity_body_m_s, *w = t->rates_body_rad_s;
  const double vxw[3] = {v[1] * w[2] - v[2] * w[1], v[2] * w[0] - v[0] * w[2], v[0] * w[1] - v[1] * w[0]};
  for (int k = 0; k < 3; k++) {
    t->specific_force_body_m_s2[k] =
        y[PLANT_CORE_ACCEL_BODY_M_S2 + k] - vxw[k] - t->force_gravity_body_N[k] / PLANT_MASS_KG;
  }

  t->airspeed_true_m_s = y[PLANT_CORE_AIRSPEED_M_S];
  t->alpha_rad = y[PLANT_CORE_ALPHA_DEG] * deg;
  t->beta_rad = y[PLANT_CORE_BETA_DEG] * deg;
  t->mach = y[PLANT_CORE_MACH];
  t->air_density_kg_m3 = y[PLANT_CORE_AIR_DENSITY_KG_M3];
  t->static_pressure_pa = y[PLANT_CORE_STATIC_PRESSURE_PA];
  t->air_temperature_k = y[PLANT_CORE_AIR_TEMPERATURE_K];
  t->dynamic_pressure_pa = 0.5 * t->air_density_kg_m3 * t->airspeed_true_m_s * t->airspeed_true_m_s;
  if (plant->wind_active) {
    /* The wind actually in effect (steady, shear, gust, turbulence): ground velocity - R(q) * air velocity. */
    const double *q = t->quaternion_nb, *a = &y[PLANT_CORE_AIR_VELOCITY_BODY_M_S];
    const double w0 = q[0], x = q[1], yq = q[2], z = q[3];
    const double r[3][3] = {{1 - 2 * (yq * yq + z * z), 2 * (x * yq - w0 * z), 2 * (x * z + w0 * yq)},
                            {2 * (x * yq + w0 * z), 1 - 2 * (x * x + z * z), 2 * (yq * z - w0 * x)},
                            {2 * (x * z - w0 * yq), 2 * (yq * z + w0 * x), 1 - 2 * (x * x + yq * yq)}};
    for (int k = 0; k < 3; k++) {
      t->wind_ned_m_s[k] = t->velocity_ned_m_s[k] - (r[k][0] * a[0] + r[k][1] * a[1] + r[k][2] * a[2]);
    }
  }
  t->throttle_actual = y[PLANT_CORE_THROTTLE];

  const double *fg = t->force_ground_body_N;
  t->on_ground = (fg[0] != 0.0 || fg[1] != 0.0 || fg[2] != 0.0) ? 1u : 0u;
  t->aero_active = y[PLANT_CORE_AERO_ACTIVE] > 0.5 ? 1u : 0u;
  if (t->aero_active) {
    const double a = y[PLANT_CORE_ALPHA_DEG], b = y[PLANT_CORE_BETA_DEG];
    if (a < PLANT_TABLE_ALPHA_MIN_DEG || a > PLANT_TABLE_ALPHA_MAX_DEG || fabs(b) > PLANT_TABLE_BETA_MAX_DEG) t->flags |= 1u;
    if (t->mach < PLANT_TABLE_MACH_MIN || t->mach > PLANT_TABLE_MACH_MAX) t->flags |= 2u;
  }
}

int32_t plant_step(Plant *plant, const PlantInput *input, PlantTruth *truth) {
  if (plant == NULL || input == NULL || truth == NULL) return PLANT_ERR_INPUT;
  if (!plant->ready) return PLANT_ERR_STATE;
  for (int k = 0; k < 3; k++) {
    if (!isfinite(input->wind_ned_m_s[k])) return PLANT_ERR_INPUT;
  }
  const PlantActuatorCommand *c = &input->command;
  double controls[PLANT_CORE_CONTROL_COUNT] = {0};
  controls[PLANT_CORE_CTRL_AILERON_LEFT] = c->aileron_left;
  controls[PLANT_CORE_CTRL_AILERON_RIGHT] = c->aileron_right;
  controls[PLANT_CORE_CTRL_ELEVATOR] = c->elevator;
  controls[PLANT_CORE_CTRL_RUDDER] = c->rudder;
  controls[PLANT_CORE_CTRL_THROTTLE] = c->throttle;
  const double time_s = plant_core_time_s(core_of(plant));
  int32_t wind_code = step_wind(plant, input->wind_ned_m_s, time_s);
  if (wind_code != PLANT_OK) return wind_code;
  plant->stepped = 1;
  double y[PLANT_CORE_SNAPSHOT_COUNT];
  int32_t code = plant_core_step(core_of(plant), controls, y);
  if (code == PLANT_CORE_ERR_INPUT) return PLANT_ERR_INPUT;
  if (code == PLANT_CORE_ERR_DIVERGED) return PLANT_ERR_DIVERGED;
  if (code != PLANT_CORE_OK) return PLANT_ERR_STATE;
  fill_truth(plant, y, time_s, truth);
  plant->last_height_m = truth->height_agl_m;
  return PLANT_OK;
}

int32_t plant_carrier_sample(const PlantCarrierConfig *c, double t, PlantCarrierSample *s) {
  if (!c || !s || !isfinite(t) || t < 0.0) return PLANT_ERR_INPUT;
  const double values[] = {c->wait_s, c->climb_s, c->cruise_s, c->height_m, c->speed_m_s};
  for (int k = 0; k < 5; k++) if (!isfinite(values[k])) return PLANT_ERR_INPUT;
  if (c->wait_s < 0 || c->climb_s < 1 || c->cruise_s < 0 ||
      c->height_m <= 0 || c->height_m > 10000 || c->speed_m_s <= 0 || c->speed_m_s > 200)
    return PLANT_ERR_INPUT;
  memset(s, 0, sizeof(*s));
  s->rigid[6] = 1.0;
  const double end = c->wait_s + c->climb_s;
  s->carried = t < end + c->cruise_s;
  if (t <= c->wait_s) return PLANT_OK;
  if (t < end) {
    const double u = (t - c->wait_s) / c->climb_s;
    const double u2 = u*u, u3 = u2*u, u4 = u3*u, u5 = u4*u;
    const double smooth = 10*u3 - 15*u4 + 6*u5;
    const double ds = 30*u2 - 60*u3 + 30*u4;
    const double dds = 60*u - 180*u2 + 120*u3;
    s->rigid[0] = c->speed_m_s*c->climb_s*(2.5*u4 - 3*u5 + u5*u);
    s->rigid[2] = -c->height_m*smooth;
    s->rigid[3] = c->speed_m_s*smooth;
    s->rigid[5] = -c->height_m*ds/c->climb_s;
    s->accel_ned_m_s2[0] = c->speed_m_s*ds/c->climb_s;
    s->accel_ned_m_s2[2] = -c->height_m*dds/(c->climb_s*c->climb_s);
  } else {
    s->rigid[0] = c->speed_m_s*(0.5*c->climb_s + t-end);
    s->rigid[2] = -c->height_m;
    s->rigid[3] = c->speed_m_s;
  }
  return PLANT_OK;
}

int32_t plant_carrier_step(Plant *plant, const PlantInput *input,
                           const PlantCarrierConfig *config, PlantTruth *truth) {
  if (!plant || !plant->ready) return PLANT_ERR_INPUT;
  return plant_carrier_step_at(plant, input, config, plant_core_time_s(core_of(plant)), 1, truth);
}

int32_t plant_carrier_step_at(Plant *plant, const PlantInput *input,
    const PlantCarrierConfig *config, double time, uint32_t advance, PlantTruth *truth) {
  if (!plant || !input || !truth || !plant->ready || advance > 1) return PLANT_ERR_INPUT;
  /* Phase one explicitly excludes wind, including delayed wind, not merely its
   * instantaneous value. Never silently replace configured weather. */
  if (low_altitude_mode(&plant->wind) || input->wind_ned_m_s[0] != 0 ||
      input->wind_ned_m_s[1] != 0 || input->wind_ned_m_s[2] != 0) return PLANT_ERR_INPUT;
  if (plant->initial.height_m != 0 || plant->initial.velocity_body_m_s[0] != 0 ||
      plant->initial.velocity_body_m_s[1] != 0 || plant->initial.velocity_body_m_s[2] != 0 ||
      plant->initial.euler_deg[0] != 0 || plant->initial.euler_deg[1] != 0 || plant->initial.euler_deg[2] != 0)
    return PLANT_ERR_INPUT;
  PlantCarrierSample now, next;
  if (plant_carrier_sample(config, time, &now) ||
      plant_carrier_sample(config, time + (advance ? PLANT_CORE_DT_S : 0.0), &next)) return PLANT_ERR_INPUT;
  const int32_t code = plant_step(plant, input, truth);
  if (code != PLANT_OK || !now.carried) return code;
  /* The prescribed next state was installed after the previous step. Thus the
   * ordinary snapshot already has consistent position, velocity and air data. */
  for (int k = 0; k < 3; k++) {
    truth->accel_ned_m_s2[k] = now.accel_ned_m_s2[k];
    truth->specific_force_body_m_s2[k] = now.accel_ned_m_s2[k] - (k == 2 ? PLANT_GRAVITY_M_S2 : 0.0);
    truth->angular_accel_body_rad_s2[k] = 0.0;
  }
  truth->on_ground = time <= config->wait_s;
  return plant_core_set_rigid_state(core_of(plant), next.rigid) == PLANT_CORE_OK ? PLANT_OK : PLANT_ERR_STATE;
}

int32_t plant_copy(Plant *dst, const Plant *src) {
  if (dst == NULL || src == NULL) return PLANT_ERR_INPUT;
  if (dst == src) return PLANT_OK;
  dst->origin_latitude_deg = src->origin_latitude_deg;
  dst->origin_longitude_deg = src->origin_longitude_deg;
  dst->wind = src->wind;
  dst->initial = src->initial;
  dst->last_height_m = src->last_height_m;
  dst->wind_start_s = src->wind_start_s;
  dst->ready = src->ready;
  dst->wind_started = src->wind_started;
  dst->stepped = src->stepped;
  dst->wind_active = src->wind_active;
  return plant_core_copy(core_of(dst), (const PlantCore *)src->core) == PLANT_CORE_OK ? PLANT_OK : PLANT_ERR_STATE;
}
