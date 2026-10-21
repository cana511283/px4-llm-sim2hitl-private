#ifndef PLANT_CORE_H
#define PLANT_CORE_H
/*
 * SkySurfer plant core: the 6DOF generated from FW_SkySurfer_HIL.slx (plant/core/build_core.m,
 * ERT Reusable function) behind a small reentrant C API.
 *
 *   controls[16] in  ->  CoreSnapshot[72] out (states, kinematics, forces/moments, air data,
 *                        atmosphere, actuators; layout below)
 *
 * Conventions
 *   - One PlantCore per aircraft. Allocate plant_core_size() bytes (malloc alignment) per instance.
 *     Instances are independent: initial conditions, states and inputs live in the instance.
 *     Aircraft parameters (mass, aero tables, ...) are generated constants shared by all instances.
 *   - Instances may run in one process, stepped one call at a time. Not for concurrent calls from
 *     several threads (the generated code also initialises shared non-finite constants).
 *   - plant_core_step() writes the snapshot sampled at t_k, then integrates to t_k + dt (ode4,
 *     dt = 0.004 s): the same output phase as the HIL ABI.
 *   - Units are the source model's: SI, except angles marked _deg.
 *   - Standard C99 only.
 */
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define PLANT_CORE_API_VERSION 0x00010200u /* 1.2: wind input and wind parameters; 1.1: state access, f(x, u) */
#define PLANT_CORE_DT_S 0.004
#define PLANT_CORE_CONTROL_COUNT 16
#define PLANT_CORE_SNAPSHOT_COUNT 72
#define PLANT_CORE_STATE_COUNT 37
#define PLANT_CORE_WIND_COUNT 4

#define PLANT_CORE_OK 0
#define PLANT_CORE_ERR_STATE 2    /* not initialised, or diverged earlier: re-initialise */
#define PLANT_CORE_ERR_DIVERGED 3 /* non-finite snapshot; the instance must be re-initialised */
#define PLANT_CORE_ERR_INPUT 4    /* null pointer or non-finite input */

/* controls[] index (same as the HIL ABI controls[0..4]); 5..15 are accepted and unused. */
enum {
  PLANT_CORE_CTRL_AILERON_LEFT = 0,
  PLANT_CORE_CTRL_AILERON_RIGHT = 1,
  PLANT_CORE_CTRL_ELEVATOR = 2,
  PLANT_CORE_CTRL_RUDDER = 3,
  PLANT_CORE_CTRL_THROTTLE = 4
};

/* CoreSnapshot layout: first index of each group (0-based). Frames: NED earth (origin at the
 * initial ground point), FRD body. Forces in N, moments in N*m, all in body axes. */
enum {
  PLANT_CORE_POSITION_NED_M = 0,        /* [3] Xe */
  PLANT_CORE_VELOCITY_BODY_M_S = 3,     /* [3] Vb */
  PLANT_CORE_QUATERNION = 6,            /* [4] q0..q3, body -> NED */
  PLANT_CORE_RATES_BODY_RAD_S = 10,     /* [3] p q r */
  PLANT_CORE_VELOCITY_NED_M_S = 13,     /* [3] Ve */
  PLANT_CORE_EULER_RAD = 16,            /* [3] roll pitch yaw; yaw is the atan2 principal value [-pi, pi] */
  PLANT_CORE_ANGULAR_ACCEL_RAD_S2 = 19, /* [3] pdot qdot rdot */
  PLANT_CORE_ACCEL_BODY_M_S2 = 22,      /* [3] Ab: dVb/dt */
  PLANT_CORE_ACCEL_NED_M_S2 = 25,       /* [3] Abe */
  PLANT_CORE_FORCE_AERO_N = 28,         /* [3] after the airspeed > 5 m/s switch (what enters the sum) */
  PLANT_CORE_MOMENT_AERO_NM = 31,       /* [3] same */
  PLANT_CORE_FORCE_THRUST_N = 34,       /* [3] */
  PLANT_CORE_MOMENT_THRUST_NM = 37,     /* [3] */
  PLANT_CORE_FORCE_GROUND_N = 40,       /* [3] */
  PLANT_CORE_FORCE_GRAVITY_N = 43,      /* [3] */
  PLANT_CORE_FORCE_TOTAL_N = 46,        /* [3] aero + thrust + ground + gravity */
  PLANT_CORE_MOMENT_TOTAL_NM = 49,      /* [3] aero + thrust */
  PLANT_CORE_AIRSPEED_M_S = 52,         /* true airspeed */
  PLANT_CORE_ALPHA_DEG = 53,
  PLANT_CORE_BETA_DEG = 54,
  PLANT_CORE_MACH = 55,
  PLANT_CORE_AIR_VELOCITY_BODY_M_S = 56, /* [3] UVWc: body velocity relative to the air */
  PLANT_CORE_AIR_RATES_BODY_RAD_S = 59,  /* [3] PQRc: body rates relative to the air */
  PLANT_CORE_AIR_TEMPERATURE_K = 62,     /* ISA */
  PLANT_CORE_SPEED_OF_SOUND_M_S = 63,
  PLANT_CORE_STATIC_PRESSURE_PA = 64,
  PLANT_CORE_AIR_DENSITY_KG_M3 = 65,
  PLANT_CORE_SURFACE_DEG = 66,           /* [4] aileron_left aileron_right elevator rudder, after servo dynamics */
  PLANT_CORE_THROTTLE = 70,              /* after the throttle lag */
  PLANT_CORE_AERO_ACTIVE = 71            /* 1 when airspeed > 5 m/s */
};

/* Continuous states x[PLANT_CORE_STATE_COUNT], in the generated order (ode4 integrates these):
 * position NED m [3], body velocity m/s [3], quaternion body -> NED [4], body rates rad/s [3],
 * the five actuator transfer-function states of the source ACTUATOR subsystem, driven by (checked with
 * plant_core_eval) throttle, right aileron, left aileron, elevator, rudder, the sixteen Dryden turbulence
 * filter states (w, w, v, v, u, r, q, p filters, two each; zero without turbulence), then three gust
 * distances (zero without gusts). */
enum {
  PLANT_CORE_X_POSITION_NED = 0,
  PLANT_CORE_X_VELOCITY_BODY = 3,
  PLANT_CORE_X_QUATERNION = 6,
  PLANT_CORE_X_RATES_BODY = 10,
  PLANT_CORE_X_ACTUATORS = 13, /* [5] throttle, aileron right, aileron left, elevator, rudder */
  PLANT_CORE_X_TURBULENCE = 18, /* [16] */
  PLANT_CORE_X_GUST = 34        /* [3] */
};

/* Initial conditions in source units (exactly what the source model takes). */
typedef struct {
  double velocity_body_m_s[3]; /* U0 V0 W0 */
  double height_m;             /* H0, above the ground datum */
  double euler_deg[3];         /* Phi0 Theta0 Psi0 */
} PlantCoreInitial;

typedef struct PlantCore PlantCore;

uint32_t plant_core_api_version(void);
size_t plant_core_size(void);

/* Initialise (or re-initialise) an instance at t = 0. */
int32_t plant_core_init(PlantCore *core, const PlantCoreInitial *initial);

/* Write the snapshot at t_k into snapshot (may be NULL), then advance one step with controls held. */
int32_t plant_core_step(PlantCore *core, const double controls[PLANT_CORE_CONTROL_COUNT],
                      double snapshot[PLANT_CORE_SNAPSHOT_COUNT]);

/* Copy the complete state of src into dst (save, restore, branch). dst needs plant_core_size() bytes. */
int32_t plant_core_copy(PlantCore *dst, const PlantCore *src);

/* Read the continuous states. */
int32_t plant_core_get_state(const PlantCore *core, double x[PLANT_CORE_STATE_COUNT]);

/* Prescribed carrier boundary, after at least one normal step. Only rigid-body states
 * [position, velocity_body, quaternion, rates] change; time, actuator states, limiter
 * history and wind states are preserved. Used after a carried step for t_(k+1). */
int32_t plant_core_set_rigid_state(PlantCore *core, const double rigid[13]);

/* State derivative f(x, u) for trim and linearisation. Writes x into core, applies controls and runs one
 * normal step: the snapshot (may be NULL) is the output at x, and xdot is the first ode4 stage f(t_k, x),
 * the same code path as the simulation. The core is advanced one step afterwards, so call this on a copy
 * (plant_core_copy) when the instance must stay unchanged. Continuing to step the core afterwards with the
 * same controls is an ordinary simulation started from x. */
int32_t plant_core_eval(PlantCore *core, const double x[PLANT_CORE_STATE_COUNT],
                      const double controls[PLANT_CORE_CONTROL_COUNT], double xdot[PLANT_CORE_STATE_COUNT],
                      double snapshot[PLANT_CORE_SNAPSHOT_COUNT]);

/* Wind, per instance, held from step to step, in source units:
 *   wind[0] VelWind kts, wind[1] PsiWind deg (the direction the wind comes FROM: 0 = from north),
 *   wind[2] IdxWind: 1 = steady horizontal wind (wind[0], wind[1]) only,
 *                    2 = wind shear (mean wind growing with height) + discrete gust + Dryden turbulence;
 *                        the steady wind of mode 1 is NOT added in mode 2 (the source model's design),
 *   wind[3] turbulence scale = the wanted Dryden W20 (wind speed at 6 m) in m/s: the core is generated at
 *           the reference W20 = 1 m/s and multiplies the Dryden outputs by this; exact below 1000 ft.
 * plant_core_init sets {0, 0, 1, 0}: no wind, as the source model. */
int32_t plant_core_set_wind(PlantCore *core, const double wind[PLANT_CORE_WIND_COUNT]);

/* Gust, shear and turbulence settings. In the generated code they are tunable parameters in one global
 * shared by every instance, so a caller with several instances sets them before each instance's step
 * (plant_truth.c does). The turbulence seeds are read by plant_core_init only. */
typedef struct {
  double gust_start_s;            /* model time the discrete gust starts */
  double gust_length_m[3];        /* x y z gust lengths (> 0) */
  double gust_magnitude_m_s[3];   /* x y z gust amplitudes; zero: no gust */
  double shear_speed_6m_m_s;      /* wind speed at 6 m (20 ft) of the shear model; zero: no mean wind in mode 2 */
  double shear_from_deg;          /* direction the shear wind comes from */
  double turbulence_from_deg;     /* direction of the turbulence axes (normally the mean wind's) */
  double turbulence_seed[4];      /* Dryden white-noise seeds (integers) */
} PlantCoreWindParameters;
void plant_core_default_wind_parameters(PlantCoreWindParameters *parameters);
int32_t plant_core_set_wind_parameters(const PlantCoreWindParameters *parameters);

/* Time of the next snapshot, in steps and seconds. */
uint64_t plant_core_step_index(const PlantCore *core);
double plant_core_time_s(const PlantCore *core);

#ifdef __cplusplus
}
#endif
#endif
