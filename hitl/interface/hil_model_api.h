#ifndef HIL_MODEL_API_H
#define HIL_MODEL_API_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define HIL_ABI_VERSION 0x00010001u
#define HIL_RAW_COUNT 58u
#if defined(_WIN32)
#if defined(HIL_BUILD_DLL)
#define HIL_EXPORT __declspec(dllexport)
#else
#define HIL_EXPORT __declspec(dllimport)
#endif
#else
#define HIL_EXPORT __attribute__((visibility("default")))
#endif
/* Capability bits are stable across recipes; raw truth remains optional. */
#define HIL_CAP_IMU 1u
#define HIL_CAP_GPS 2u
#define HIL_CAP_PRESSURE 4u
#define HIL_CAP_TRUTH 8u
#define HIL_CAP_RESET 16u
#define HIL_CAP_RUNTIME_INITIAL 32u
#define HIL_CAP_CARRIER 512u /* optional armed-gated prescribed carrier */
#define HIL_CAP_WIND 256u /* hil_model_configure_wind */
/* initialize/reset/step/terminate require serialized calls. Reset after worker stop. */
/* Natural C alignment. No packing. All arrays have fixed dimensions. */
typedef struct {
  uint32_t abi_version, struct_size;
  double height_m, body_velocity_m_s[3], euler_deg[3], latitude_deg, longitude_deg;
  /* Explicit GPS datum conversion: MSL = WGS84 ellipsoid height - separation. */
  double geoid_separation_m;
  uint32_t seed_policy; /* 0: the fixed default sensor seed. Other values unsupported. */
  uint32_t reserved[7];
} HilInitialConditions;
typedef struct {
  uint32_t abi_version, struct_size;
  double controls[16];
  uint64_t step_index, actuator_age_ns, input_sequence;
  uint32_t actuator_valid, actuator_new, fc_armed, reserved;
} HilModelInput;
typedef struct {
  uint32_t abi_version, struct_size;
  uint64_t model_time_ns, imu_sample_time_ns, gps_sample_time_ns;
  double accel_m_s2[3], gyro_rad_s[3], magnetic_uT[3];
  double absolute_pressure_pa, differential_pressure_pa;
  double gps_lla_msl[3], velocity_ned_m_s[3], ground_speed_m_s, course_deg;
  double raw_source[HIL_RAW_COUNT];
  uint32_t imu_valid, imu_updated, gps_valid, gps_updated;
  uint32_t pressure_height_valid, temperature_valid, fields_updated, reserved;
} HilModelOutput;
typedef struct {
  uint32_t abi_version, struct_size;
  uint32_t initial_size, input_size, output_size, output_phase;
  uint64_t fixed_step_ns, imu_period_ns, gps_period_ns;
  char aircraft_id[32], build_id[64], source_sha256[65], platform[32];
  uint32_t raw_count, seed_policy, instance_count, reserved;
  uint64_t capabilities, initial_supported;
} HilModelInfo;
HIL_EXPORT uint32_t hil_model_get_abi_version(void);
HIL_EXPORT int32_t hil_model_get_info(HilModelInfo *out);
HIL_EXPORT int32_t hil_model_initialize(const HilInitialConditions *initial);
HIL_EXPORT int32_t hil_model_step(const HilModelInput *input,HilModelOutput *output);
HIL_EXPORT int32_t hil_model_reset(const HilInitialConditions *initial);
/* Optional (HIL_CAP_WIND): wind of the run, after initialize/reset and before the first step.
 * steady_ned_m_s is the NED velocity of the air (direction it blows TO; vertical part ignored). The model has
 * two wind modes: steady, or shear + gust + turbulence (any of these non-zero; the steady wind is then
 * ignored). start_mode 0: after airborne (height > start_height_m), 1: at start_time_s, 2: from t = 0.
 * Speeds m/s, directions deg (the direction the wind comes FROM), turbulence = Dryden W20 (at 6 m). */
typedef struct {
  uint32_t abi_version, struct_size;
  double steady_ned_m_s[3];
  uint32_t start_mode, reserved0;
  double start_height_m, start_time_s, ramp_s, gust_start_s;
  double gust_length_m[3], gust_magnitude_m_s[3];
  double shear_speed_6m_m_s, shear_from_deg, turbulence_speed_6m_m_s, turbulence_from_deg;
  uint32_t turbulence_seed[4];
} HilWindConfig;
HIL_EXPORT int32_t hil_model_configure_wind(const HilWindConfig *wind);
/* Optional carrier extension. Configure once after reset, before stepping.
 * Only ground/zero-velocity/zero-Euler initial conditions and no wind. Arming
 * must be confirmed in HilModelInput.fc_armed; unknown is never armed.
 * output.reserved, with HIL_CAP_CARRIER: 0 disabled, 1 waiting for arm,
 * 2 attached, 3 released. No output/header/clock layout change. */
typedef struct {
  uint32_t abi_version, struct_size;
  double wait_s, climb_s, cruise_s, height_m, speed_m_s;
} HilCarrierConfig;
HIL_EXPORT int32_t hil_model_configure_carrier(const HilCarrierConfig *carrier);
HIL_EXPORT int32_t hil_model_terminate(void);
HIL_EXPORT int32_t hil_model_get_last_error(char *buffer,uint32_t capacity);
#ifdef __cplusplus
}
#endif
#endif
