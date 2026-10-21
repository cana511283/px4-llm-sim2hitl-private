/* SkySurfer HIL model ABI (hil_model_api.h) on top of the plant core, C Truth and C sensors
 * (plant/include/skysurfer_plant.h, plant_sensors.h). One instance; calls are serialized by the host. */
#include "hil_model_api.h"
#include "skysurfer_plant.h"
#include "plant_sensors.h"
#include "plant_carrier.h"
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifndef HIL_SOURCE_HASH
#define HIL_SOURCE_HASH "unknown"
#endif
#ifndef HIL_PLATFORM
#define HIL_PLATFORM "unknown"
#endif
#ifndef HIL_BUILD_ID
#define HIL_BUILD_ID "unidentified"
#endif
/* seed_policy 0: the fixed default seed of the sensor noise (reproducible runs). */
#define HIL_DEFAULT_SEED 1u
static const double kPi=3.14159265358979323846, kG0=9.80665;
static Plant *plant=NULL;
static PlantSensors *sensors=NULL;
static int active=0, carrier_enabled=0, carrier_started=0, wind_configured=0;
static int carrier_ground_initial=0;
static uint64_t carrier_arm_step=0;
static PlantCarrierConfig carrier;

static uint64_t next_step=0;
static double held_controls[16], geoid_separation, steady_wind[3];
static char last_error[256];
static int32_t fail(int32_t code,const char *msg) {snprintf(last_error,sizeof(last_error),"%s",msg);return code;}
static int valid_header(uint32_t a,uint32_t s,size_t expected) {return a==HIL_ABI_VERSION && s==expected;}
_Static_assert(sizeof(double)==8,"Requires IEEE double storage");
_Static_assert(offsetof(HilModelInput,controls)==8,"Unexpected ABI alignment");
_Static_assert(sizeof(HilInitialConditions)==120,"Unexpected initial ABI");
_Static_assert(sizeof(HilModelInput)==176,"Unexpected input ABI");
_Static_assert(sizeof(HilModelOutput)==680,"Unexpected output ABI");
_Static_assert(sizeof(HilWindConfig)==168,"Unexpected wind ABI");
uint32_t hil_model_get_abi_version(void) {return HIL_ABI_VERSION;}
int32_t hil_model_get_info(HilModelInfo *o) {
  if(!o || !valid_header(o->abi_version,o->struct_size,sizeof(*o))) return fail(4,"info ABI mismatch");
  memset(o,0,sizeof(*o));o->abi_version=HIL_ABI_VERSION;o->struct_size=sizeof(*o);
  o->initial_size=sizeof(HilInitialConditions);o->input_size=sizeof(HilModelInput);o->output_size=sizeof(HilModelOutput);
  o->output_phase=0; /* t_k, sampled BEFORE the ode4 update */
  o->fixed_step_ns=o->imu_period_ns=4000000;o->gps_period_ns=100000000;
  snprintf(o->aircraft_id,sizeof(o->aircraft_id),"skysurfer");
  snprintf(o->build_id,sizeof(o->build_id),"%s",HIL_BUILD_ID);
  snprintf(o->source_sha256,sizeof(o->source_sha256),"%s",HIL_SOURCE_HASH);
  snprintf(o->platform,sizeof(o->platform),"%s",HIL_PLATFORM);o->raw_count=HIL_RAW_COUNT;o->instance_count=1;o->capabilities=0x3f|HIL_CAP_WIND|HIL_CAP_CARRIER;o->initial_supported=0x7f;
  return 0;
}
static int32_t initialize(const HilInitialConditions *i) {
  if(!i || !valid_header(i->abi_version,i->struct_size,sizeof(*i))) return fail(4,"initial ABI mismatch");
  if(i->seed_policy!=0) return fail(4,"Only the fixed default sensor seed (seed_policy 0) is supported");
  if(!isfinite(i->height_m)||i->height_m<0||i->height_m>10000||!isfinite(i->latitude_deg)||fabs(i->latitude_deg)>90||!isfinite(i->longitude_deg)||fabs(i->longitude_deg)>180||!isfinite(i->geoid_separation_m)) return fail(4,"Invalid initial conditions");
  for(int k=0;k<3;k++) if(!isfinite(i->body_velocity_m_s[k])||!isfinite(i->euler_deg[k])||fabs(i->body_velocity_m_s[k])>200) return fail(4,"Invalid velocity or Euler angles");
  if(fabs(i->euler_deg[0])>180||fabs(i->euler_deg[1])>80||fabs(i->euler_deg[2])>360) return fail(4,"Euler initial condition outside supported range");
  if(!plant) plant=malloc(plant_size());
  if(!sensors) sensors=malloc(plant_sensors_size());
  if(!plant||!sensors) return fail(5,"Out of memory");
  PlantInitialConditions ic;memset(&ic,0,sizeof(ic));
  ic.height_agl_m=i->height_m;ic.latitude_deg=i->latitude_deg;ic.longitude_deg=i->longitude_deg;ic.seed=HIL_DEFAULT_SEED;
  for(int k=0;k<3;k++) {ic.velocity_body_m_s[k]=i->body_velocity_m_s[k];ic.euler_rad[k]=i->euler_deg[k]*(kPi/180.0);}
  carrier_enabled=carrier_started=wind_configured=0;
  carrier_ground_initial=i->height_m==0;
  for(int k=0;k<3;k++) if(i->body_velocity_m_s[k]!=0 || i->euler_deg[k]!=0) carrier_ground_initial=0;
  active=0;
  if(plant_init(plant,&ic)!=PLANT_OK) return fail(4,"Plant rejected the initial conditions");
  PlantSensorConfig config;plant_sensors_default_config(&config);
  if(plant_sensors_init(sensors,&config,ic.seed)!=PLANT_OK) return fail(4,"Sensors rejected the configuration");
  geoid_separation=i->geoid_separation_m;
  memset(held_controls,0,sizeof(held_controls));memset(steady_wind,0,sizeof(steady_wind));next_step=0;
  active=1;last_error[0]=0;return 0;
}
int32_t hil_model_initialize(const HilInitialConditions *i) {if(active) return fail(2,"Already initialized; use reset after host stops stepping");return initialize(i);}
int32_t hil_model_reset(const HilInitialConditions *i) {return initialize(i);}
int32_t hil_model_configure_wind(const HilWindConfig *w) {
  if(!active) return fail(2,"Model is not initialized");
  if(next_step!=0 || carrier_enabled) return fail(2,"Wind must be set before stepping and cannot be mixed with carrier mode");
  if(!w||!valid_header(w->abi_version,w->struct_size,sizeof(*w))) return fail(4,"wind ABI mismatch");
  for(int k=0;k<3;k++) if(!isfinite(w->steady_ned_m_s[k])||fabs(w->steady_ned_m_s[k])>100) return fail(4,"Invalid steady wind");
  PlantWindConfig c;plant_default_wind_config(&c);
  c.start_mode=w->start_mode;c.start_height_m=w->start_height_m;c.start_time_s=w->start_time_s;c.ramp_s=w->ramp_s;
  c.gust_start_s=w->gust_start_s;
  for(int k=0;k<3;k++) {c.gust_length_m[k]=w->gust_length_m[k];c.gust_magnitude_m_s[k]=w->gust_magnitude_m_s[k];}
  c.shear_speed_6m_m_s=w->shear_speed_6m_m_s;c.shear_from_deg=w->shear_from_deg;
  c.turbulence_speed_6m_m_s=w->turbulence_speed_6m_m_s;c.turbulence_from_deg=w->turbulence_from_deg;
  for(int k=0;k<4;k++) c.turbulence_seed[k]=w->turbulence_seed[k];
  if(plant_configure_wind(plant,&c)!=PLANT_OK) return fail(4,"Plant rejected the wind");
  wind_configured=1;
  memcpy(steady_wind,w->steady_ned_m_s,sizeof(steady_wind));
  return 0;
}
int32_t hil_model_configure_carrier(const HilCarrierConfig *c) {
  if(!active || next_step!=0 || wind_configured || !carrier_ground_initial || carrier_enabled)
    return fail(2,"Carrier requires fresh ground/zero-velocity/zero-Euler reset without wind");
  if(!c || !valid_header(c->abi_version,c->struct_size,sizeof(*c))) return fail(4,"Carrier ABI mismatch");
  PlantCarrierConfig config={c->wait_s,c->climb_s,c->cruise_s,c->height_m,c->speed_m_s};
  PlantCarrierSample sample;
  if(plant_carrier_sample(&config,0,&sample)!=PLANT_OK) return fail(4,"Invalid carrier configuration");
  /* An exactly prescribed cruise otherwise produces identical float differential
   * pressure for seconds. PX4 correctly rejects that as a stuck sensor. Model
   * a small measurement noise here, without changing ordinary-flight defaults,
   * plant truth, or the FC's validity checks. Only configurable before step 0. */
  PlantSensorConfig sensor_config;plant_sensors_default_config(&sensor_config);
  sensor_config.diff_pressure_noise_pa=0.1;
  if(plant_sensors_init(sensors,&sensor_config,HIL_DEFAULT_SEED)!=PLANT_OK)
    return fail(4,"Sensors rejected the carrier configuration");
  carrier=config;carrier_enabled=1;carrier_started=0;carrier_arm_step=0;return 0;
}
int32_t hil_model_terminate(void) {active=0;free(plant);free(sensors);plant=NULL;sensors=NULL;return 0;}
/* raw_source[58]: positions and units of the recorded reference (plant/reference); Truth fields from the
 * C Truth with the P8 corrections (true airspeed, horizontal ground speed, G = -f_z / g), sensor
 * fields in the source units (gauss, hPa, deg*1e7, mm, cm/s, cdeg). */
static void fill_raw(const PlantTruth *t,const PlantSensorOutput *s,double *r) {
  const double deg=180.0/kPi;
  r[0]=t->surface_deflection_rad[1]*deg;r[1]=t->surface_deflection_rad[0]*deg;r[2]=t->surface_deflection_rad[2]*deg;
  r[3]=t->throttle_actual;r[4]=t->surface_deflection_rad[3]*deg;
  r[5]=t->angular_accel_body_rad_s2[0];
  for(int k=0;k<4;k++) r[6+k]=t->quaternion_nb[k];
  /* yaw in [0, 2*pi): a tiny negative angle plus 2*pi rounds to exactly 2*pi, which wraps to 0 */
  r[10]=fmod(t->euler_rad[2],2*kPi);if(r[10]<0) r[10]+=2*kPi;if(r[10]>=2*kPi) r[10]=0.0;
  r[11]=t->euler_rad[1];r[12]=t->euler_rad[0];
  r[13]=t->angular_accel_body_rad_s2[1];r[14]=t->angular_accel_body_rad_s2[2];
  for(int k=0;k<3;k++) {r[15+k]=t->velocity_ned_m_s[k];r[22+k]=t->position_ned_m[k];r[28+k]=t->rates_body_rad_s[k];r[31+k]=t->specific_force_body_m_s2[k];}
  r[18]=-t->velocity_ned_m_s[2];r[19]=t->height_agl_m;r[20]=t->latitude_deg;r[21]=t->longitude_deg;
  r[25]=t->alpha_rad*deg;r[26]=t->beta_rad*deg;r[27]=t->airspeed_true_m_s;
  r[34]=-t->specific_force_body_m_s2[2]/kG0;r[35]=t->ground_speed_m_s;r[36]=t->mach;r[37]=t->alpha_rad;r[38]=t->beta_rad;
  for(int k=0;k<3;k++) {r[39+k]=s->accel_m_s2[k];r[42+k]=s->gyro_rad_s[k];r[45+k]=s->magnetic_ut[k]/100.0;r[53+k]=s->gps_velocity_ned_m_s[k]*100.0;}
  r[48]=s->absolute_pressure_pa/100.0;r[49]=s->differential_pressure_pa/100.0;
  r[50]=s->gps_lla_msl[0]*1e7;r[51]=s->gps_lla_msl[1]*1e7;r[52]=s->gps_lla_msl[2]*1000.0;
  r[56]=s->gps_ground_speed_m_s*100.0;r[57]=s->gps_course_deg*100.0;
}
int32_t hil_model_step(const HilModelInput *i,HilModelOutput *o) {
  if(!active) return fail(2,"Model is not initialized");
  if(!i||!o||!valid_header(i->abi_version,i->struct_size,sizeof(*i))||!valid_header(o->abi_version,o->struct_size,sizeof(*o))) return fail(4,"step ABI mismatch");
  if(i->step_index!=next_step) return fail(2,"Unexpected step index");
  if(i->actuator_valid>1||i->actuator_new>1||i->fc_armed>2) return fail(4,"Invalid input flags");
  if(i->actuator_valid && i->actuator_new) {
    for(int k=0;k<16;k++) if(!isfinite(i->controls[k])) return fail(4,"Nonfinite controls");
    memcpy(held_controls,i->controls,sizeof(held_controls));
  }
  PlantInput in;memset(&in,0,sizeof(in));
  in.command.aileron_left=held_controls[0];in.command.aileron_right=held_controls[1];in.command.elevator=held_controls[2];
  in.command.rudder=held_controls[3];in.command.throttle=held_controls[4];
  memcpy(in.wind_ned_m_s,steady_wind,sizeof(in.wind_ned_m_s));
  PlantTruth t;PlantSensorOutput s;
  uint32_t carrier_phase=0;
  int32_t code;
  if(carrier_enabled) {
    if(carrier_started && i->fc_armed!=1) {active=0;return fail(2,"Carrier run lost confirmed armed state; stop and recover");}
    if(!carrier_started && i->fc_armed==1) {carrier_started=1;carrier_arm_step=next_step;}
    const double elapsed=carrier_started ? (double)(next_step-carrier_arm_step)*0.004 : 0.0;
    carrier_phase=!carrier_started ? 1u : elapsed<carrier.wait_s+carrier.climb_s+carrier.cruise_s ? 2u : 3u;
    code=plant_carrier_step_at(plant,&in,&carrier,elapsed,(uint32_t)carrier_started,&t);
  } else code=plant_step(plant,&in,&t);
  if(code==PLANT_ERR_DIVERGED) {active=0;return fail(3,"Plant returned nonfinite data; reset required");}
  if(code!=PLANT_OK) {active=0;return fail(2,"Plant step failed; reset required");}
  if(plant_sensors_step(sensors,&t,&s)!=PLANT_OK) {active=0;return fail(2,"Sensor step failed; reset required");}
  memset(o,0,sizeof(*o));o->abi_version=HIL_ABI_VERSION;o->struct_size=sizeof(*o);o->model_time_ns=next_step*4000000;
  o->reserved=carrier_phase;
  fill_raw(&t,&s,o->raw_source);
  for(int k=0;k<HIL_RAW_COUNT;k++) if(!isfinite(o->raw_source[k])) {active=0;return fail(3,"Plant returned nonfinite data; reset required");}
  /* Recipe envelope: fault on divergence before float wire conversion (same limits as before). */
  for(int k=0;k<3;k++) if(fabs(s.accel_m_s2[k])>1000) {active=0;return fail(3,"Acceleration outside validated runtime envelope; reset required");}
  for(int k=0;k<3;k++) if(fabs(s.gyro_rad_s[k])>100) {active=0;return fail(3,"Angular rate outside runtime envelope; reset required");}
  for(int k=0;k<3;k++) if(fabs(s.gps_velocity_ned_m_s[k])>200) {active=0;return fail(3,"GPS velocity outside runtime envelope; reset required");}
  for(int k=0;k<3;k++) {o->accel_m_s2[k]=s.accel_m_s2[k];o->gyro_rad_s[k]=s.gyro_rad_s[k];o->magnetic_uT[k]=s.magnetic_ut[k];}
  o->absolute_pressure_pa=s.absolute_pressure_pa;o->differential_pressure_pa=s.differential_pressure_pa;
  o->imu_valid=o->imu_updated=1;o->imu_sample_time_ns=o->model_time_ns;o->fields_updated=0x7ff;
  o->gps_lla_msl[0]=s.gps_lla_msl[0];o->gps_lla_msl[1]=s.gps_lla_msl[1];o->gps_lla_msl[2]=s.gps_lla_msl[2]-geoid_separation;
  memcpy(o->velocity_ned_m_s,s.gps_velocity_ned_m_s,sizeof(o->velocity_ned_m_s));
  o->ground_speed_m_s=s.gps_ground_speed_m_s;o->course_deg=s.gps_course_deg;
  o->gps_sample_time_ns=(uint64_t)llround(s.gps_sample_time_s*1e9);o->gps_updated=s.gps_updated;o->gps_valid=1;
  next_step++;return 0;
}
int32_t hil_model_get_last_error(char *b,uint32_t capacity) {if(!b||!capacity) return 4;snprintf(b,capacity,"%s",last_error);return 0;}
