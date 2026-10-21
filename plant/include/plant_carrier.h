#ifndef PLANT_CARRIER_H
#define PLANT_CARRIER_H
#include "skysurfer_plant.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Phase-one carrier: no wind; heading north, zero Euler angles and body rates.
 * Ground wait -> smooth prescribed ascent/acceleration -> level cruise -> release.
 * This is a kinematic transport rig, not an aerodynamic mother-aircraft model.
 * wait_s starts at the first model step; callers must separately gate transport
 * on real arming in HITL. The plant interface does NOT authorize arming/release. */
typedef struct {
  double wait_s, climb_s, cruise_s, height_m, speed_m_s;
} PlantCarrierConfig;
typedef struct {
  double rigid[13];
  double accel_ned_m_s2[3];
  uint32_t carried;
} PlantCarrierSample;
int32_t plant_carrier_sample(const PlantCarrierConfig *config, double time_s, PlantCarrierSample *sample);
/* Call from the first step of an origin/zero-velocity/zero-attitude plant.
 * t_k truth is published before integration. During carriage, external reaction
 * supports the prescribed motion; the free-flight force components in PlantTruth
 * exclude that reaction. At release, the same core continues without resetting
 * its clock, actuator/filter states, or the downstream sensors. */
int32_t plant_carrier_step(Plant *plant, const PlantInput *input,
                           const PlantCarrierConfig *config, PlantTruth *truth);
/* Same physics with an explicit carrier clock. While waiting for arming, pass
 * time_s=0 and advance=0; model and sensor time still advance normally. Once
 * started, use monotonically increasing carrier time and advance=1. */
int32_t plant_carrier_step_at(Plant *plant, const PlantInput *input,
    const PlantCarrierConfig *config, double time_s, uint32_t advance, PlantTruth *truth);
#ifdef __cplusplus
}
#endif
#endif
