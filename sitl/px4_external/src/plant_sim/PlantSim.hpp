/**
 * plant_sim: the aircraft plant (generated core + C Truth + C sensors) inside PX4.
 *
 * Lockstep like PX4's simulator_sih: this module owns the PX4 clock and advances it by the plant step
 * (4 ms). Each step it reads the PX4 actuator outputs (actuator_outputs_sim, channels 1..5 =
 * controls[0..4] of the HIL ABI), steps the plant, publishes the IMU through PX4Accelerometer /
 * PX4Gyroscope (so PX4's own sensors module and gyro filtering run), and publishes the estimator
 * outputs (attitude, position, airspeed, air data, land detected) directly from Truth instead of EKF2.
 * Above the control layer it is a stub: it publishes the vehicle status, control mode, arming and the
 * stick inputs of the scenario. See sitl/README.md.
 */
#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <lib/dataman_client/DatamanClient.hpp>

#include <stdio.h>

#include "skysurfer_plant.h"
#include "plant_sensors.h"
#include "plant_carrier.h"

class PlantSim : public ModuleBase<PlantSim>
{
public:
	PlantSim(float duration_s, const char *output_dir, const char *scenario_path);
	~PlantSim() override;

	static int task_spawn(int argc, char *argv[]);
	static PlantSim *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	struct Sticks {
		float roll, pitch, yaw, throttle;
	};
	struct Step {
		float start_s, end_s; // relative to the first plant step
		Sticks delta;         // added to the base sticks while active
	};
	static constexpr int kMaxSteps = 64;

	static constexpr uint64_t kStepUs = 4000; // plant fixed step (PLANT_CORE_DT_S)
	static constexpr float kAirDensitySeaLevel = 1.225f;
	// Start-up: (1) wall-clock steps until pwm_out_sim publishes (all PX4 modules and work queues exist;
	// waiting for lockstep earlier can deadlock while work queues are still being created); (2) this many
	// lockstep steps disarmed with the plant held, so every run reaches the same PX4 state (output modules
	// also assign PWM_MAIN_FUNC only while disarmed); (3) armed, the plant starts with the first output stamped at or after the armed stub.
	static constexpr uint64_t kDisarmedWarmupSteps = 100;

	bool load_scenario();
	Sticks scenario_sticks(float t) const;
	void publish_stub(hrt_abstime now, const Sticks &sticks, bool armed);
	void publish_truth(hrt_abstime now, const PlantTruth &truth, const PlantSensorOutput &sensors);
	void read_controls(bool armed);
	void advance_clock();

	float _duration_s;
	char _output_dir[256] {};
	char _scenario_path[256] {};
	// Scenario (written by sitl/run_sitl.py from a scenario JSON): mode, initial state, base sticks, steps.
	bool _have_carrier{false};
	PlantCarrierConfig _carrier{};
	bool _acro{false};
	bool _offboard{false};  // OFFBOARD with offboard_control_mode.velocity (a module of modules/ does the guidance)
	bool _native_mission{false};
	bool _auto_active{false};
	float _auto_after_s{100.f};
	mission_item_s _mission_items[32] {};
	int _mission_count{0};
	mission_s _mission{};
	bool _mission_sent{false};
	float _truth_height{0.f};
	float _truth_speed{0.f};
	PlantInitialConditions _ic{};
	PlantWindConfig _wind{};       // scenario line "wind" (plant/wind.py sitl_line)
	double _steady_wind_ned[3] {};
	bool _have_wind{false};
	Sticks _base{0.f, 0.f, 0.f, -1.f};
	Step _step_list[kMaxSteps] {};
	int _num_steps{0};

	Plant *_plant{nullptr};
	PlantSensors *_sensors{nullptr};
	double _controls[5] {};
	uint64_t _sim_time_us{0};
	uint64_t _steps{0};
	bool _modules_up{false};   // pwm_out_sim has published (phase 2)
	bool _outputs_seen{false}; // first armed output received (phase 3)
	hrt_abstime _armed_time{0}; // first publication of the armed stub
	double _origin_lat_deg{0.0};
	double _origin_lon_deg{0.0};
	FILE *_trace{nullptr};

	PX4Accelerometer _px4_accel{1310988}; // DRV_IMU_DEVTYPE_SIM, bus 1, address 1 (CAL_ACC0_ID)
	PX4Gyroscope _px4_gyro{1310988};      // (CAL_GYRO0_ID)

	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs_sim)};
	uORB::Publication<actuator_armed_s> _armed_pub{ORB_ID(actuator_armed)};
	uORB::Publication<airspeed_validated_s> _airspeed_pub{ORB_ID(airspeed_validated)};
	uORB::Publication<manual_control_setpoint_s> _manual_pub{ORB_ID(manual_control_setpoint)};
	uORB::Publication<vehicle_air_data_s> _air_data_pub{ORB_ID(vehicle_air_data)};
	uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_control_mode_s> _control_mode_pub{ORB_ID(vehicle_control_mode)};
	uORB::Publication<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_status_s> _status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<home_position_s> _home_pub{ORB_ID(home_position)};
	uORB::Publication<mission_s> _mission_pub{ORB_ID(mission)};
	uORB::Subscription _mission_result_sub{ORB_ID(mission_result)};
};
