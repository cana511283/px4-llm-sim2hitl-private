#include "PlantSim.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/time.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

static uint64_t wall_us()
{
	struct timeval t;
	gettimeofday(&t, nullptr);
	return (uint64_t)t.tv_sec * 1000000ULL + (uint64_t)t.tv_usec;
}

PlantSim::PlantSim(float duration_s, const char *output_dir, const char *scenario_path) : _duration_s(duration_s)
{
	snprintf(_output_dir, sizeof(_output_dir), "%s", output_dir);
	snprintf(_scenario_path, sizeof(_scenario_path), "%s", scenario_path);
}

PlantSim::~PlantSim()
{
	free(_plant);
	free(_sensors);

	if (_trace) {
		fclose(_trace);
	}
}

// Scenario file, one record per line (sitl/run_sitl.py writes it):
//   mode stabilized|acro|offboard
//   ic <height_agl_m> <u> <v> <w m/s body> <roll> <pitch> <yaw deg> <lat> <lon deg>
//   base <roll> <pitch> <yaw> <throttle>          sticks held for the whole run
//   step <start_s> <end_s> <roll> <pitch> <yaw> <throttle>   added to base while start <= t < end
bool PlantSim::load_scenario()
{
	FILE *f = fopen(_scenario_path, "r");

	if (!f) {
		PX4_ERR("cannot open scenario %s", _scenario_path);
		return false;
	}

	char line[512]; // the wind line has 22 numbers
	bool have_ic = false;

	while (fgets(line, sizeof(line), f)) {
		char word[16] {};
		double v[9] {};
		int terrain_valid = -1;

		if (sscanf(line, "%15s", word) != 1 || word[0] == '#') {
			continue;
		}

		if (!strcmp(word, "mode")) {
			char mode[32] {};
			sscanf(line, "%*s %31s", mode);
			_acro = !strcmp(mode, "acro");
			_offboard = !strcmp(mode, "offboard");
			_native_mission = !strcmp(mode, "mission");

			if (!_acro && !_offboard && !_native_mission && strcmp(mode, "stabilized")) {
				PX4_ERR("unsupported mode %s", mode);
				fclose(f);
				return false;
			}

		} else if (!strcmp(word, "ic") && sscanf(line, "%*s %lf %lf %lf %lf %lf %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3], &v[4],
				&v[5], &v[6], &v[7], &v[8]) == 9) {
			_ic.height_agl_m = v[0];

			for (int k = 0; k < 3; k++) {
				_ic.velocity_body_m_s[k] = v[1 + k];
				_ic.euler_rad[k] = v[4 + k] * (M_PI / 180.0);
			}

			_ic.latitude_deg = v[7];
			_ic.longitude_deg = v[8];
			have_ic = true;

		} else if (!strcmp(word, "wind")) {
			// steady_ned[3] start_mode start_height start_time ramp gust_start gust_length[3] gust_magnitude[3]
			// shear_speed shear_from turbulence_speed turbulence_from seed[4] (plant/wind.py sitl_line)
			double w[22] {};
			int used = 0;
			const char *cursor = line + 4;
			int count = 0;

			while (count < 22 && sscanf(cursor, "%lf%n", &w[count], &used) == 1) {
				cursor += used;
				count++;
			}

			if (count != 22) {
				PX4_ERR("bad wind line: %s", line);
				fclose(f);
				return false;
			}

			plant_default_wind_config(&_wind);

			for (int k = 0; k < 3; k++) {
				_steady_wind_ned[k] = w[k];
				_wind.gust_length_m[k] = w[8 + k];
				_wind.gust_magnitude_m_s[k] = w[11 + k];
			}

			_wind.start_mode = (uint32_t)w[3];
			_wind.start_height_m = w[4];
			_wind.start_time_s = w[5];
			_wind.ramp_s = w[6];
			_wind.gust_start_s = w[7];
			_wind.shear_speed_6m_m_s = w[14];
			_wind.shear_from_deg = w[15];
			_wind.turbulence_speed_6m_m_s = w[16];
			_wind.turbulence_from_deg = w[17];

			for (int k = 0; k < 4; k++) {
				_wind.turbulence_seed[k] = (uint32_t)w[18 + k];
			}

			_have_wind = true;

		} else if (!strcmp(word, "carrier") && sscanf(line, "%*s %lf %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3], &v[4]) == 5) {
            _carrier = PlantCarrierConfig{v[0], v[1], v[2], v[3], v[4]};
            PlantCarrierSample sample{};
            if (plant_carrier_sample(&_carrier, 0., &sample) != PLANT_OK) {
                fclose(f);
                return false;
            }
            _have_carrier = true;
		} else if (!strcmp(word, "terrain_valid") && sscanf(line, "%*s %d", &terrain_valid) == 1
                   && (terrain_valid == 0 || terrain_valid == 1)) {
            _terrain_measurement = terrain_valid == 1;
		} else if (!strcmp(word, "auto_after") && sscanf(line, "%*s %lf", &v[0]) == 1) {
            _auto_after_s = v[0];
        } else if (!strcmp(word, "mission") && _mission_count < 32
                   && sscanf(line, "%*s %lf %lf %lf %lf %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]) == 8) {
            mission_item_s &item = _mission_items[_mission_count++];
            item.nav_cmd = (uint16_t)v[0];
            item.autocontinue = true;
            item.origin = ORIGIN_MAVLINK;
            for (int k=0; k<7; k++) { item.params[k] = (float)v[k+1]; }
            if (item.nav_cmd == NAV_CMD_WAYPOINT || item.nav_cmd == NAV_CMD_LOITER_TO_ALT || item.nav_cmd == NAV_CMD_LAND) {
                item.lat = v[5]; item.lon = v[6]; item.altitude = v[7];
                item.frame = 6; item.altitude_is_relative = true;
                item.yaw = PX4_ISFINITE(v[4]) ? (float)(v[4]*M_PI/180.) : NAN;
                if (item.nav_cmd == NAV_CMD_LOITER_TO_ALT) {
                    // Match mavlink_mission.cpp: LOITER_TO_ALT uses param2
                    // for radius, unlike LOITER_TIME/UNLIM which use param3.
                    item.time_inside = 0.f; item.acceptance_radius = 0.f;
                    item.loiter_radius = (float)v[2];
                    item.force_heading = v[1]>0; item.loiter_exit_xtrack = v[4]>0;
                    item.yaw = NAN;
                }
            } else { item.frame = 2; }
		} else if (!strcmp(word, "base") && sscanf(line, "%*s %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3]) == 4) {
			_base = Sticks{(float)v[0], (float)v[1], (float)v[2], (float)v[3]};

		} else if (!strcmp(word, "step") && _num_steps < kMaxSteps
			   && sscanf(line, "%*s %lf %lf %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == 6) {
			_step_list[_num_steps++] = Step{(float)v[0], (float)v[1], Sticks{(float)v[2], (float)v[3], (float)v[4], (float)v[5]}};

		} else {
			PX4_ERR("bad scenario line: %s", line);
			fclose(f);
			return false;
		}
	}

	fclose(f);
	if (_native_mission && (!_have_carrier || _mission_count == 0 || !PX4_ISFINITE(_auto_after_s)
			       || (double)_auto_after_s < _carrier.wait_s + _carrier.climb_s + _carrier.cruise_s)) {
		PX4_ERR("native mission needs a carrier and a valid post-release AUTO time");
		return false;
	}
	return have_ic;
}

PlantSim::Sticks PlantSim::scenario_sticks(float t) const
{
	Sticks s = _base;

	for (int i = 0; i < _num_steps; i++) {
		if (t >= _step_list[i].start_s && t < _step_list[i].end_s) {
			s.roll += _step_list[i].delta.roll;
			s.pitch += _step_list[i].delta.pitch;
			s.yaw += _step_list[i].delta.yaw;
			s.throttle += _step_list[i].delta.throttle;
		}
	}

	return s;
}

void PlantSim::advance_clock()
{
	_sim_time_us += kStepUs;
	struct timespec ts;
	abstime_to_ts(&ts, _sim_time_us);
	px4_clock_settime(CLOCK_MONOTONIC, &ts);
}

void PlantSim::read_controls(bool armed)
{
	actuator_outputs_s out;

	if (!_actuator_outputs_sub.update(&out)) {
		return;
	}

	_modules_up = true;

	// The first armed output must have been computed after the armed stub was published (publish_stub runs
	// later in the loop than this read); an earlier sample is still a disarmed one.
	if (!_outputs_seen && !(armed && _armed_time != 0 && out.timestamp >= _armed_time)) {
		return;
	}

	if (armed) {
		_outputs_seen = true;

		for (int i = 0; i < 5; i++) {
			// Disarmed channels are NaN; hold zero like the HIL adapter holds its initial controls.
			_controls[i] = PX4_ISFINITE(out.output[i]) ? (double)out.output[i] : 0.0;
		}
	}
}

void PlantSim::publish_stub(hrt_abstime now, const Sticks &sticks, bool armed)
{
	if (armed && _armed_time == 0) {
		_armed_time = now;
	}

	vehicle_status_s status{};
	status.timestamp = now;
	const uint8_t nav_state = _auto_active ? vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION : _acro ? vehicle_status_s::NAVIGATION_STATE_ACRO
				  : _offboard ? vehicle_status_s::NAVIGATION_STATE_OFFBOARD : vehicle_status_s::NAVIGATION_STATE_STAB;
	status.nav_state = nav_state;
	status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;
	status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;
	status.system_id=1; status.component_id=1;
	status.nav_state_user_intention=nav_state;
	_status_pub.publish(status);

	// Same flags as commander's ModeUtil (control_mode.cpp) for NAVIGATION_STATE_STAB / NAVIGATION_STATE_ACRO, and for
	// NAVIGATION_STATE_OFFBOARD with offboard_control_mode.velocity (no Commander here: the module's signal is not read).
	vehicle_control_mode_s mode{};
	mode.timestamp = now;
	mode.flag_armed = armed;
	mode.flag_control_manual_enabled = !_offboard && !_auto_active;
	mode.flag_control_auto_enabled = _auto_active;
	mode.flag_control_position_enabled = _auto_active;
	mode.flag_control_offboard_enabled = _offboard;
	mode.flag_control_velocity_enabled = _offboard || _auto_active;
	mode.flag_control_altitude_enabled = _offboard || _auto_active;
	mode.flag_control_climb_rate_enabled = _offboard || _auto_active;
	mode.flag_control_acceleration_enabled = _offboard;
	mode.flag_control_attitude_enabled = !_acro;
	mode.flag_control_rates_enabled = true;
	mode.flag_control_allocation_enabled = true;
	mode.source_id = nav_state;
	_control_mode_pub.publish(mode);

	actuator_armed_s actuator_armed{};
	actuator_armed.timestamp = now;
	actuator_armed.armed = armed;
	actuator_armed.prearmed = armed;
	actuator_armed.ready_to_arm = true;
	_armed_pub.publish(actuator_armed);

	manual_control_setpoint_s manual{};
	manual.timestamp = now;
	manual.timestamp_sample = now;
	manual.valid = true;
	manual.data_source = manual_control_setpoint_s::SOURCE_RC;
	manual.roll = sticks.roll;
	manual.pitch = sticks.pitch;
	manual.yaw = sticks.yaw;
	manual.throttle = sticks.throttle;
	_manual_pub.publish(manual);

	vehicle_land_detected_s land{};
	land.timestamp = now;
	land.landed = _native_mission && _auto_active && _truth_height < .05f && _truth_speed < 1.f;
	_land_detected_pub.publish(land);
}

void PlantSim::publish_truth(hrt_abstime now, const PlantTruth &t, const PlantSensorOutput &s)
{
    _truth_height = t.height_agl_m; _truth_speed = t.ground_speed_m_s;
    if (_native_mission && _steps % 50 == 0) {
        home_position_s home{}; home.timestamp=now;home.lat=_origin_lat_deg;home.lon=_origin_lon_deg;
        home.alt=0.f;home.valid_alt=home.valid_hpos=home.valid_lpos=true;home.update_count=1;
        _home_pub.publish(home);
        if (!_mission_sent) {
            _mission.timestamp=now; _mission_pub.publish(_mission); _mission_sent=true;
        }
    }
	_px4_accel.update(now, (float)s.accel_m_s2[0], (float)s.accel_m_s2[1], (float)s.accel_m_s2[2]);
	_px4_gyro.update(now, (float)s.gyro_rad_s[0], (float)s.gyro_rad_s[1], (float)s.gyro_rad_s[2]);

	vehicle_attitude_s attitude{};
	attitude.timestamp_sample = now;

	for (int i = 0; i < 4; i++) {
		attitude.q[i] = (float)t.quaternion_nb[i];
	}

	attitude.timestamp = now;
	_attitude_pub.publish(attitude);

	vehicle_local_position_s lpos{};
	lpos.timestamp_sample = now;
	lpos.xy_valid = lpos.z_valid = lpos.v_xy_valid = lpos.v_z_valid = true;
	lpos.x = (float)t.position_ned_m[0];
	lpos.y = (float)t.position_ned_m[1];
	lpos.z = (float)t.position_ned_m[2];
	lpos.vx = (float)t.velocity_ned_m_s[0];
	lpos.vy = (float)t.velocity_ned_m_s[1];
	lpos.vz = (float)t.velocity_ned_m_s[2];
	lpos.z_deriv = lpos.vz;
	lpos.ax = (float)t.accel_ned_m_s2[0];
	lpos.ay = (float)t.accel_ned_m_s2[1];
	lpos.az = (float)t.accel_ned_m_s2[2];
	lpos.xy_global = lpos.z_global = true;
	lpos.ref_timestamp = 0;
	lpos.ref_lat = _origin_lat_deg;
	lpos.ref_lon = _origin_lon_deg;
	lpos.ref_alt = 0.f;
	lpos.heading = (float)t.euler_rad[2];
	lpos.heading_good_for_control = true;
	lpos.unaided_heading = NAN;
	lpos.dist_bottom = (float)t.height_agl_m;
	lpos.dist_bottom_valid = _terrain_measurement;
	lpos.eph = lpos.epv = 0.01f;
	lpos.evh = lpos.evv = 0.01f;
	lpos.vxy_max = lpos.vz_max = lpos.hagl_min = lpos.hagl_max_z = lpos.hagl_max_xy = INFINITY;
	lpos.timestamp = now;
	_local_position_pub.publish(lpos);

	vehicle_global_position_s gpos{};
	gpos.timestamp_sample = now;
	gpos.lat = t.latitude_deg;
	gpos.lon = t.longitude_deg;
	gpos.alt = (float)t.altitude_msl_m;
	gpos.alt_ellipsoid = gpos.alt;
	gpos.terrain_alt = (float)(t.altitude_msl_m - t.height_agl_m);
	gpos.terrain_alt_valid = _terrain_measurement;
	gpos.eph = gpos.epv = 0.01f;
	gpos.timestamp = now;
	_global_position_pub.publish(gpos);

	const float tas = (float)t.airspeed_true_m_s;
	const float cas = tas * sqrtf((float)t.air_density_kg_m3 / kAirDensitySeaLevel);
	airspeed_validated_s airspeed{};
	airspeed.indicated_airspeed_m_s = cas;
	airspeed.calibrated_airspeed_m_s = cas;
	airspeed.true_airspeed_m_s = tas;
	airspeed.airspeed_source = airspeed_validated_s::SOURCE_SENSOR_1;
	airspeed.calibrated_ground_minus_wind_m_s = NAN;
	airspeed.calibraded_airspeed_synth_m_s = NAN;
	airspeed.timestamp = now;
	_airspeed_pub.publish(airspeed);

	vehicle_air_data_s air{};
	air.timestamp_sample = now;
	air.baro_alt_meter = (float)t.altitude_msl_m;
	air.baro_pressure_pa = (float)t.static_pressure_pa;
	air.ambient_temperature = (float)(t.air_temperature_k - 273.15);
	air.rho = (float)t.air_density_kg_m3;
	air.timestamp = now;
	_air_data_pub.publish(air);
}

void PlantSim::run()
{
	_plant = (Plant *)malloc(plant_size());
	_sensors = (PlantSensors *)malloc(plant_sensors_size());

	// Air start from the scenario (decision 8), fixed sensor seed as in the HIL adapter.
	if (!load_scenario()) {
		PX4_ERR("scenario rejected");
		return;
	}

	PlantInitialConditions ic = _ic;
	ic.seed = 1;
	_origin_lat_deg = ic.latitude_deg;
	_origin_lon_deg = ic.longitude_deg;
	PlantSensorConfig config;
	plant_sensors_default_config(&config);

	if (!_plant || !_sensors || plant_init(_plant, &ic) != PLANT_OK || plant_sensors_init(_sensors, &config, ic.seed) != PLANT_OK) {
		PX4_ERR("plant initialisation failed");
		return;
	}

	if (_have_wind && plant_configure_wind(_plant, &_wind) != PLANT_OK) {
		PX4_ERR("plant rejected the wind");
		return;
	}

    if (_native_mission) {
        DatamanClient client;
        _mission.mission_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
        _mission.count=_mission_count; _mission.current_seq=0;
        _mission.land_start_index=-1;_mission.land_index=-1;_mission.mission_id=20261021;
        for (int k=0;k<_mission_count;k++) {
            auto &item=_mission_items[k];
            if (item.nav_cmd==NAV_CMD_DO_LAND_START) { _mission.land_start_index=k; }
            if (item.nav_cmd==NAV_CMD_LAND) { _mission.land_index=k; }
            if (!client.writeSync(DM_KEY_WAYPOINTS_OFFBOARD_0,k,reinterpret_cast<uint8_t *>(&item),sizeof(item))) {
                PX4_ERR("mission dataman write failed"); return;
            }
        }
        if (!client.writeSync(DM_KEY_MISSION_STATE,0,reinterpret_cast<uint8_t *>(&_mission),sizeof(_mission))) {
            PX4_ERR("mission state write failed");return;
        }
    }
	char path[320];
	snprintf(path, sizeof(path), "%s/trace.csv", _output_dir);
	_trace = fopen(path, "w");

	if (_trace) {
		fprintf(_trace, "t_s,roll_rad,pitch_rad,yaw_rad,p_rad_s,q_rad_s,r_rad_s,tas_m_s,height_m,"
			"stick_roll,stick_pitch,stick_yaw,stick_throttle,ail_l,ail_r,elev,rud,thr,gs_m_s,wind_n_m_s,wind_e_m_s,wind_d_m_s,carrier_attached\n");
	}

	_px4_accel.set_temperature(25.f);
	_px4_gyro.set_temperature(25.f);
	_sim_time_us = 0; // this module owns the PX4 clock (lockstep), as simulator_sih does

	PlantInput input{};

	for (int k = 0; k < 3; k++) {
		input.wind_ned_m_s[k] = _steady_wind_ned[k];
	}

	PlantTruth truth{};
	PlantSensorOutput sensors{};
	// Rounded: a float division truncated 60 s to 14,999 steps.
	const uint64_t total_steps = (uint64_t)llround((double)_duration_s * 1e6 / (double)kStepUs);
	uint64_t wall_start = 0;
	uint64_t warmup = 0;          // all start-up steps (wall clock and lockstep)
	uint64_t lockstep_warmup = 0; // start-up steps after the modules are up
	bool diverged = false;

	// The plant is held at its initial state (truth and IMU of t_0 republished) during the disarmed start-up
	// and until PX4 produces the first armed actuator output; then every step advances the plant.
	if (_have_carrier) { plant_carrier_step(_plant, &input, &_carrier, &truth); }
	else { plant_step(_plant, &input, &truth); }
	plant_sensors_step(_sensors, &truth, &sensors);

	while (!should_exit() && _steps < total_steps) {
		advance_clock();
		const hrt_abstime now = hrt_absolute_time();
		const bool armed = lockstep_warmup >= kDisarmedWarmupSteps;
		read_controls(armed);

		if (_outputs_seen) {
			if (wall_start == 0) {
				wall_start = wall_us();
			}

			input.command.aileron_left = _controls[0];
			input.command.aileron_right = _controls[1];
			input.command.elevator = _controls[2];
			input.command.rudder = _controls[3];
			input.command.throttle = _controls[4];

			if ((_have_carrier ? plant_carrier_step(_plant, &input, &_carrier, &truth) : plant_step(_plant, &input, &truth)) != PLANT_OK || plant_sensors_step(_sensors, &truth, &sensors) != PLANT_OK) {
				PX4_ERR("plant diverged at step %llu", (unsigned long long)_steps);
				diverged = true;
				break;
			}

			_steps++;

		} else {
			warmup++;

			if (_modules_up) {
				lockstep_warmup++;
			}
		}

		_auto_active = _native_mission && _steps * kStepUs * 1e-6f >= _auto_after_s;
        const Sticks sticks = scenario_sticks(_steps * kStepUs * 1e-6f);
		publish_stub(now, sticks, armed);
		publish_truth(now, truth, sensors);

		if (_trace && _outputs_seen) {
			fprintf(_trace, "%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f,%u\n",
				truth.time_s, truth.euler_rad[0], truth.euler_rad[1], truth.euler_rad[2], truth.rates_body_rad_s[0],
				truth.rates_body_rad_s[1], truth.rates_body_rad_s[2], truth.airspeed_true_m_s, truth.height_agl_m,
				(double)sticks.roll, (double)sticks.pitch, (double)sticks.yaw, (double)sticks.throttle,
				_controls[0], _controls[1], _controls[2], _controls[3], _controls[4], truth.ground_speed_m_s,
				truth.wind_ned_m_s[0], truth.wind_ned_m_s[1], truth.wind_ned_m_s[2],
                (unsigned)(_have_carrier && truth.time_s < _carrier.wait_s + _carrier.climb_s + _carrier.cruise_s));
		}

		if (_modules_up) {
			px4_lockstep_wait_for_components();

		} else {
			usleep(kStepUs); // phase 1: wall clock (px4_usleep would wait for the simulated clock)
		}
	}

	const uint64_t wall_end = wall_us();
	const double wall_s = wall_start ? (wall_end - wall_start) * 1e-6 : 0.0;
	const double sim_s = _steps * kStepUs * 1e-6;
	// How the run ended (summary "end"): sitl/run_sitl.py counts only "complete" as a run of the whole scenario.
	const char *end = diverged ? "diverged" : _steps < total_steps ? "stopped" : "complete";
	PX4_INFO("PLANT_SIM_RESULT steps=%llu sim_s=%.3f wall_s=%.3f realtime_factor=%.2f warmup_steps=%llu end=%s",
		 (unsigned long long)_steps, sim_s, wall_s, wall_s > 0 ? sim_s / wall_s : 0.0, (unsigned long long)warmup, end);
	snprintf(path, sizeof(path), "%s/summary.json", _output_dir);
	FILE *summary = fopen(path, "w");

	if (summary) {
		fprintf(summary, "{\"steps\": %llu, \"requested_steps\": %llu, \"end\": \"%s\", \"sim_s\": %.6f, \"wall_s\": %.6f, "
			"\"realtime_factor\": %.3f, \"warmup_steps\": %llu, \"wall_start_unix_us\": %llu, \"wall_end_unix_us\": %llu}\n",
			(unsigned long long)_steps, (unsigned long long)total_steps, end, sim_s, wall_s, wall_s > 0 ? sim_s / wall_s : 0.0,
			(unsigned long long)warmup, (unsigned long long)wall_start, (unsigned long long)wall_end);
		fclose(summary);
	}

	if (_trace) {
		fclose(_trace);
		_trace = nullptr;
	}

	// PX4's shutdown worker waits on the simulated clock, which only this module advances: keep it moving.
	px4_shutdown_request();

	for (int i = 0; i < 2500; i++) {
		advance_clock();
		usleep(1000);
	}
}

int PlantSim::print_status()
{
	PX4_INFO("steps %llu, sim time %.3f s", (unsigned long long)_steps, _steps * kStepUs * 1e-6);
	return 0;
}

int PlantSim::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("plant_sim", SCHED_DEFAULT, SCHED_PRIORITY_MAX, 4096, (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

PlantSim *PlantSim::instantiate(int argc, char *argv[])
{
	float duration = 20.f;
	const char *output = ".";
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	const char *scenario = "scenario.txt";

	while ((ch = px4_getopt(argc, argv, "d:o:s:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd': duration = (float)atof(myoptarg); break;

		case 'o': output = myoptarg; break;

		case 's': scenario = myoptarg; break;

		default: print_usage("unknown option"); return nullptr;
		}
	}

	return new PlantSim(duration, output, scenario);
}

int PlantSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PlantSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Aircraft plant (core + C Truth + C sensors) in lockstep inside PX4; see sitl/README.md.");
	PRINT_MODULE_USAGE_NAME("plant_sim", "simulation");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLOAT('d', 20.f, 0.f, 1e6f, "Simulated duration after the first actuator output [s]", true);
	PRINT_MODULE_USAGE_PARAM_STRING('o', ".", nullptr, "Output directory (trace.csv, summary.json)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('s', "scenario.txt", nullptr, "Scenario file (written by sitl/run_sitl.py)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int plant_sim_main(int argc, char *argv[])
{
	return PlantSim::main(argc, argv);
}
