#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/sensor_radiation.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/offboard_control_mode.h>


constexpr float THRESHOLD = 3.0f;
constexpr int AVG_COUNT = 15;
constexpr int MISSDIRECTION_STREAK = 3;
constexpr float STEP_MIN = 1.0f;
constexpr float STEP_MAX = 15.0f;
constexpr float STEP_K = STEP_MIN * 1.732f; /* sqrt(THRESHOLD): na pragu korak = 1 m */
constexpr float H_FLOOR = 0.01f;


typedef enum {START, NAVIGATE, REDIRECT, FLY, UNDEFINED_STATE, WAITING, SOURCE_FOUND} px4_state;
typedef enum {XPLUS = 0, XMINUS, YPLUS, YMINUS, UNDEFINED} px4_direction;

using namespace time_literals;

class RadiationDetector : public ModuleBase, public px4::ScheduledWorkItem
{

public:
	static Descriptor desc;

	RadiationDetector();
    	~RadiationDetector() override;

	static int task_spawn(int argc, char *argv[]);
    	static int custom_command(int argc, char *argv[]);
    	static int print_usage(const char *reason = nullptr);
    	int init();

	int print_status() override;

private:

	void Run() override;
	void choose_direction();
	void publishSetpoint(float x, float y, float z);
	void inspect_field();
	void saveRadiationMap();
	float stepFromIntensity(float intensity) const;
	bool arrived() const;

	sensor_radiation_s radiation;
	vehicle_local_position_s position;
	vehicle_local_position_s position_prev;
	trajectory_setpoint_s curr_setpoint;

	std::pair<float, vehicle_local_position_s> radioactive_field;
	std::vector<std::pair<float, vehicle_local_position_s>> radiation_in_directions;
	std::vector<std::pair<float, vehicle_local_position_s>> radiation_map;

	float target_z;
	float sum;
	int read_counter;
	int missdirection_counter = 0;


	bool skip_Y = false;
	bool skip_X = false;
	bool detected = false;
	bool start_ended = false;
	bool px4_stucked = false;


	float arrive_eps = 0.35f;
	float last_step = 1.0f;

	uint64_t search_start;

	px4_state curr_state;
	px4_state sub_state;
	px4_direction curr_direction;
	px4_direction prev_direction;

	uORB::SubscriptionCallbackWorkItem _radiation_sub{this,ORB_ID(sensor_radiation)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};

	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};

};
