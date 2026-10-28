#include "radiation_detector.hpp"

#include <px4_platform_common/log.h>

#include <stdio.h>
#include <cmath>



ModuleBase::Descriptor RadiationDetector::desc{task_spawn, custom_command, print_usage};

RadiationDetector::RadiationDetector() :
	ModuleBase(),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_trajectory_setpoint_pub.advertise();
	_offboard_control_mode_pub.advertise();

}

RadiationDetector::~RadiationDetector()
{
	_radiation_sub.unregisterCallback();
}


int RadiationDetector::init(){

	if (!_radiation_sub.registerCallback()) {
		return PX4_ERROR;
	}

	curr_state = START;
	sub_state = START;
	curr_direction = XPLUS;
	target_z = -2.0;

	radioactive_field.first = -1.0f;
	search_start = 0;

	curr_setpoint = {};

	curr_setpoint.position[0] = 0.0f;
	curr_setpoint.position[1] = 0.0f;
	curr_setpoint.position[2] = target_z;

	curr_setpoint.velocity[0] = NAN;
	curr_setpoint.velocity[1] = NAN;
	curr_setpoint.velocity[2] = NAN;

	curr_setpoint.acceleration[0] = NAN;
	curr_setpoint.acceleration[1] = NAN;
	curr_setpoint.acceleration[2] = NAN;

	curr_setpoint.jerk[0] = NAN;
	curr_setpoint.jerk[1] = NAN;
	curr_setpoint.jerk[2] = NAN;

	curr_setpoint.yaw = NAN;
	curr_setpoint.yawspeed = NAN;

	ScheduleNow();

	return OK;
}


int RadiationDetector::task_spawn(int argc, char *argv[]){

	RadiationDetector *instance = new RadiationDetector();

	if(!instance){
		PX4_ERR("radiation detector allocation failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if(instance->init() != PX4_OK){
		delete instance;
		desc.object.store(nullptr);
		desc.task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int RadiationDetector::custom_command(int argc, char *argv[]){
	return print_usage("unknown command");
}

int RadiationDetector::print_status(){
	PX4_INFO("Running on work queue");
	return 0;
}

int RadiationDetector::print_usage(const char *reason){

	if (reason) {
		PX4_WARN("%s\n", reason);
	}
	PRINT_MODULE_DESCRIPTION("radiation detection module");
	PRINT_MODULE_USAGE_NAME("radiation_detector", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	return 0;
}

float RadiationDetector::stepFromIntensity(float intensity) const
{
	const float h = (intensity > H_FLOOR) ? intensity : H_FLOOR;
	float step = STEP_K / sqrtf(h);

	if (step < STEP_MIN) {
		step = STEP_MIN;
	}

	if (step > STEP_MAX) {
		step = STEP_MAX;
	}

	return step;
}

bool RadiationDetector::arrived() const
{
	const float dx = fabsf(position.x - curr_setpoint.position[0]);
	const float dy = fabsf(position.y - curr_setpoint.position[1]);
	return (dx <= arrive_eps) && (dy <= arrive_eps);
}

void RadiationDetector::Run(){


	offboard_control_mode_s offboard_control_mode{};
	offboard_control_mode.timestamp = hrt_absolute_time();

	offboard_control_mode.position = true;
	offboard_control_mode.velocity = false;
	offboard_control_mode.acceleration = false;
	offboard_control_mode.attitude = false;
	offboard_control_mode.body_rate = false;

	_offboard_control_mode_pub.publish(offboard_control_mode);


	curr_setpoint.timestamp = hrt_absolute_time();

	_trajectory_setpoint_pub.publish(curr_setpoint);


	if(_radiation_sub.update(&radiation)){

        	if (_local_pos_sub.copy(&position)) {


			switch(curr_state){

				case START:{
					position_prev = position;
					if(position_prev.z > target_z) {
						curr_state = START;
						break;
					}
					search_start = hrt_absolute_time();
					radiation_map.push_back({radiation.radiation_reading, position});
					curr_state = NAVIGATE;
					break;

				}
				case NAVIGATE:{
					choose_direction();
					break;
				}
				case FLY:{
					inspect_field();
					break;
				}
				case REDIRECT:{
					break;
				}
				case UNDEFINED_STATE:{
					break;
				}
				case WAITING:{
					if (!arrived()) {
						break;
					}

					if(sub_state == FLY){
						curr_state = FLY;
						sub_state = UNDEFINED_STATE;
						break;
					}
					if(sub_state == REDIRECT){
						position_prev = position;
						curr_state = NAVIGATE;
						sub_state = UNDEFINED_STATE;

						if(prev_direction == XPLUS || prev_direction == XMINUS){
							curr_direction = YPLUS;
						}else{
							curr_direction = XPLUS;
						}
						break;
					}
					if(start_ended){
						sub_state = FLY;
						start_ended = false;
						px4_stucked = false;
						curr_state = FLY;
						break;
					}
					curr_state = NAVIGATE;
					if(skip_Y && prev_direction == XMINUS){
						curr_direction = UNDEFINED;
						break;
					}
					curr_direction = static_cast<px4_direction>((int)prev_direction + 1);
					break;
				}
				case SOURCE_FOUND:{
					uint64_t elapsed_s = (hrt_absolute_time() - search_start) / 1000000ULL;
					PX4_INFO("SOURCE FOUND:\n");
					PX4_INFO("RADIATION: %.2f", (double)radioactive_field.first);
					PX4_INFO("Position: x=%.2f y=%.2f z=%.2f",
						(double)radioactive_field.second.x,
						(double)radioactive_field.second.y,
						(double)radioactive_field.second.z);

					PX4_INFO("SEARCH TIME: %llu min %llu s",
         					(unsigned long long)(elapsed_s / 60),
         					(unsigned long long)(elapsed_s % 60));

					saveRadiationMap();

					curr_state = UNDEFINED_STATE;
					break;
				}

			}

		}
		else {
			PX4_ERR("Failed to update vehicle_local_position");
		}
	}

	ScheduleDelayed(50_ms);
}

void RadiationDetector::inspect_field(){
	sum += radiation.radiation_reading;
	read_counter++;

	if(read_counter >= AVG_COUNT){
		sum /= AVG_COUNT;

		const float step = stepFromIntensity(sum);

		if(!skip_X && !skip_Y && sum < radiation_map.back().first) {


			sum = 0.0;
			read_counter = 0;
			missdirection_counter++;

			if(missdirection_counter >= MISSDIRECTION_STREAK){

				if(detected){
					radiation_map.push_back({radioactive_field.first, radioactive_field.second});
					curr_state = SOURCE_FOUND;
					return;
				}
				px4_stucked = true;

			}
			curr_state = NAVIGATE; sub_state = REDIRECT;
			position_prev = position;
 			return;
		}


		if(sum >= THRESHOLD){
			detected = true;
			if(sum > radioactive_field.first){
				radioactive_field.first =  sum;
				radioactive_field.second = position;
			}
		}

		if(!skip_X && !skip_Y){
			radiation_map.push_back({sum, position});
			missdirection_counter = 0;
		}else{
			skip_X = false; skip_Y = false;
		}


		sum = 0.0;
		read_counter = 0;

		position_prev = position;

		switch(curr_direction){
			case XPLUS:{
				publishSetpoint(position.x + step, position.y, target_z);
				break;
			}

			case XMINUS:{
				publishSetpoint(position.x - step, position.y, target_z);
				break;
			}

			case YPLUS:{
				publishSetpoint(position.x, position.y + step, target_z);
				break;
			}

			case YMINUS:{
				publishSetpoint(position.x, position.y - step, target_z);
				break;
			}
			case UNDEFINED:{
				break;
			}
		}

		curr_state = WAITING;
		sub_state = FLY;


	}
}

void RadiationDetector::choose_direction(){
	float x = position_prev.x;
	float y = position_prev.y;
	const float probe = stepFromIntensity(radiation.radiation_reading);

	switch(curr_direction){
		case XPLUS:{
			if(sub_state == REDIRECT){
				x = position_prev.x - probe;
				prev_direction = XPLUS;
				curr_state = WAITING;
				skip_X = true;
				break;
			}
			x = position_prev.x + probe;
			curr_state = WAITING;
			prev_direction = XPLUS;
			break;
		}
		case XMINUS:{
			if(sub_state == REDIRECT){
				x = position_prev.x + probe;
				prev_direction = XMINUS;
				curr_state = WAITING;
				skip_X = true;
				break;
			}
			sum += radiation.radiation_reading;
			read_counter++;

			if(read_counter >= AVG_COUNT){
				sum /= AVG_COUNT;
			}else{
				return;
			}
			radiation_in_directions.push_back({sum, position});
			x = position_prev.x - probe;
			curr_state = WAITING;
			prev_direction = XMINUS;
			sum = 0; read_counter = 0;
			break;
		}
		case YPLUS:{
			if(sub_state == REDIRECT){
				y = position_prev.y - probe;
				prev_direction = YPLUS;
				curr_state = WAITING;
				skip_Y = true;
				break;
			}
			if(!skip_X){
				sum += radiation.radiation_reading;
				read_counter++;

				if(read_counter >= AVG_COUNT){
					sum /= AVG_COUNT;
				}else{
					return;
				}
				radiation_in_directions.push_back({sum, position});
			}

			y = position_prev.y + probe;
			curr_state = WAITING;
			prev_direction = YPLUS;
			sum = 0; read_counter = 0;
			break;
		}
		case YMINUS:{
			if(sub_state == REDIRECT){
				y = position_prev.y + probe;
				prev_direction = YMINUS;
				curr_state = WAITING;
				skip_Y = true;
				break;
			}
			sum += radiation.radiation_reading;
			read_counter++;

			if(read_counter >= AVG_COUNT){
				sum /= AVG_COUNT;
			}else{
				return;
			}
			radiation_in_directions.push_back({sum, position});
			y = position_prev.y - probe;
			curr_state = WAITING;
			prev_direction = YMINUS;
			sum = 0; read_counter = 0;
			break;
		}
		case UNDEFINED:{
			sum += radiation.radiation_reading;
			read_counter++;

			if(read_counter >= AVG_COUNT){
				sum /= AVG_COUNT;
			}else{
				return;
			}
			radiation_in_directions.push_back({sum, position});
			sum = 0; read_counter = 0;
			float max = -1.0;
			int n = radiation_in_directions.size();
			for(int i = 0; i < n; i++){
				if(radiation_in_directions[i].first > max){
					max = radiation_in_directions[i].first;
					curr_direction = skip_X ? static_cast<px4_direction>(i + 2) : static_cast<px4_direction>(i);
				}
			}
			radiation_in_directions.clear();

			const float step = stepFromIntensity(max);
			x = position_prev.x;
			y = position_prev.y;
			switch (curr_direction) {
				case XPLUS:  x += step; break;
				case XMINUS: x -= step; break;
				case YPLUS:  y += step; break;
				case YMINUS: y -= step; break;
				default: break;
			}

			curr_state = WAITING;
			if(sub_state == START || px4_stucked){
				start_ended = true;
			}else{
				sub_state = FLY;
			}
			break;

		}

	}
	publishSetpoint(x, y, target_z);
}

void RadiationDetector::publishSetpoint(float x, float y, float z)
{
	const float dx = x - position.x;
	const float dy = y - position.y;
	last_step = sqrtf(dx * dx + dy * dy);

	arrive_eps = 0.20f * last_step;
	if (arrive_eps < 0.35f) {
		arrive_eps = 0.35f;
	}

	curr_setpoint.position[0] = x;
	curr_setpoint.position[1] = y;
	curr_setpoint.position[2] = z;
}


void RadiationDetector::saveRadiationMap(){
	FILE *file = fopen("/tmp/radiation_map.csv", "w");

	if (file == nullptr) {
		PX4_ERR("Failed to open radiation_map.csv");
		return;
	}

	fprintf(file, "x,y,radiation\n");

	for (const auto &measurement : radiation_map) {
		const float radiation_csv = measurement.first;
		const float x_csv = measurement.second.x;
		const float y_csv = measurement.second.y;

		fprintf(file, "%.3f,%.3f,%.3f\n",
			(double)x_csv,
			(double)y_csv,
			(double)radiation_csv);
	}

	fclose(file);
}

extern "C" __EXPORT int radiation_detector_main(int argc, char *argv[])
{
	return ModuleBase::main(RadiationDetector::desc, argc, argv);
}
