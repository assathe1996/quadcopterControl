#include "observer.hpp"

static double prev_timestamp;

Observer::Observer() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1),
	_loop_perf(perf_alloc(PC_ELAPSED, "observer")) {
		set_firmware_dir();
		_current_state = get_state();
		_estimated_state = _current_state;

		prev_timestamp = _sensor_combined.timestamp;
}

Observer::~Observer() {
	perf_free(_loop_perf);
}

bool Observer::init() {
	ScheduleOnInterval(1000_us);
	return true;
}

// Private functions
void Observer::publish_acuator_controls() {
	_actuator_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
	_actuators.timestamp = hrt_absolute_time();
	static int response = orb_publish(ORB_ID(actuator_controls_0), _actuator_pub, &_actuators);
	memset(&_actuators, 0, sizeof(_actuators));

	if (response != 0)
		PX4_ERR("Actuator publish unsuccessful");
}

void Observer::publish_sensor_correction() {
	_sensor_correction_pub = orb_advertise(ORB_ID(sensor_correction), &_sensor_correction);
	_sensor_correction.timestamp = hrt_absolute_time();
	static int response = orb_publish(ORB_ID(sensor_correction), _sensor_correction_pub, &_sensor_correction);
	memset(&_sensor_correction, 0, sizeof(_sensor_correction));

	if (response != 0)
		PX4_ERR("Actuator publish unsuccessful");
}

void Observer::poll_sensor_combined() {
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
	orb_unsubscribe(_sensor_combined_sub);
}

void Observer::poll_sensor_mag() {
	_sensor_mag_sub = orb_subscribe(ORB_ID(sensor_mag));
	orb_copy(ORB_ID(sensor_mag), _sensor_mag_sub, &_sensor_mag);
	orb_unsubscribe(_sensor_mag_sub);
}

void Observer::poll_vehicle_acceleration() {
	_vehicle_acceleration_sub = orb_subscribe(ORB_ID(vehicle_acceleration));
	orb_copy(ORB_ID(vehicle_acceleration), _vehicle_acceleration_sub, &_vehicle_acceleration);
	orb_unsubscribe(_vehicle_acceleration_sub);
}

void Observer::poll_vehicle_angular_velocity() {
	_vehicle_angular_velocity_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
	orb_copy(ORB_ID(vehicle_angular_velocity), _vehicle_angular_velocity_sub, &_vehicle_angular_velocity);
	orb_unsubscribe(_vehicle_angular_velocity_sub);
}

void Observer::poll_vehicle_magnetometer() {
	_vehicle_magnetometer_sub = orb_subscribe(ORB_ID(vehicle_magnetometer));
	orb_copy(ORB_ID(vehicle_magnetometer), _vehicle_magnetometer_sub, &_vehicle_magnetometer);
	orb_unsubscribe(_vehicle_magnetometer_sub);
}

void Observer::poll_vechicle_local_position() {
	_vehicle_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	orb_copy(ORB_ID(vehicle_local_position), _vehicle_pos_sub, &_vehicle_pos);
	orb_unsubscribe(_vehicle_pos_sub);
}

void Observer::poll_vehicle_attitide() {
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_attitude);
	orb_unsubscribe(_vehicle_attitude_sub);
}

void Observer::set_firmware_dir() {
	static std::__fs::filesystem::path current_path = std::__fs::filesystem::current_path();
	firmware_dir = std::string(current_path);
	static size_t firmware_pos = firmware_dir.find("Firmware");
	firmware_dir = firmware_dir.substr(0, firmware_pos);
}

Matrix<float, 12, 1> Observer::get_state() {
	poll_vechicle_local_position();
	poll_vehicle_attitide();
	poll_vehicle_angular_velocity();

	static Matrix<float, 12, 1> state;

	state(0, 0) = _vehicle_pos.vx;
	state(1, 0) = _vehicle_pos.x;
	state(2, 0) = _vehicle_pos.vy;
	state(3, 0) = _vehicle_pos.y;
	state(4, 0) = _vehicle_pos.vz;
	state(5, 0) = _vehicle_pos.z;

	state(6, 0) = _vehicle_angular_velocity.xyz[0];
	state(8, 0) = _vehicle_angular_velocity.xyz[1];
	state(10, 0) = _vehicle_angular_velocity.xyz[2];

	state(7, 0) =  Eulerf(Quatf(_vehicle_attitude.q)).phi();
	state(9, 0) =  Eulerf(Quatf(_vehicle_attitude.q)).theta();
	state(11, 0) =  Eulerf(Quatf(_vehicle_attitude.q)).psi();

	return state;
}

Matrix<float,12, 1> Observer::integrator_estimator() {
	poll_sensor_combined();

	static Matrix<float, 12, 1> estimated_state;
	//const double us_to_s = 0.000001;
	//static float time_diff = (_sensor_combined.timestamp * us_to_s - prev_timestamp * us_to_s);
	//prev_timestamp = _sensor_combined.timestamp;
	float time_diff = 0.008000;


	estimated_state(0, 0) = _sensor_combined.accelerometer_m_s2[0] * time_diff;
	estimated_state(1, 0) += estimated_state(0, 0) * time_diff;
	estimated_state(2, 0) = _sensor_combined.accelerometer_m_s2[1] * time_diff;
	estimated_state(3, 0) += estimated_state(2, 0)  * time_diff;
	estimated_state(4, 0) = _sensor_combined.accelerometer_m_s2[2] * time_diff;
	estimated_state(5, 0) += estimated_state(3, 0) * time_diff;

	estimated_state(6, 0) = _sensor_combined.gyro_rad[0];
	estimated_state(7, 0) += estimated_state(6, 0) * time_diff;
	estimated_state(8, 0) = _sensor_combined.gyro_rad[1];
	estimated_state(9, 0) += estimated_state(8, 0) * time_diff;
	estimated_state(10, 0) = _sensor_combined.gyro_rad[2];
	estimated_state(11, 0) += estimated_state(10, 0) * time_diff;

	
	return estimated_state;
}

float Observer::calculate_rms(Matrix<float,12, 1> vector) {
	float rms_value = 0;
	const unsigned SIZE = 12;

	for (unsigned i = 0; i < SIZE; i++) {
		rms_value += (vector(i, 0) * vector(i, 0));
	}
	rms_value = sqrt(rms_value);
	return rms_value;
}

// ModuleBase functions
int Observer::task_spawn(int argc, char *argv[]) {
	Observer *instance = new Observer();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Observer::custom_command(int argc, char *argv[]) {
	return 0;
}

int Observer::print_usage(const char *reason) {
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR( Observer)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("observer", "controller");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// OVERRIDE functions
int Observer::print_status() {
	PX4_INFO("Running");
	perf_print_counter(_loop_perf);
	return 0;
}

void Observer::Run() {
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	_current_state = get_state();
	_estimated_state = integrator_estimator();

	float rms_error = calculate_rms(_current_state - _estimated_state);
	PX4_INFO("RMS error of all states: %f", double(rms_error));

	perf_end(_loop_perf);
}

int observer_main(int argc, char *argv[]){
	return Observer::main(argc, argv);
}
