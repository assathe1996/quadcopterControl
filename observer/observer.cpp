#include "observer.hpp"

const float g = -9.81;
static std::uint64_t previous_time;
static Matrix<float, 12, 1> prev_x_hat_dot;

Observer::Observer() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1),
	_loop_perf(perf_alloc(PC_ELAPSED, "observer")) {
		set_firmware_dir();
		_equilibrium_state = get_ekf_state();

		read_K();
		read_A_L_C();
		read_B();


		previous_time = hrt_absolute_time();
		_x_hat = complementary_filter();
		_y = get_ekf_state();

		for (int i = 0; i < 12; i++) {
			_current_state(i, 0) = 0;
			prev_x_hat_dot(i, 0) = 0;
		}
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

Matrix<float, 12, 1> Observer::get_ekf_state() {
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
	const float us_to_s = 0.000001;
	float time_diff_accel = _sensor_combined.accelerometer_integral_dt * us_to_s;
	float time_diff_gyro = _sensor_combined.gyro_integral_dt * us_to_s;

	estimated_state(0, 0) += _sensor_combined.accelerometer_m_s2[0] * time_diff_accel;
	estimated_state(1, 0) += estimated_state(0, 0) * time_diff_accel;
	estimated_state(2, 0) += _sensor_combined.accelerometer_m_s2[1] * time_diff_accel;
	estimated_state(3, 0) += estimated_state(2, 0)  * time_diff_accel;
	estimated_state(4, 0) += (_sensor_combined.accelerometer_m_s2[2] - g)* time_diff_accel;
	estimated_state(5, 0) += estimated_state(3, 0) * time_diff_accel;

	estimated_state(6, 0) = _sensor_combined.gyro_rad[0];
	estimated_state(7, 0) += estimated_state(6, 0) * time_diff_gyro;
	estimated_state(8, 0) = _sensor_combined.gyro_rad[1];
	estimated_state(9, 0) += estimated_state(8, 0) * time_diff_gyro;
	estimated_state(10, 0) = _sensor_combined.gyro_rad[2];
	estimated_state(11, 0) += estimated_state(10, 0) * time_diff_gyro;

	return estimated_state;
}

Matrix<float, 12, 1> Observer::complementary_filter() {
	poll_sensor_combined();
	poll_sensor_mag();

	poll_vechicle_local_position();
	poll_vehicle_angular_velocity();

	const float gyro_weight = 0.98f;
	const float accel_weight = 0.02f;

	static Matrix<float, 12, 1> estimated_state;
	const float us_to_s = 0.000001;
	float time_diff_gyro = _sensor_combined.gyro_integral_dt * us_to_s;

	estimated_state(0, 0) = _vehicle_pos.vx;
	estimated_state(1, 0) = _vehicle_pos.x;
	estimated_state(2, 0) = _vehicle_pos.vy;
	estimated_state(3, 0) = _vehicle_pos.y;
	estimated_state(4, 0) = _vehicle_pos.vz;
	estimated_state(5, 0) = _vehicle_pos.z;
	
	float roll_from_acc =  atan2(_sensor_combined.accelerometer_m_s2[1], sqrt(pow(_sensor_combined.accelerometer_m_s2[0], 2) + pow(_sensor_combined.accelerometer_m_s2[2], 2)));
	float pitch_from_acc = atan2(-_sensor_combined.accelerometer_m_s2[0], sqrt(pow(_sensor_combined.accelerometer_m_s2[1], 2) + pow(_sensor_combined.accelerometer_m_s2[2], 2)));
	
	float mag_length = sqrt(pow(_sensor_mag.x, 2) + pow(_sensor_mag.y, 2) + pow(_sensor_mag.z, 2));
	float mag_x = _sensor_mag.x / mag_length;
	float mag_y = _sensor_mag.y / mag_length;
	float mag_z = _sensor_mag.z / mag_length;


	float gyro_lpf_weight = 0.5;
	float p_lpf_x = (1.0f-gyro_lpf_weight)*estimated_state(6,0) + gyro_lpf_weight * _sensor_combined.gyro_rad[0];
	float p_lpf_y = (1.0f-gyro_lpf_weight)*estimated_state(8,0) + gyro_lpf_weight * _sensor_combined.gyro_rad[1];
	float p_lpf_z = (1.0f-gyro_lpf_weight)*estimated_state(10,0) + gyro_lpf_weight * _sensor_combined.gyro_rad[2];


	estimated_state(6, 0) = p_lpf_x;
	//estimated_state(6, 0) = std::accumulate(gyro_x_memory.begin(), gyro_x_memory.end(), 0.0f) / gyro_x_memory.size() ;
	estimated_state(7, 0) = gyro_weight * (estimated_state(7, 0) + estimated_state(6, 0) * time_diff_gyro) + accel_weight * roll_from_acc; 
	
	estimated_state(6, 0) = p_lpf_y;
	//estimated_state(8, 0) = _sensor_combined.gyro_rad[1];
	estimated_state(9, 0) = gyro_weight * (estimated_state(9, 0) + estimated_state(8, 0) * time_diff_gyro) + accel_weight * pitch_from_acc;

	float numerator = sin(estimated_state(7, 0))*mag_z - cos(estimated_state(7, 0))*mag_y;
	float denominator = cos(estimated_state(9, 0))*mag_x + sin(estimated_state(7, 0))*sin(estimated_state(9, 0))*mag_y + cos(estimated_state(9, 0))*sin(estimated_state(7, 0))*mag_z;
	float yaw_from_mag = atan2(numerator, denominator);

	estimated_state(6, 0) = p_lpf_z;
	//estimated_state(10, 0) = _sensor_combined.gyro_rad[2];
	estimated_state(11, 0) = gyro_weight * (estimated_state(11, 0) + estimated_state(10, 0) * time_diff_gyro) + accel_weight * yaw_from_mag;

	
	return estimated_state;
	/*poll_sensor_combined();
	poll_sensor_mag();
	//const float linear_vel_weight = 0.98f;
	//const float linear_accel_weight = 0.02f;
	const float gyro_weight = 0.98f;
	const float accel_weight = 0.02f;

	static Matrix<float, 12, 1> estimated_state;
	const float us_to_s = 0.000001;
	float time_diff_accel = _sensor_combined.accelerometer_integral_dt * us_to_s;
	float time_diff_gyro = _sensor_combined.gyro_integral_dt * us_to_s;

	estimated_state(0, 0) += _sensor_combined.accelerometer_m_s2[0] * time_diff_accel;
	estimated_state(1, 0) += estimated_state(0, 0) * time_diff_accel;
	estimated_state(2, 0) += _sensor_combined.accelerometer_m_s2[1] * time_diff_accel;
	estimated_state(3, 0) += estimated_state(2, 0)  * time_diff_accel;
	estimated_state(4, 0) += (_sensor_combined.accelerometer_m_s2[2] - g)* time_diff_accel;
	estimated_state(5, 0) += estimated_state(3, 0) * time_diff_accel;
	
	float roll_from_acc =  atan2(_sensor_combined.accelerometer_m_s2[1], sqrt(pow(_sensor_combined.accelerometer_m_s2[0], 2) + pow(_sensor_combined.accelerometer_m_s2[2], 2)));
	float pitch_from_acc = atan2(-_sensor_combined.accelerometer_m_s2[0], sqrt(pow(_sensor_combined.accelerometer_m_s2[1], 2) + pow(_sensor_combined.accelerometer_m_s2[2], 2)));
	
	float mag_length = sqrt(pow(_sensor_mag.x, 2) + pow(_sensor_mag.y, 2) + pow(_sensor_mag.z, 2));
	float mag_x = _sensor_mag.x / mag_length;
	float mag_y = _sensor_mag.y / mag_length;
	float mag_z = _sensor_mag.z / mag_length;

	estimated_state(6, 0) = _sensor_combined.gyro_rad[0];
	estimated_state(7, 0) = gyro_weight * (estimated_state(7, 0) + estimated_state(6, 0) * time_diff_gyro) + accel_weight * roll_from_acc; 
	estimated_state(8, 0) = _sensor_combined.gyro_rad[1];
	estimated_state(9, 0) = gyro_weight * (estimated_state(9, 0) + estimated_state(8, 0) * time_diff_gyro) + accel_weight * pitch_from_acc;

	float numerator = sin(estimated_state(7, 0))*mag_z - cos(estimated_state(7, 0))*mag_y;
	float denominator = cos(estimated_state(9, 0))*mag_x + sin(estimated_state(7, 0))*sin(estimated_state(9, 0))*mag_y + cos(estimated_state(9, 0))*sin(estimated_state(7, 0))*mag_z;
	float yaw_from_mag = atan2(numerator, denominator);

	estimated_state(10, 0) = _sensor_combined.gyro_rad[2];
	estimated_state(11, 0) = gyro_weight * (estimated_state(11, 0) + estimated_state(10, 0) * time_diff_gyro) + accel_weight * yaw_from_mag;
	
	return estimated_state;*/
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

void Observer::write_state(Matrix <float, 12, 1> state, std::string filename) {
	const unsigned SIZE = 12;
	hrt_abstime now = hrt_absolute_time();

	static std::ofstream outfile;
	static std::string current_path = firmware_dir + "Firmware/src/modules/observer/outputs/" + filename + ".txt";
	outfile.open(current_path, std::ios::out | std::ios::app);

	outfile << now << "\t";
	for (unsigned count = 0; count < SIZE; count++) {
		if (count == 11) {
			outfile << state(count, 0) << "\t";
			outfile << calculate_rms(state) << "\n";
		}
		else
			outfile << state(count, 0) << "\t";
	}
	outfile.close();
}

void Observer::write_magnetometer_vals() {
	static std::ofstream outfile;
	static std::string current_path = firmware_dir + "Firmware/src/modules/observer/outputs/" + "mag_val" + ".txt";
	outfile.open(current_path, std::ios::out | std::ios::app);

	poll_vechicle_local_position();
	poll_sensor_mag();

	outfile << _vehicle_pos.x << "\t";
	outfile << _vehicle_pos.y << "\t";
	outfile << _vehicle_pos.z << "\t";

	outfile << _sensor_mag.x << "\t";
	outfile << _sensor_mag.y << "\t";
	outfile << _sensor_mag.z << "\n";

	outfile.close();
}

void Observer::read_K() {
	const unsigned ROW_SIZE = 4;
	const unsigned COL_SIZE = 12;

	static std::ifstream infile;
	static std::string current_path = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/K.txt";
	infile.open(current_path);

	if (infile.is_open() == true) {
		for (unsigned row = 0; row < ROW_SIZE; row++) {
			std::string line;
			getline(infile, line);
			std::stringstream this_stream(line);
			for (unsigned column = 0 ; column < COL_SIZE; column++) {
				this_stream >> _K(row, column);
			}
		}
		infile.close();
	} else 
		PX4_ERR("K.txt not opened");
}

void Observer::read_A_L_C() {
	const unsigned SIZE = 12;

	static std::ifstream infile_A, infile_L, infile_C;
	static std::string current_path_A = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/A.txt";
	static std::string current_path_L = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/L.txt";
	static std::string current_path_C = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/C.txt";

	infile_A.open(current_path_A);
	infile_L.open(current_path_L);
	infile_C.open(current_path_C);

	if (infile_A.is_open() == true && infile_L.is_open() == true && infile_C.is_open() == true) {
		for (unsigned row = 0; row < SIZE; row++) {
			std::string line_A, line_L, line_C;
			getline(infile_A, line_A);
			getline(infile_L, line_L);
			getline(infile_C, line_C);

			std::stringstream this_stream_A(line_A);
			std::stringstream this_stream_L(line_L);
			std::stringstream this_stream_C(line_C);

			for (unsigned column = 0 ; column < SIZE; column++) {
				this_stream_A >> _A(row, column);
				this_stream_L >> _L(row, column);
				this_stream_C >> _C(row, column);
			}
		}
		infile_A.close();
		infile_L.close();
		infile_C.close();
	} else 
		PX4_ERR("A.txt or L.txt or C.txt not opened");	
}

void Observer::read_B() {
	const unsigned ROW_SIZE = 12;
	const unsigned COL_SIZE = 4;

	static std::ifstream infile;
	static std::string current_path = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/B.txt";
	infile.open(current_path);

	if (infile.is_open() == true) {
		for (unsigned row = 0; row < ROW_SIZE; row++) {
			std::string line;
			getline(infile, line);
			std::stringstream this_stream(line);
			for (unsigned column = 0 ; column < COL_SIZE; column++) {

				this_stream >> _B(row, column);
			}
		}
		infile.close();
	} else 
		PX4_ERR("B.txt not opened");	
}

void Observer::compute() {
	static Matrix<float, 12, 1> delta_x;

	delta_x = _equilibrium_state - _current_state;
	_u_control = _K * delta_x;
}

void Observer::normalize() {
    _u_control(1,0) = fmin(fmax((_u_control(1,0)), -1.0f), 1.0f);  
    _u_control(2,0) = fmin(fmax((_u_control(2,0)),  -1.0f), 1.0f);
    _u_control(3,0) = fmin(fmax((_u_control(3,0)), -1.0f), 1.0f);
    _u_control(0,0) = fmin(fmax((_u_control(0,0)), 0.0f), 1.0f) + 0.20f;
    /*_u_control(1, 0) = fmin(fmax((_u_control(1, 0)) / (0.1080f * 4.0f), -1.0f), 1.0f) * 0.4f;
    _u_control(2, 0) = fmin(fmax((_u_control(2, 0)) / (0.1080f * 4.0f), -1.0f), 1.0f) * 0.4f;
    _u_control(3, 0) = fmin(fmax((_u_control(3, 0)) / (0.1f * 4.0f), -1.0f), 1.0f) * 0.4f;
    _u_control(0, 0) = fmin(fmax((_u_control(0, 0) + 16.0f) / (16.0f), 0.0f), 1.0f);*/
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

	_y = get_ekf_state();
	_x_hat_dot = _A * _current_state + _B * (_u_control) +  _L * (_y -  _current_state);

	std::uint64_t time_diff = hrt_absolute_time() - previous_time;
	if (time_diff > 0.0) {
		_current_state += (_x_hat_dot) * time_diff * 1e-6 ;
		previous_time = hrt_absolute_time();
	}

	compute();
	normalize();
	
	write_state(_current_state, "linear_observer");
	write_state(_y, "ground_truth");


	perf_end(_loop_perf);
}

int observer_main(int argc, char *argv[]){
	return Observer::main(argc, argv);
}
