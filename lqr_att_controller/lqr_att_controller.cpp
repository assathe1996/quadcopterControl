#include "lqr_att_controller.hpp"

static bool show_height;
static float height_setpoint;
static float x_setpoint;
static float y_setpoint;
static float yaw_setpoint;
//static bool height_entered = false;

LQRattControl::LQRattControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1),
	_loop_perf(perf_alloc(PC_ELAPSED, "lqr_att_control")) {
		set_firmware_dir();
		_equilibrium_state = get_state();
		read_K();

		show_height = false;
		x_setpoint = _equilibrium_state(1, 0);
		y_setpoint = _equilibrium_state(3, 0);
		height_setpoint = _equilibrium_state(5, 0);
		yaw_setpoint = _equilibrium_state(11, 0);
		memset(&_actuators, 0, sizeof(_actuators));
}

LQRattControl::~LQRattControl() {
	perf_free(_loop_perf);
}

bool LQRattControl::init() {
	ScheduleOnInterval(1000_us);
	return true;
}

// Private functions
void LQRattControl::poll_vechicle_local_position() {
	_vehicle_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	orb_copy(ORB_ID(vehicle_local_position), _vehicle_pos_sub, &_vehicle_pos);
	orb_unsubscribe(_vehicle_pos_sub);
}

void LQRattControl::poll_vehicle_attitide() {
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_attitude);
	orb_unsubscribe(_vehicle_attitude_sub);
}

void LQRattControl::poll_vehicle_angular_velocity() {
	_vehicle_angular_velocity_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
	orb_copy(ORB_ID(vehicle_angular_velocity), _vehicle_angular_velocity_sub, &_vehicle_angular_velocity);
	orb_unsubscribe(_vehicle_angular_velocity_sub);
}

void LQRattControl::publish_acuator_controls() {
	_actuator_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
	_actuators.timestamp = hrt_absolute_time();
	static int response = orb_publish(ORB_ID(actuator_controls_0), _actuator_pub, &_actuators);
	memset(&_actuators, 0, sizeof(_actuators));

	if (response != 0)
		PX4_ERR("Actuator publish unsuccessful");
}

Matrix<float, 12, 1> LQRattControl::get_state() {
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

void LQRattControl::set_equilibrium_state() {
	_equilibrium_state(5, 0) = height_setpoint;
	_equilibrium_state(1, 0) = x_setpoint;
	_equilibrium_state(3, 0) = y_setpoint;
	_equilibrium_state(11, 0) = yaw_setpoint;	
}

void LQRattControl::set_firmware_dir() {
	static std::__fs::filesystem::path current_path = std::__fs::filesystem::current_path();
	firmware_dir = std::string(current_path);
	static size_t firmware_pos = firmware_dir.find("Firmware");
	firmware_dir = firmware_dir.substr(0, firmware_pos);
}

void LQRattControl::read_K() {
	const unsigned SIZE = 12;

	static std::ifstream infile;
	static std::string current_path = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/K.txt";
	infile.open(current_path);

	if (infile.is_open() == true) {
		for (unsigned row = 0; row < SIZE; row++) {
			std::string line;
			getline(infile, line);
			std::stringstream this_stream(line);
			for (unsigned column = 0 ; column < SIZE; column++) 
				this_stream >> _K(row, column);
		}
		infile.close();
	} else 
		PX4_ERR("K.txt not opened");
}

void LQRattControl::write_state(Matrix <float, 12, 1> state) {
	const unsigned SIZE = 12;
	hrt_abstime now = hrt_absolute_time();

	static std::ofstream outfile;
	static std::string current_path = firmware_dir + "Firmware/src/modules/lqr_att_controller/model_params/state.txt";
	outfile.open(current_path, std::ios::out | std::ios::app);

	outfile << now << "\t";
	for (unsigned count = 0; count < SIZE; count++) {
		if (count == 11)
			outfile << state(count, 0) << "\n";
		else
			outfile << state(count, 0) << "\t";
	}
	outfile.close();
}

void LQRattControl::compute() {
	static Matrix<float, 12, 1> delta_x;

	delta_x = _equilibrium_state - _current_state;
	_u_control = _K * delta_x;
}

void LQRattControl::normalize() {
    _u_control(1,0) = fmin(fmax((_u_control(1,0)), -1.0f), 1.0f);  
    _u_control(2,0) = fmin(fmax((_u_control(2,0)),  -1.0f), 1.0f);
    _u_control(3,0) = fmin(fmax((_u_control(3,0)), -1.0f), 1.0f);
    _u_control(0,0) = fmin(fmax((_u_control(0,0)), 0.0f), 1.0f) + 0.20f;
}

void LQRattControl::display() {
	if (show_height == true) {

		poll_vechicle_local_position();
		poll_vehicle_attitide();
		poll_vehicle_angular_velocity();

		PX4_INFO("height: %f x: %f y: %f yaw: %f", double(_vehicle_pos.z), double(_vehicle_pos.x), double(_vehicle_pos.y), double(Eulerf(Quatf(_vehicle_attitude.q)).psi()));
		show_height = false;
	}

}

Matrix<float, 12, 1> LQRattControl::complementary_filter() {
	poll_sensor_combined();
	poll_sensor_mag();

	poll_vechicle_local_position();

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

	estimated_state(6, 0) = _sensor_combined.gyro_rad[0];
	estimated_state(7, 0) = gyro_weight * (estimated_state(7, 0) + estimated_state(6, 0) * time_diff_gyro) + accel_weight * roll_from_acc; 
	estimated_state(8, 0) = _sensor_combined.gyro_rad[1];
	estimated_state(9, 0) = gyro_weight * (estimated_state(9, 0) + estimated_state(8, 0) * time_diff_gyro) + accel_weight * pitch_from_acc;

	float numerator = sin(estimated_state(7, 0))*mag_z - cos(estimated_state(7, 0))*mag_y;
	float denominator = cos(estimated_state(9, 0))*mag_x + sin(estimated_state(7, 0))*sin(estimated_state(9, 0))*mag_y + cos(estimated_state(9, 0))*sin(estimated_state(7, 0))*mag_z;
	float yaw_from_mag = atan2(numerator, denominator);

	estimated_state(10, 0) = _sensor_combined.gyro_rad[2];
	estimated_state(11, 0) = gyro_weight * (estimated_state(11, 0) + estimated_state(10, 0) * time_diff_gyro) + accel_weight * yaw_from_mag;
	
	return estimated_state;
}

void LQRattControl::poll_sensor_combined() {
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
	orb_unsubscribe(_sensor_combined_sub);
}

void LQRattControl::poll_sensor_mag() {
	_sensor_mag_sub = orb_subscribe(ORB_ID(sensor_mag));
	orb_copy(ORB_ID(sensor_mag), _sensor_mag_sub, &_sensor_mag);
	orb_unsubscribe(_sensor_mag_sub);
}

// ModuleBase functions
int LQRattControl::task_spawn(int argc, char *argv[]) {
	LQRattControl *instance = new LQRattControl();

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

int LQRattControl::custom_command(int argc, char *argv[]) {
	static int ret; 
	static bool command_present = false;
	
	if (argc == 1) {
		if (!strncmp(argv[0], "show_data", 9)) {
			command_present = true;
			show_height = true;
		}
	}

	if (argc == 2) {
		if (!strncmp(argv[0], "set_height", 10)) {
			command_present = true;
			static std::string::size_type sz;
			height_setpoint = std::stof (argv[1], &sz); 
			PX4_INFO("Setting height to: %f", double(height_setpoint));
		}
	}

	if (argc == 2) {
		if (!strncmp(argv[0], "set_x", 5)) {
			command_present = true;
			static std::string::size_type sz;
			x_setpoint = std::stof (argv[1], &sz); 
			PX4_INFO("Setting X to: %f", double(x_setpoint));
		}
	}

	if (argc == 2) {
		if (!strncmp(argv[0], "set_y", 5)) {
			command_present = true;
			static std::string::size_type sz;
			y_setpoint = std::stof (argv[1], &sz); 
			PX4_INFO("Setting Y to: %f", double(y_setpoint));
		}
	}

	if (argc == 2) {
		if (!strncmp(argv[0], "set_yaw", 7)) {
			command_present = true;
			static std::string::size_type sz;
			yaw_setpoint = std::stof (argv[1], &sz); 
			PX4_INFO("Setting yaw to: %f", double(yaw_setpoint));
		}
	}

	if (command_present == false) {ret = print_usage("unknown command");}
	else {ret = 0;}

	return ret;
}

int LQRattControl::print_usage(const char *reason) {
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR( LQRattControl)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("lqr_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "|Starts the controller");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set_height <value>", "|Sets height setpoint of the vehicle");
	PRINT_MODULE_USAGE_ARG("Argument:<value>", "|(float) *UP direction is negative", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("show_height", "|Displays cuurent height of the vehicle");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// OVERRIDE functions
int LQRattControl::print_status() {
	PX4_INFO("Running");
	perf_print_counter(_loop_perf);
	return 0;
}

void LQRattControl::Run() {
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	set_equilibrium_state();
	_current_state = complementary_filter();
	//_current_state = get_state();

	compute();
	normalize();
	display();

	_actuators.control[0] = _u_control(1, 0);
	_actuators.control[1] = _u_control(2, 0);
	_actuators.control[2] = _u_control(3, 0);
	_actuators.control[3] = _u_control(0, 0);
	publish_acuator_controls();
	
	perf_end(_loop_perf);
}

int lqr_att_controller_main(int argc, char *argv[]){
	return LQRattControl::main(argc, argv);
}
