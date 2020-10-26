#include "lqr_att_controller.hpp"

static bool show_height;
static float height_setpoint;

LQRattControl::LQRattControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1),
	_loop_perf(perf_alloc(PC_ELAPSED, "lqr_att_control")) {
		set_firmware_dir();
		_equilibrium_state = get_state();
		read_K();

		show_height = false;
		height_setpoint = _equilibrium_state(5, 0);
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

void LQRattControl::compute() {
	static Matrix<float, 12, 1> delta_x;

	delta_x = _equilibrium_state - _current_state;
	_u_control = _K * delta_x;
}

void LQRattControl::normalize() {
	_u_control(1,0) = fmin(fmax((_u_control(1,0))/(0.1080f*4.0f), -1.0f), 1.0f);  
    _u_control(2,0) = fmin(fmax((_u_control(2,0))/(0.1080f*4.0f),  -1.0f), 1.0f);
    _u_control(3,0) = fmin(fmax((_u_control(3,0))/(0.1f*1.0f), -1.0f), 1.0f);
    _u_control(0,0) = fmin(fmax((_u_control(0,0)+5.886f)/16.0f, 0.0f), 1.0f);
}

void LQRattControl::display() {
	//PX4_INFO("%f %f %f %f", double(_u_control(0, 0)), double(_u_control(1, 0)),
	//	double(_u_control(2, 0)), double(_u_control(3, 0)));
	if (show_height == true) {
		PX4_INFO("Current height: %f", double(_vehicle_pos.z));
		show_height = false;
	}

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
		if (!strncmp(argv[0], "show_height", 11)) {
			command_present = true;
			show_height = true;
		}
	}

	if (argc == 2) {
		if (!strncmp(argv[0], "set_height", 10)) {
			command_present = true;
			static std::string::size_type sz;
			height_setpoint = std::stof (argv[1], &sz); 
			PX4_INFO("Setting height to: %lf", double(height_setpoint));
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
	_current_state = get_state();

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
