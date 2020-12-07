#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_mag.h>

#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <vector>
#include <numeric>

using namespace matrix;
using namespace time_literals;

extern "C" __EXPORT int lqr_att_controller_main(int argc, char *argv[]);

class LQRattControl : public ModuleBase<LQRattControl>, public ModuleParams, public px4::ScheduledWorkItem {

public:
	LQRattControl();
	virtual ~LQRattControl();

	bool init();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	void Run() override;

private:
	orb_advert_t 	_actuator_pub{nullptr};

	int _vehicle_pos_sub{-1};
	int _vehicle_attitude_sub{-1};
	int _vehicle_angular_velocity_sub{-1};

	struct actuator_controls_s			_actuators {};
	struct vehicle_local_position_s		_vehicle_pos{};
	struct vehicle_attitude_s			_vehicle_attitude{};
	struct vehicle_angular_velocity_s	_vehicle_angular_velocity{};

	Matrix<float, 12, 1> _equilibrium_state;
	Matrix<float, 12, 1> _current_state;
	Matrix<float, 4, 1> _u_control;
	Matrix<float, 4, 12> _K;

	Matrix<float, 12, 12> _A;
	Matrix<float, 12, 12> _L;
	Matrix<float, 12, 12> _C;
	Matrix<float, 12, 4> _B;
	Matrix<float, 12, 1> _x_hat;
	Matrix<float, 12, 1> _x_hat_dot;
	Matrix<float, 12, 1> _y;

	std::string firmware_dir;
	perf_counter_t	_loop_perf;

	void poll_vechicle_local_position();
	void poll_vehicle_attitide();
	void poll_vehicle_angular_velocity();

	void publish_acuator_controls();

	Matrix<float, 12, 1> get_ekf_state();
	void set_equilibrium_state();
	void set_firmware_dir();
	void read_K();
	void read_A_L_C();
	void read_B();
	void write_state(Matrix <float, 12, 1> state);

	void compute();
	void normalize();
	void display();


	Matrix<float, 12, 1> complementary_filter();
	void poll_sensor_combined();
	void poll_sensor_mag();
	float calculate_rms(Matrix<float,12, 1> vector);

	int _sensor_combined_sub{-1};
	int _sensor_mag_sub{-1};

	struct sensor_combined_s            _sensor_combined {};
	struct sensor_mag_s                 _sensor_mag {};


};

