#include "QuadrotorLQRControl.hpp"

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_tasks.h>

//#include <eigen3/Eigen/Dense> // ++ Math operations with Matrices ++

/* -------------------- added part ---------------------*/
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <iostream>
#include <cmath>
#include <memory>
#include <sstream>
#include <cerrno>

using namespace matrix;
using namespace std;

QuadrotorLQRControl::QuadrotorLQRControl()
{

    for (int i=0;i<12;i++)
    {
       _current_state(i,0) = 0.0f;
       _eq_point(i,0) = 0.0f;
    }

    _eq_point(1,0) =  0.0f;
    _eq_point(3,0) =  0.0f;
    _eq_point(5,0) = -1.0f;

    u_control(0,0) = 0.0f;
    u_control(1,0) = 0.0f;
    u_control(2,0) = 0.0f;
    u_control(3,0) = 0.0f;

    _mag_readings(0,0) = 0.0f;
    _mag_readings(1,0) = 0.0f;
    _mag_readings(2,0) = 0.0f;
    //_K = readMatrixK("~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/new_controller.txt");

    // char command[65] = "ls ~/px4-19/Firmware/src/modules/mc_att_control/lqr_files";
    // system(command);

    //_K = readMatrixK("~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/tgmK.txt");
    _K = readMatrixK("tgmK.txt");
      
    //_K = readMatrixK("lqr_files/tgmK.txt");
    //_P = readMatrixP("~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/tgmP.txt");
    _P = readMatrixP("tgmP.txt");
    ff_thrust = 5.886f; // [N]

    _auto_eq_point_flag = true;

    ofstream outfile1;
     outfile1.open("~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/control_input.txt", std::ios::out);
     outfile1.close();

        
     ofstream outfile3;
     outfile3.open("~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/state.txt", std::ios::out);
     outfile3.close();

     
     ofstream outfile5;
     outfile5.open("~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/lyapunov.txt", std::ios::out);
     outfile5.close();

    ofstream outfile4;
     outfile4.open("~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/ekf.txt", std::ios::out);
     outfile4.close();

     _past_time = hrt_absolute_time() * 1e-6;

}



Matrix <float, 4, 12>  QuadrotorLQRControl::readMatrixK(const char *filename)
    {

    static Matrix <float, 4, 12> result;
    static int rows = 4;
    static int cols = 12;
    ifstream infile (filename);
    PX4_INFO("trying to read K at %s", filename);
    //char command[120] = "if test -f ~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/control_input.txt; then echo 'exists'; else echo 'no exist'; fi;";
    //char command[100] = "ls ~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/";
    // char command[10] = "pwd";
    // system(command);
    cout << strerror(errno) << '\n';
    if (infile.is_open()){
      PX4_INFO("K file OPENED!");
         for (int i=0; i<rows;i++){
    		string line;
    		getline(infile, line);
    		stringstream stream(line);
    		for (int j=0; j<cols; j++){
    			stream >> result(i,j);
    		}

    	}
    	infile.close();
    }else cout << "Unable to open file\r\n";
    return result;

 }

Matrix<float, 12, 1> QuadrotorLQRControl::complementary_filter() {

  const float gyro_weight = 0.98f;
  const float accel_weight = 0.02f;
  // set this to 1 to not filter at all, the closer to 0, the more weight given to old measurements
  const float gyro_lpf_weight = 1.0f;

  static Matrix<float, 12, 1> estimated_state;
  float time_diff_gyro = _gyro_readings(3,0);

  estimated_state(0, 0) = _current_state(0,0);
  estimated_state(1, 0) = _current_state(1,0);
  estimated_state(2, 0) = _current_state(2,0);
  estimated_state(3, 0) = _current_state(3,0);
  estimated_state(4, 0) = _current_state(4,0);
  estimated_state(5, 0) = _current_state(5,0);

  float roll_from_acc =  atan2(_acc_readings(1,0), sqrt(pow(_acc_readings(0,0), 2) + pow(_acc_readings(2,0), 2)));
  float pitch_from_acc = atan2(-_acc_readings(0,0), sqrt(pow(_acc_readings(1,0), 2) + pow(_acc_readings(2,0), 2)));
  
  float mag_length = sqrt(pow(_mag_readings(0,0), 2) + pow(_mag_readings(1,0), 2) + pow(_mag_readings(2,0), 2));
  float mag_x = _mag_readings(0,0) / mag_length;
  float mag_y = _mag_readings(1,0) / mag_length;
  float mag_z = _mag_readings(2,0) / mag_length;

  //estimated_state(6, 0) = _sensor_combined.gyro_rad[0];
  float p_lpf = (1.0f-gyro_lpf_weight)*estimated_state(6,0) + gyro_lpf_weight*_gyro_readings(0,0);
  float p_raw = p_lpf;//_gyro_readings(0,0);
  estimated_state(6,0) = p_lpf;
  estimated_state(7, 0) = gyro_weight * (estimated_state(7, 0) + p_raw * time_diff_gyro) + accel_weight * roll_from_acc; 
  //estimated_state(8, 0) = _sensor_combined.gyro_rad[1];
  float q_lpf = (1.0f-gyro_lpf_weight)*estimated_state(8,0) + gyro_lpf_weight*_gyro_readings(1,0);
  float q_raw = q_lpf;//_gyro_readings(1,0);
  estimated_state(8,0) = q_lpf;
  estimated_state(9, 0) = gyro_weight * (estimated_state(9, 0) + q_raw * time_diff_gyro) + accel_weight * pitch_from_acc;

  float numerator = sin(estimated_state(7, 0))*mag_z - cos(estimated_state(7, 0))*mag_y;
  float denominator = cos(estimated_state(9, 0))*mag_x + sin(estimated_state(7, 0))*sin(estimated_state(9, 0))*mag_y + cos(estimated_state(9, 0))*sin(estimated_state(7, 0))*mag_z;
  float yaw_from_mag = atan2(numerator, denominator);

  // messed around with assuming theta, phi small when using mag data to estimate heading
  // didn't make much of a difference
  // float numerator = mag_y;
  // float denominator = mag_x;
  // float yaw_from_mag = -atan2(numerator, denominator);

  //estimated_state(10, 0) = _sensor_combined.gyro_rad[2];
  estimated_state(10, 0) = (1.0f-gyro_lpf_weight)*estimated_state(10,0) + gyro_lpf_weight*_gyro_readings(2,0);
  estimated_state(11, 0) = gyro_weight * (estimated_state(11, 0) + estimated_state(10, 0) * time_diff_gyro) + accel_weight * yaw_from_mag;
  // was also messing around with using only the mag data to get heading - it's much noisier
  //estimated_state(11, 0) = gyro_weight;
  
  return estimated_state;
}

Matrix<float,4,1> QuadrotorLQRControl::LQRcontrol()
{
       
     
     //static Matrix<float,4,1> u_control;
     static Matrix<float,4,1> u_control_norm;
     static Matrix<float,12,1> state_est;
     static Matrix<float,12,1> delta_x;
     //static Matrix<float,12,1> delta_x_est;
     static Matrix<float, 1,12> v_b;
     static Matrix<float,1,12> delta_x_T;
     static Matrix<float,1,1> _lyap_fun;     
     const hrt_abstime now = hrt_absolute_time();
     float _current_time = now *1e-6;
    // float dt = _current_time-_past_time;
     
    _past_time = _current_time;
    state_est = complementary_filter();
    delta_x   = _current_state - _eq_point; 
    //delta_x_est = state_est - _eq_point;

    u_control = - _K*(delta_x);
    
    delta_x_T = delta_x.transpose();
    
    v_b = delta_x_T*_P;
    _lyap_fun = v_b*delta_x;
    //cout<< dt << "\t" << _P(0,0) << "\n";
   // !! IMPORTANT scale the control inputs.......


    // u_control_norm(1,0) = fmin(fmax((u_control(1,0))/(0.1080f*4.0f), -1.0f), 1.0f);  
    // u_control_norm(2,0) = fmin(fmax((u_control(2,0))/(0.1080f*4.0f),  -1.0f), 1.0f);
    // u_control_norm(3,0) = fmin(fmax((u_control(3,0))/(0.1f*1.0f), -1.0f), 1.0f);
    // u_control_norm(0,0) = fmin(fmax((u_control(0,0)+ff_thrust)/16.0f, 0.0f), 1.0f);
    u_control_norm(1,0) = fmin(fmax((u_control(1,0))/(0.1080f*4.0f), -1.0f), 1.0f) * 0.4f;  
    u_control_norm(2,0) = fmin(fmax((u_control(2,0))/(0.1080f*4.0f),  -1.0f), 1.0f) * 0.4f;
    u_control_norm(3,0) = fmin(fmax((u_control(3,0))/(0.1f*1.0f), -1.0f), 1.0f)* 0.4f;
    u_control_norm(0,0) = fmin(fmax((u_control(0,0)+ff_thrust)/16.0f, 0.0f), 1.0f);

   // not normalized control inputs
     u_control(0,0) = u_control_norm(0,0)*16.0f;
     u_control(1,0) = u_control_norm(1,0)*4.0f;
     u_control(2,0) = u_control_norm(2,0)*4.0f;
     u_control(3,0) = u_control_norm(3,0)*0.05f;
    //"\t" <<  u_control(0,0)+ff_thrust << "\n";
         /* Save data*/
    // writeStateOnFile("~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/state.txt", _current_state, now);
    // writeInputOnFile("~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/control_input.txt", u_control_norm, now); 
    // writeLyapunovOnFile("~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/lyapunov.txt", _lyap_fun(0,0), now); 
    // writeStateOnFile("~/px4-19/Firmware/src/modules/mc_att_control/lqr_files/ekf.txt", _current_state_ekf, now);
    writeStateOnFile("state.txt", _current_state, now);
    writeStateOnFile("state_est.txt", state_est, now);
    writeMagReadingsOnFile("mag_data.txt", now);
    //writeInputOnFile("control_input.txt", u_control_norm, now); 
    //writeLyapunovOnFile("lyapunov.txt", _lyap_fun(0,0), now); 
    //writeStateOnFile("ekf.txt", _current_state_ekf, now);

    return u_control_norm;    

}

Matrix<float,4,1> QuadrotorLQRControl::normalizationControlInputs(Matrix<float,4,1> _u)
{
   Matrix<float,4,1> _u_norm;
   _u_norm(0,0) = _u(0,0)*16.0f;
   _u_norm(1,0) = _u(1,0)*(0.1080f*4.0f);
   _u_norm(2,0) = _u(2,0)*(0.1080f*4.0f);
   _u_norm(3,0) = _u(3,0)*(0.1f*1.0f); 

return _u_norm;
}

Matrix<float,4,1> QuadrotorLQRControl::getLQRcontrols()
{

return u_control;

}

void QuadrotorLQRControl::setCurrentState(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

      _current_state(0,0) = _v_local_pos.vx;
      _current_state(1,0) = _v_local_pos.x;
      _current_state(2,0) = _v_local_pos.vy;
      _current_state(3,0) = _v_local_pos.y;
      _current_state(4,0) = _v_local_pos.vz;
      _current_state(5,0) = _v_local_pos.z;
    
      _current_state(6,0)  = _v_att.rollspeed;
      _current_state(7,0)  = Eulerf(Quatf(_v_att.q)).phi();
      _current_state(8,0)  = _v_att.pitchspeed;
      _current_state(9,0)  = Eulerf(Quatf(_v_att.q)).theta();
      _current_state(10,0) = _v_att.yawspeed;	
      _current_state(11,0) = Eulerf(Quatf(_v_att.q)).psi();

}

void QuadrotorLQRControl::setCurrentStateEkf(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

      _current_state_ekf(0,0) = _v_local_pos.vx;
      _current_state_ekf(1,0) = _v_local_pos.x;
      _current_state_ekf(2,0) = _v_local_pos.vy;
      _current_state_ekf(3,0) = _v_local_pos.y;
      _current_state_ekf(4,0) = _v_local_pos.vz;
      _current_state_ekf(5,0) = _v_local_pos.z;
    
      _current_state_ekf(6,0)  = _v_att.rollspeed;
      _current_state_ekf(7,0)  = Eulerf(Quatf(_v_att.q)).phi();
      _current_state_ekf(8,0)  = _v_att.pitchspeed;
      _current_state_ekf(9,0)  = Eulerf(Quatf(_v_att.q)).theta();
      _current_state_ekf(10,0) = _v_att.yawspeed;	
      _current_state_ekf(11,0) = Eulerf(Quatf(_v_att.q)).psi();

}

void QuadrotorLQRControl::setSensorData(struct sensor_mag_s _sensor_mag, struct sensor_combined_s _sensor_combined)
{
      const float us_to_s = 0.000001;
      const float mag_lpf_weight = 1.0f;

      _mag_readings_hist(0,0) = _mag_readings(0,0);
      _mag_readings_hist(1,0) = _mag_readings(1,0);
      _mag_readings_hist(2,0) = _mag_readings(2,0);      

      _mag_readings(0,0) = mag_lpf_weight*_sensor_mag.x + (1.0f-mag_lpf_weight)*_mag_readings_hist(0,0);
      _mag_readings(1,0) = mag_lpf_weight*_sensor_mag.y + (1.0f-mag_lpf_weight)*_mag_readings_hist(0,0);
      _mag_readings(2,0) = mag_lpf_weight*_sensor_mag.z + (1.0f-mag_lpf_weight)*_mag_readings_hist(0,0);

      _gyro_readings(0,0) = _sensor_combined.gyro_rad[0];
      _gyro_readings(1,0) = _sensor_combined.gyro_rad[1];
      _gyro_readings(2,0) = _sensor_combined.gyro_rad[2];
      _gyro_readings(3,0) = (float)_sensor_combined.gyro_integral_dt * us_to_s;

      _acc_readings(0,0) = _sensor_combined.accelerometer_m_s2[0];
      _acc_readings(1,0) = _sensor_combined.accelerometer_m_s2[1];
      _acc_readings(2,0) = _sensor_combined.accelerometer_m_s2[2];
      _acc_readings(3,0) = (float)_sensor_combined.accelerometer_integral_dt * us_to_s;
           
}

void QuadrotorLQRControl::setAutoEqPoint(struct vehicle_attitude_s _v_att, struct vehicle_local_position_s  _v_local_pos)
{

      _eq_point(0,0) = _v_local_pos.vx;
      _eq_point(1,0) = _v_local_pos.x;
      _eq_point(2,0) = _v_local_pos.vy;
      _eq_point(3,0) = _v_local_pos.y;
      _eq_point(4,0) = _v_local_pos.vz;
      _eq_point(5,0) = _v_local_pos.z;
    
      _eq_point(6,0)  = _v_att.rollspeed;
      _eq_point(7,0)  = Eulerf(Quatf(_v_att.q)).phi();
      _eq_point(8,0)  = _v_att.pitchspeed;
      _eq_point(9,0)  = Eulerf(Quatf(_v_att.q)).theta();
      _eq_point(10,0) = _v_att.yawspeed;	
      _eq_point(11,0) = Eulerf(Quatf(_v_att.q)).psi();


}

void QuadrotorLQRControl::setEquilibriumPoint(Matrix<float,12,1> eqPoint)
{

      _eq_point(0,0) = eqPoint(0,0);
      _eq_point(1,0) = eqPoint(1,0);
      _eq_point(2,0) = eqPoint(2,0);
      _eq_point(3,0) = eqPoint(3,0);
      _eq_point(4,0) = eqPoint(4,0);
      _eq_point(5,0) = eqPoint(5,0);
    
      _eq_point(6,0)  = eqPoint(6,0);
      _eq_point(7,0)  = eqPoint(7,0);
      _eq_point(8,0)  = eqPoint(8,0);
      _eq_point(9,0)  = eqPoint(9,0);
      _eq_point(10,0) = eqPoint(10,0);	
      _eq_point(11,0) = eqPoint(11,0);
       
}

void QuadrotorLQRControl::setAutoEqPointFlag(bool flag)
{

   _auto_eq_point_flag = flag;
}

bool QuadrotorLQRControl::getAutoEqPointFlag()
{

   return _auto_eq_point_flag;
}

/* Save data on files */

void QuadrotorLQRControl::writeStateOnFile(const char *filename, Matrix <float, 12, 1> vect, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);
        
        outfile << t << "\t";   // time
       
	for(int i=0;i<12;i++){
		if(i==11){
			outfile << vect(i,0) << "\n";
		}else{
	         outfile << vect(i,0) << "\t";
		}
	}
	outfile.close();
	return;
}

void QuadrotorLQRControl::writeMagReadingsOnFile(const char *filename, hrt_abstime t) {

  ofstream outfile;
  outfile.open(filename, std::ios::out | std::ios::app);
        
  outfile << t << "\t";   // time
  outfile << _mag_readings(1,0) << "\t";
  outfile << _mag_readings(2,0) << "\t";
  outfile << _mag_readings(3,0) << "\n";
  outfile.close();
  return;
}


void QuadrotorLQRControl::writeInputOnFile(const char *filename, Matrix <float, 4, 1> vect, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);
        
        outfile << t << "\t";   // time
        
	for(int i=0;i<4;i++){
		if(i==3){
			outfile << vect(i,0) << "\n";
		}else{
	         outfile << vect(i,0) << "\t";
		}
	}
	outfile.close();
	return;
}

void QuadrotorLQRControl::writeLyapunovOnFile(const char *filename, float value, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);
        
        outfile << t << "\t" << value << "\n";   
	outfile.close();
	return;
}

Matrix <float, 12, 12>  QuadrotorLQRControl::readMatrixP(const char *filename)
    {

    static Matrix <float, 12, 12> result;
    static int rows = 12;
    static int cols = 12;
    ifstream infile;
    PX4_INFO("trying to read P at %s", filename);
    infile.open(filename);
    if (infile.is_open()){
      PX4_INFO("P file OPENED!");
         for (int i=0; i<rows;i++){
    		string line;
    		getline(infile, line);
    		stringstream stream(line);
    		for (int j=0; j<cols; j++){
    			stream >> result(i,j);
    		}

    	}
    	infile.close();
    }else cout << "Unable to open file\r\n";
    return result;

 }
