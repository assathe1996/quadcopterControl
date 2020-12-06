clear
clc
close all

x_mag_baseline = .21506;
y_mag_baseline = .01021;
z_mag_baseline = .42974;
incl_baseline = 63.41469;
decl_baseline = 2.71808;
H_magn_baseline = .21530;
T_magn_baseline = 2.71808;

%state_data_file = "/Users/tmcnama2/PX4/PX4-Autopilot/build/px4_sitl_default/tmp/rootfs/state.txt";
state_data_file = "~/px4-19/Firmware/build/px4_sitl_default/tmp/rootfs/state.txt";
state_data = readmatrix(state_data_file);
t0 = state_data(1,1);
t = (state_data(:,1) - t0)/1e6;
x = state_data(:,2:13);
% ekf_t = state_data(:,1) - t0;
% ekf_x = state_data(:,3);
% ekf_y = state_data(:,5);
% ekf_z = state_data(:,7);
% ekf_th = state_data(:,9);
% ekf_phi = state_data(:,11);
% ekf_psi = state_data(:,13);

est_state_data_file = "~/px4-19/Firmware/build/px4_sitl_default/tmp/rootfs/state_est.txt";
est_state_data = readmatrix(est_state_data_file);
t0 = est_state_data(1,1);
t_est = (est_state_data(:,1) - t0)/1e6;
x_est = est_state_data(:,2:13);
% est_t = est_state_data(:,1) - t0;
% est_x = est_state_data(:,3);
% est_y = est_state_data(:,5);
% est_z = est_state_data(:,7);
% est_th = est_state_data(:,9);
% est_phi = est_state_data(:,11);
% est_psi = est_state_data(:,13);
plot_quadrotor_model(x_est,t_est, [], x, t, [], true, false, true, true, false)

mag_data_file = "~/px4-19/Firmware/build/px4_sitl_default/tmp/rootfs/mag_data.txt";
mag_data = readmatrix(mag_data_file);
mag_t = (mag_data(:,1) - mag_data(1,1))/1e6;
mag_x = mag_data(:,2) - x_mag_baseline;
mag_y = mag_data(:,3) - y_mag_baseline;
mag_z = mag_data(:,4) - z_mag_baseline;

figure()
plot(mag_t, mag_x, mag_t, mag_y, mag_t, mag_z);
xlabel('time (s)')
legend('x','y','z')
grid on
