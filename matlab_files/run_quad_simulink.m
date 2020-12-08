clear
clc

rng(2);
run('quadrocopter_LQR.m')
ctrlr_poles = eig(A-B*K);
obsv_poles = 4*ctrlr_poles;
L = place(A', eye(12), obsv_poles);
save('sys_matrices.mat', 'A','B','L', 'K');
%obsv_poles = [-45, -46, -47, -48, -1, -2, -3, -4, -5, -6, -7, -8];
x0 = zeros(12,1);
% run2 values (run 1 was 15, 15 15)
px_init = -5; py_init = -5; pz_init = -5;
x0(2) = px_init;
x0(4) = py_init;
x0(6) = pz_init;

xhat0 = x0;
xext0 = [x0;xhat0];

sensor_noise_variance = .25*ones(12,1);

T_final = 10;

sim_out = sim('quadrotor_linear_obsv_model',(0:0.01:T_final));
x = sim_out.yout{1}.Values.Data;
t = sim_out.yout{1}.Values.Time;
xhat = sim_out.yout{2}.Values.Data;
plot_pos = true;
plot_velocity = true;
plot_angle = true;
plot_angvel = true;
plot_input = false;
legend_labels = ["true state"; "estimated state"];
plot_quadrotor_model(x,t, [], xhat, t, [], plot_pos, plot_velocity, plot_angle, plot_angvel, plot_input, legend_labels)
