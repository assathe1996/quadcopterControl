clear
clc

rng(2);
run('quadrocopter_LQR.m')

x0 = zeros(12,1);
% run2 values (run 1 was 15, 15 15)
px_init = -15; py_init = -15; pz_init = -15;
x0(2) = px_init;
x0(4) = py_init;
x0(6) = pz_init;

T_final = 10;

sim_out = sim('quadrotor_model',(0:0.01:T_final));
x = sim_out.yout{1}.Values.Data;
t = sim_out.yout{1}.Values.Time;
plot_pos = true;
plot_velocity = true;
plot_angle = true;
plot_angvel = true;
plot_input = false;
legend_labels = ["true state"; "estimated state"];
plot_quadrotor_model(x,t, [], x, t, [], plot_pos, plot_velocity, plot_angle, plot_angvel, plot_input, legend_labels)
