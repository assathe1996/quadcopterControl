% Tim McNamara
% 18-776 Project Part 1

function dx_ext_dt = quadrotor_exact(t, x_ext, K, L, sensor_noise, A, B)
    m = 0.6;
    Jx = 0.0092;
    Jy = 0.0091;
    Jz = 0.0101;
    g = 9.81;

    x = x_ext(1:12);
    px_dot = x(1);
%   px =     x(2);
    py_dot = x(3);
%   py =     x(4);
    pz_dot = x(5);
%   pz =     x(6);
    p =      x(7);
    phi =    x(8);
    q =      x(9); 
    theta =  x(10);
    r =      x(11);
    psi =    x(12);
    
    x_hat = x_ext(13:24);
    px_dot_hat = x_hat(1);
%   px =     x(2);
    py_dot_hat = x_hat(3);
%   py =     x(4);
    pz_dot_hat = x_hat(5);
%   pz =     x(6);
    p_hat =      x_hat(7);
    phi_hat =    x_hat(8);
    q_hat =      x_hat(9); 
    theta_hat =  x_hat(10);
    r_hat =      x_hat(11);
    psi_hat =    x_hat(12);
    
    y = zeros(12,1);
    for i = 1:12
        y(i) = x(i) + normrnd(0,sensor_noise(i));
    end
    
    u = -K*x_hat;
    u(1) = u(1) + m*g;
    
    u_clipped = zeros(4,1);
    u_clipped(1) = max([min([u(1), 16]),0]);
    u_clipped(2) = max([min([u(2), .432]),-.432]);
    u_clipped(3) = max([min([u(3), .432]),-.432]);
    u_clipped(4) = max([min([u(4), .1]),-.1]);
    
%     F = u(1) + m*g;
%     tau_phi = u(2);
%     tau_theta = u(3);
%     tau_psi = u(4);

    F = u_clipped(1);
    tau_phi = u_clipped(2);
    tau_theta = u_clipped(3);
    tau_psi = u_clipped(4);
    
    az = -F/m;

    dx_dt =  [cos(phi)*sin(theta)*az;
             px_dot;
             -sin(phi)*az;
             py_dot;
             g + cos(phi)*cos(theta)*az;
             pz_dot;             
             1/Jx*tau_phi;
             p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
             1/Jy*tau_theta;
             q*cos(phi) - r*sin(phi);
             1/Jz*tau_psi;
             q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)];

    %dx_hat_dt = A*x_hat + B*u_clipped + L*(y-x_hat);
    dx_hat_dt = [cos(phi_hat)*sin(theta_hat)*az;
                 px_dot_hat;
                 -sin(phi_hat)*az;
                 py_dot_hat;
                 g + cos(phi_hat)*cos(theta_hat)*az;
                 pz_dot_hat;
                 1/Jx*tau_phi;
                 p_hat + q*sin(phi_hat)*tan(theta_hat) + r_hat*cos(phi_hat)*tan(theta_hat);
                 1/Jy*tau_theta;
                 q*cos(phi_hat) - r*sin(phi_hat);
                 1/Jz*tau_psi;
                 q*sin(phi_hat)/cos(theta_hat) + r*cos(phi_hat)/cos(theta_hat)];
    dx_hat_dt = dx_hat_dt + L*(y-x_hat);        
    dx_ext_dt = [dx_dt; dx_hat_dt];
end