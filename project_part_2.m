clear
clc

% Playing around with designing what a linear observer would look like if
% we had estimates for all of the states

run('quadrocopter_LQR.m')

C = eye(12);
ctrlr_poles = eig(A-B*K);

% From https://ecal.berkeley.edu/files/ce295/CH02-StateEstimation.pdf
% ". A general rule-of-thumb is that the observer eigenvalues should be 
% placed 2-10 times faster than the slowest stable eigenvalue of the energy
% system itself."
slowest_ctrlr_pole = max(real(ctrlr_poles));
obsv_poles = 6*ctrlr_poles;
L = place((A-B*K)',C', obsv_poles)';
L_file = "/Users/tmcnama2/px4-19/Firmware/build/px4_sitl_default/tmp/rootfs/tgmL.txt";
writematrix(L,L_file,'Delimiter','tab')

