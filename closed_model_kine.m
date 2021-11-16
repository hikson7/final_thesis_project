function state_out = closed_model_combined(t, cur_state, trajhandle, trajdhandle)

% q1_dr = cur_state(1); % spinning angle of shell (unused)      (theta)
% q2_dr = cur_state(2);   % swinging angle of the counterweight 
qd1_dr = cur_state(3);  % angular velocity of (1)               (theta_d)
% qd2_dr = cur_state(4);  % angular velocity of (2)             

q1_st = cur_state(5); % spinning angle of shell (unused)      (phi)
% q2_st = cur_state(6);   % swinging angle of the counterweight 
qd1_st = cur_state(7);  % angular velocity of (5)               (phi_d)
% qd2_st = cur_state(8);  % angular velocity of (6)  

[theta_d_des, phi_des, phi_d_des] = pursuit_kinematic_model(t, trajhandle, trajdhandle);


global R;
global t_prev;
dt = t-t_prev;
t = t_prev;

update_pose(theta_d_des, phi_des, phi_d_des, R, dt);

state_out(3, 1) = theta_d_des;
state_out(5, 1) = phi_des;
state_out(7, 1) = phi_d_des;

state_out(9, 1) = 0;
state_out(10, 1) = 0;
state_out(11, 1) = 0;
end