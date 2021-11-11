function state_out = closed_model_combined(t, cur_state, trajhandle, trajdhandle)

global R;

m1 = 0.05; % mass of shell
m2 = 0.1; % mass of internal
m3 = 1; % mass of counter weight
l = 0.03; % length of connection rod
R = 0.08;  % radius of shell
g = 9.80665;
j1 = 0.1;

% q1_dr = cur_state(1); % spinning angle of shell (unused)      (theta)
q2_dr = cur_state(2);   % swinging angle of the counterweight 
qd1_dr = cur_state(3);  % angular velocity of (1)               (theta_d)
qd2_dr = cur_state(4);  % angular velocity of (2)             

% q1_st = cur_state(5); % spinning angle of shell (unused)      (phi)
q2_st = cur_state(6);   % swinging angle of the counterweight 
qd1_st = cur_state(7);  % angular velocity of (5)               (phi_d)
qd2_st = cur_state(8);  % angular velocity of (6)             

[M_dr, C_dr, B_dr] = create_matrices(q2_dr, qd2_dr, m1, m2, m3, R, l, j1, g);
[M_st, C_st, B_st] = create_matrices(q2_st, qd2_st, m1, m2, m3, R, l, j1, g);

% controls

% temporary open-loop
const_drive_torque = 0.0005;
const_steer_torque = 0;

if t > 5 && t < 5.5
    const_steer_torque = -0.01;
elseif t >= 5.5 && t < 6
    const_steer_torque = 0.01;
end

% global cur_pose cur_varphi;
[theta_d_des, phi_des, phi_d_des] = pursuit_kinematic_model(t, trajhandle, trajdhandle);

state_out(9, 1) = theta_d_des;
state_out(10, 1) = phi_des;
state_out(11, 1) = phi_d_des;

% global arr_theta_d_des arr_phi_des arr_phi_d_des;
% arr_theta_d_des = cat(1, arr_theta_d_des, theta_d_des);
% arr_phi_d_des = cat(1, arr_phi_d_des, phi_d_des);
% arr_phi_des = cat(1, arr_phi_des, phi_des);

% use theta_d_des
tau_dr = [const_drive_torque; const_drive_torque];
qdd_dr = M_dr^(-1)*(B_dr.*tau_dr-C_dr);

% use phi_des, phi_d_des
tau_st = [const_steer_torque; const_steer_torque];
qdd_st = M_st^(-1)*(B_st.*tau_st-C_st);

% resulting state

state_out(1, 1) = qd1_dr;
state_out(2, 1) = qd2_dr;
state_out(3, 1) = qdd_dr(1);
state_out(4, 1) = qdd_dr(2);

state_out(5, 1) = qd1_st;
state_out(6, 1) = qd2_st;
state_out(7, 1) = qdd_st(1);
state_out(8, 1) = qdd_st(2);

global t_prev;
dt = t-t_prev;
update_pose(qdd_dr(1), qd1_st, qdd_st(1), R, dt);

t_prev = t;

end

function [M, C, B] = create_matrices(q2, qd2, m1, m2, m3, R, l, j1, g)
    M(1, 1) = m1*5/3+m2+m3;
    M(1, 2) = m3*l*cos(q2);
    M(2, 1) = m3*l*cos(q2);
    M(2, 2) = j1+m3*l^2;

    C(1, 1) = -m3*l*sin(q2)*qd2^2;
    C(2, 1) = m3*g*l*sin(q2);

    B(1, 1) = 1/R;
    B(2, 1) = 1;
end


function update_pose(theta_d, phi, phi_d, R, dt)
    global cur_pose cur_varphi;

    varphi_d = theta_d*sin(phi);
    varphi = cur_varphi + varphi_d*dt;

    xdot = R*(theta_d*cos(phi)*cos(varphi)+phi_d*sin(varphi));
    ydot = R*(theta_d*cos(phi)*sin(varphi)-phi_d*cos(varphi));

    x = cur_pose(1) + xdot*dt;
    y = cur_pose(2) + ydot*dt;

    cur_pose(1) = x;
    cur_pose(2) = y;
    cur_varphi = varphi;
end