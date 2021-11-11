function state_out = decoupled_combined_model(t, cur_state)

global R;

m1 = 0.05; % mass of shell
m2 = 0.1; % mass of internal
m3 = 1; % mass of counter weight
l = 0.03; % length of connection rod
R = 0.08;  % radius of shell
g = 9.80665;
j1 = 0.1;

% q1_dr = cur_state(1); % unused
q2_dr = cur_state(2);
qd1_dr = cur_state(3);
qd2_dr = cur_state(4);

% q1_st = cur_state(5); % unused
q2_st = cur_state(6);
qd1_st = cur_state(7);
qd2_st = cur_state(8);

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

tau_dr = [const_drive_torque; const_drive_torque];
qdd_dr = M_dr^(-1)*(B_dr.*tau_dr-C_dr);

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