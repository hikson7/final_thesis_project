function [state_out] = model(t, state)

m1 = 1; % mass of shell
m2 = 1; % mass of internal
m3 = 1; % mass of counter weight
l = 0.3; % length of connection rod
R = 1;  % radius of shell
g = 9.80665;
j1 = 1;

% q1 = shell turn angle
% q2 = swing angle
% q1 = state(1);
q2 = state(2);
qd1 = state(3);
qd2 = state(4);

M(1, 1) = m1*5/3+m2+m3;
M(1, 2) = m3*l*cos(q2);
M(2, 1) = m3*l*cos(q2);
M(2, 2) = j1+m3*l^2;

C(1, 1) = -m3*l*sin(q2)*qd2^2;
C(2, 1) = m3*g*l*sin(q2);

B(1, 1) = 1/R;
B(2, 1) = 1;

tau = [1; 1]; % always same output for open loop?
% tau = [0; 0];

state_out(1, 1) = qd1;
state_out(2, 1) = qd2;

qdd = M^(-1)*(B.*tau-C);

state_out(3, 1) = qdd(1);
state_out(4, 1) = qdd(2);
end