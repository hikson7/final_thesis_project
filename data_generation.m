function data_generation

clc
clear
figure(1);
clf

tspan = [0, 10];

q1_0 = 0;
q2_0 = 0;
qd1_0 = 0;
qd2_0 = 0;

state = [q1_0; q2_0; qd1_0; qd2_0];

options = odeset("RelTol", 1e-5);

[t, state_out] = ode45(@model, tspan, state, options);

theta = state_out(:, 1);   % shell turn angle
% Q2 = F(:, 2);
theta_d = state_out(:, 3);  % shell turning vel
% Qd2 = F(:, 4);

% steer angles
phi = zeros(size(t, 1), 1);
phi_d = zeros(size(t, 1), 1);

R = 1;

% p = R*cos(theta)/sin(theta); % turning cone radius
% varphi = phi*sin(theta)*p; % heading
varphi = phi;

xdot = R*(theta_d.*cos(phi).*cos(varphi)+phi_d.*sin(varphi));
ydot = R*(theta_d.*cos(phi).*sin(varphi)-phi_d.*cos(varphi));

plot(xdot.*t, ydot.*t, 'b-');
t
end