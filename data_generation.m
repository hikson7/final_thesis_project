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

% theta = state_out(:, 1);   % shell turn angle
% Q2 = F(:, 2);
theta_d = state_out(:, 3);  % shell turning vel
% Qd2 = F(:, 4);

% steer angles
phi = zeros(size(t, 1), 1);
phi_d = zeros(size(t, 1), 1);

timesteps = size(t, 1);
time_mid = round(timesteps/2);
step_dur = 5;

steer_t = t(time_mid:time_mid+step_dur);

% steering impulse input
steer_vel = 0.01; % [rad/s]
phi_d(time_mid:time_mid+step_dur) = steer_vel;

% for i=time_mid:time_mid+step_dur
%     if i+1 == time_mid+step_dur
%         break;
%     end
%     phi(i+1) = phi(i) + phi_d(i).*t(i);
% end
phi(time_mid:time_mid+step_dur) = phi_d(time_mid:time_mid+step_dur).*steer_t;


R = 1;

% heading of robot

varphi_d = theta_d.*sin(phi);
varphi = zeros(size(t, 1), 1);
x = zeros(size(t, 1), 1);
y = zeros(size(t, 1), 1);
% xdot = zeros(size(t, 1), 1);
% ydot = zeros(size(t, 1), 1);

% for i=1:size(t, 1)
%     if i+1 == size(t, 1)+1
%         break;
%     end
%     varphi(i+1) = varphi(i) + varphi_d(i).*t(i);
% end

for i=1:size(t, 1)
    if i+1 == size(t, 1)+1
        break;
    end
    varphi(i+1) = varphi(i) + varphi_d(i).*t(i);

    xdot = R*(theta_d(i).*cos(phi(i)).*cos(varphi(i))+phi_d(i).*sin(varphi(i)));
    ydot = R*(theta_d(i).*cos(phi(i)).*sin(varphi(i))-phi_d(i).*cos(varphi(i)));

    x(i+1) = x(i) + xdot*t(i);
    y(i+1) = y(i) + ydot*t(i);
end

% xdot = R*(theta_d.*cos(phi).*cos(varphi)+phi_d.*sin(varphi));
% ydot = R*(theta_d.*cos(phi).*sin(varphi)-phi_d.*cos(varphi));

plot(x, y, 'b-');
grid on;
grid minor;
range = 1000;

% axis([-range/2 range/2 -range/2 range/2])

figure(2);

plot(t, phi, 'b-');

figure(3);

plot(t, phi_d, 'b-');
end