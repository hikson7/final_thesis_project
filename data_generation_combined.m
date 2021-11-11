function data_generation_combined

clc
clear
clf

% radius of shell. Parameter used for dynamics and kinematics
global R;

tspan = [0, 10];

% initial conditions
q1_dr_0 = 0;
q2_dr_0 = 0;
qd1_dr_0 = 0;
qd2_dr_0 = 0;
q1_st_0 = 0;
q2_st_0 = 0;
qd1_st_0 = 0;
qd2_st_0 = 0;

state = [q1_dr_0; q2_dr_0; qd1_dr_0; qd2_dr_0; q1_st_0; q2_st_0; qd1_st_0; qd2_st_0; ];

% options = odeset("RelTol", 1e-5);
options = odeset("RelTol", 1e-8);
% [t, state_out] = ode45(@open_model_combined, tspan, state, options);
[t, state_out] = ode45(@closed_model_combined, tspan, state, options);

% retrieve necessary outputs
theta_d = state_out(:, 3);  % shell turning velocity
phi = state_out(:, 5);
phi_d = state_out(:, 7);

[x, y, ] = calculate_pose(theta_d, phi, phi_d, R, t);

figure(1);
plot(x, y, 'b-');
title("Spherical robot motion in 2D plane");
xlabel("x displacement [m]")
ylabel("y displacement [m]")
grid on;
grid minor;
% range = 1000;
% axis([-range/2 range/2 -range/2 range/2])

figure(2);
subplot(2, 1, 1);

plot(t, phi, 'b-');
grid on;
grid minor;
title("Steering input as angular displacement")
xlabel("time [s]")
ylabel("angular displacement [rad]")

% figure(3);
subplot(2, 1, 2);
plot(t, phi_d, 'b-');
grid on;
grid minor;
title("Steering input as angular velocity");
xlabel("time [t]")
ylabel("anguar velocity [rad/s]")

end

function [x, y, varphi] = calculate_pose(theta_d, phi, phi_d, R, t)
    
    tlen = size(t, 1);
    % heading of robot
    varphi_d = theta_d.*sin(phi);
    varphi = zeros(tlen, 1);
    
    x = zeros(tlen, 1);
    y = zeros(tlen, 1);
    
    for i=1:tlen
        if i+1 == tlen+1
            break;
        end
        varphi(i+1) = varphi(i) + varphi_d(i).*t(i);

        xdot = R*(theta_d(i).*cos(phi(i)).*cos(varphi(i))+phi_d(i).*sin(varphi(i)));
        ydot = R*(theta_d(i).*cos(phi(i)).*sin(varphi(i))-phi_d(i).*cos(varphi(i)));

        x(i+1) = x(i) + xdot*t(i);
        y(i+1) = y(i) + ydot*t(i);
    end
end