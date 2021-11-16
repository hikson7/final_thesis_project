function data_generation_combined

clc
clear
clf

% radius of shell. Parameter used for dynamics and kinematics
global R;
 
t_top = 10;
tspan = [0, t_top];

% initial conditions
q1_dr_0 = 0;
q2_dr_0 = 0;
qd1_dr_0 = 0;
qd2_dr_0 = 0;
q1_st_0 = 0;
q2_st_0 = 0;
qd1_st_0 = 0;
qd2_st_0 = 0;

% state = [q1_dr_0; q2_dr_0; qd1_dr_0; qd2_dr_0; q1_st_0; q2_st_0; qd1_st_0; qd2_st_0; 0; 0; 0];
state = [q1_dr_0; q2_dr_0; qd1_dr_0; qd2_dr_0; q1_st_0; q2_st_0; qd1_st_0; qd2_st_0];
% options = odeset("RelTol", 1e-5);
options = odeset("RelTol", 1e-8);
[t, state_out] = ode45(@open_model_combined, tspan, state, options);

trajhandle = @trajsin;
trajdhandle = @trajdsin;

global epi_prev epi_sum phi_des_prev;
epi_prev = 0;
epi_sum = 0;
phi_des_prev = 0;


% init_pose = [0; 0; pi/4];
init_pose = [0; 0; 0];
global cur_pose cur_varphi;
cur_pose(1) = init_pose(1, 1);
cur_pose(2) = init_pose(2, 1);
cur_varphi = init_pose(3, 1);

global t_prev;
t_prev = 0;

global theta_d_err_total theta_d_err_prev steer_err_total steer_err_prev;
theta_d_err_total = 0;
theta_d_err_prev = 0;
steer_err_total = 0;
steer_err_prev = 0;
% [t, state_out] = ode45(@closed_model_combined, tspan, state, trajhandle, trajdhandle, options);
% [t, state_out] = ode15s(@closed_model_kine, tspan, state, trajhandle, trajdhandle, options);
% ode15s 
% retrieve necessary outputs
theta_d = state_out(:, 3);  % shell turning velocity
phi = state_out(:, 5);
phi_d = state_out(:, 7);

% theta_d_des = state_out(:, 9);
% phi_des = state_out(:, 10);
% phi_d_des = state_out(:, 11);

[x, y, ] = calculate_pose(init_pose, theta_d, phi, phi_d, R, t);

figure(1);
% 
% [x_d, y_d] = trajsin(t);
% plot(x_d, y_d, 'b-');

% hold on;
plot(x, y, '-b');
% hold off;

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

subplot(2, 1, 2);
plot(t, phi_d, 'b-');
grid on;
grid minor;
title("Steering input as angular velocity");
xlabel("time [t]")
ylabel("anguar velocity [rad/s]")
% 
% figure(3);
% subplot(3, 1, 1);
% 
% plot(t, theta_d-theta_d_des, 'b-');
% grid on;
% grid minor;
% title("Theta_d error")
% xlabel("time [s]")
% ylabel("angular velocity [rad/s]")
% 
% subplot(3, 1, 2);
% plot(t, phi-phi_des, 'b-');
% grid on;
% grid minor;
% title("Phi error");
% xlabel("time [t]")
% ylabel("anguar displacement [rad]")
% 
% subplot(3, 1, 3);
% plot(t, phi_d-phi_d_des, 'b-');
% grid on;
% grid minor;
% title("Phi_d error");
% xlabel("time [t]")
% ylabel("anguar velocity [rad/s]")


% figure(4);
% subplot(2, 1, 1);

% plot(t, theta_d-theta_d_des, 'b-');
% grid on;
% grid minor;
% title("x error")
% xlabel("time [s]")
% ylabel("angular velocity [rad/s]")
% 
% subplot(2, 1, 2);
% plot(t, phi-phi_des, 'b-');
% grid on;
% grid minor;
% title("Phi error");
% xlabel("time [t]")
% ylabel("anguar displacement [rad]")

end

function [x, y, varphi] = calculate_pose(init_pose, theta_d, phi, phi_d, R, t)
    tlen = size(t, 1);
    % heading of robot
    varphi_d = theta_d.*sin(phi);
    varphi = zeros(tlen, 1);
    
    x = zeros(tlen, 1);
    y = zeros(tlen, 1);
    
    x(1, 1) = init_pose(1, 1);
    y(1, 1) = init_pose(2, 1);
    varphi(1, 1) = init_pose(3, 1);
    
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