function data_generation

clc
clear
figure(1);
clf

% radius of shell. Parameter used for dynamics and kinematics
global R;

tspan = [0, 10];

% initial conditions
q1_0 = 0;
q2_0 = 0;
qd1_0 = 0;
qd2_0 = 0;

state = [q1_0; q2_0; qd1_0; qd2_0];

% options = odeset("RelTol", 1e-5);
options = odeset("RelTol", 1e-8);
[t1, state_out] = ode45(@model_drive, tspan, state, options);

% retrieve necessary outputs
theta_d = state_out(:, 3);  % shell turning velocity

% Insert artificial steering

% steer angles
% phi = zeros(size(t, 1), 1);
% phi_d = zeros(size(t, 1), 1);

%%% here find decoupled input.
[t2, state_out] = ode45(@model_steer, tspan, state, options);
% steer angle and velocity
phi = state_out(:, 1);
phi_d = state_out(:, 3);
% phi = 
% phi_d = 

duration = 1;
% steer_velocity = -0.01;
% 
% for i=2:8
% insert_at = i;      % [s]
% [phi, phi_d] = insert_steer_vel_input(t, insert_at, duration, steer_velocity, phi, phi_d);
% end



[x, y, ] = calculate_pose(theta_d, phi, phi_d, R, t1, t2);

plot(x, y, 'b-');
title("Spherical robot motion in 2D plane, input duration = "+duration+" [s]");
xlabel("x displacement [m]")
ylabel("y displacement [m]")
grid on;
grid minor;
% range = 1000;
% axis([-range/2 range/2 -range/2 range/2])

figure(2);
subplot(2, 1, 1);

plot(t2, phi, 'b-');
grid on;
grid minor;
title("Steering input as angular displacement")
xlabel("time [s]")
ylabel("angular displacement [rad]")

% figure(3);
subplot(2, 1, 2);
plot(t2, phi_d, 'b-');
grid on;
grid minor;
title("Steering input as angular velocity");
xlabel("time [t]")
ylabel("anguar velocity [rad/s]")

end

function [x, y, varphi] = calculate_pose(theta_d_ori, phi_ori, phi_d_ori, R, t1, t2)
    
    % make them the same length
    inc_max = max(size(t1, 1), size(t2, 1))
    
    theta_d = zeros(inc_max, 1);
    phi = zeros(inc_max, 1);
    phi_d = zeros(inc_max, 1);
    
    j = 1;
    % if t1 has finer increments than t2
    if size(t1, 1) < size(t2, 1)
        t = t2;
        phi = phi_ori;
        phi_d = phi_d_ori;
    else
        t = t1;
        theta_d = theta_d_ori;
    end
    
    for i=1:inc_max
        if size(t1, 1) < size(t2, 1) 
            theta_d(i) = theta_d_ori(j);
            if t1(j) < t2(i)
                j = j + 1;
            end
        else
            phi(i) = phi_ori(j);
            phi_d(i) = phi_d_ori(j);
            if t2(j) < t1(i)
                j = j + 1;
            end
        end
    end
% 
%     % add fillers for t2
%     if size(t1, 1) > size(t2, 1)
%         t = t1;
%         theta_d = theta_d_ori;
%         for i=1:inc_max
%             phi(i) = phi_ori(j);
%             phi_d(i) = phi_d_ori(j);
%             
%             if t1(i) > t2(j)
%                 j = j + 1;
%             end
%         end
%     end
    
    % heading of robot
    varphi_d = theta_d.*sin(phi);
    varphi = zeros(inc_max, 1);
    
    x = zeros(inc_max, 1);
    y = zeros(inc_max, 1);
    
    for i=1:inc_max
        if i+1 == inc_max+1
            break;
        end
        varphi(i+1) = varphi(i) + varphi_d(i).*t(i);

        xdot = R*(theta_d(i).*cos(phi(i)).*cos(varphi(i))+phi_d(i).*sin(varphi(i)));
        ydot = R*(theta_d(i).*cos(phi(i)).*sin(varphi(i))-phi_d(i).*cos(varphi(i)));

        x(i+1) = x(i) + xdot*t(i);
        y(i+1) = y(i) + ydot*t(i);
    end
end

% function [phi, phi_d] = insert_steer_vel_input(timesteps, insert_at, duration, steer_vel, phi, phi_d)
%     % Insert artificial steering
% 
%     num_timesteps = size(timesteps, 1);
%     start_timestep = 0;
%     end_timestep = 0;
%     for i=1:num_timesteps
%         if timesteps(i) > insert_at && start_timestep == 0
%             start_timestep = i;
%         end
%         if timesteps(i) > insert_at+duration
%             end_timestep = i;
%             break
%         end
%     end
%     if end_timestep == 0
%         end_timestep = num_timesteps;
%     end
% %     end_timestep = start_timestep+duration;
% 
%     insert_section = start_timestep:end_timestep;
% 
%     steer_t = timesteps(insert_section);    
%     
%     phi_d(insert_section) = steer_vel;
%     phi(insert_section) = phi_d(insert_section).*steer_t;
% end
% 
% function [x, y, varphi] = calculate_pose(theta_d, phi, phi_d, R, t)
%     
%     % heading of robot
% 
%     varphi_d = theta_d.*sin(phi);
%     varphi = zeros(size(t, 1), 1);
%     x = zeros(size(t, 1), 1);
%     y = zeros(size(t, 1), 1);
% 
%     for i=1:size(t, 1)
%         if i+1 == size(t, 1)+1
%             break;
%         end
%         varphi(i+1) = varphi(i) + varphi_d(i).*t(i);
% 
%         xdot = R*(theta_d(i).*cos(phi(i)).*cos(varphi(i))+phi_d(i).*sin(varphi(i)));
%         ydot = R*(theta_d(i).*cos(phi(i)).*sin(varphi(i))-phi_d(i).*cos(varphi(i)));
% 
%         x(i+1) = x(i) + xdot*t(i);
%         y(i+1) = y(i) + ydot*t(i);
%     end
% end