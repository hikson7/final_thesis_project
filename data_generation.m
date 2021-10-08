function data_generation

clc
clear
figure(1);
clf

tspan = [0, 10];

% initial conditions
q1_0 = 0;
q2_0 = 0;
qd1_0 = 0;
qd2_0 = 0;

state = [q1_0; q2_0; qd1_0; qd2_0];

options = odeset("RelTol", 1e-5);

[t, state_out] = ode45(@model, tspan, state, options);

% retrieve necessary outputs
theta_d = state_out(:, 3);  % shell turning velocity

% Insert artificial steering

% steer angles
phi = zeros(size(t, 1), 1);
phi_d = zeros(size(t, 1), 1);

insert_at = 5;      % [s]
duration = 0.5;     % [s]
steer_velocity = 0.01;  % [rad/s]
[phi, phi_d] = insert_steer_vel_input(t, insert_at, duration, steer_velocity, phi, phi_d);

insert_at = 7;
duration = 0.5;
steer_velocity = -0.01;  
[phi, phi_d] = insert_steer_vel_input(t, insert_at, duration, steer_velocity, phi, phi_d);


R = 1;

[x, y, ] = calculate_pose(theta_d, phi, phi_d, R, t);

plot(x, y, 'b-');
grid on;
grid minor;
% range = 1000;

% axis([-range/2 range/2 -range/2 range/2])

figure(2);

plot(t, phi, 'b-');

figure(3);

plot(t, phi_d, 'b-');

end

function [phi, phi_d] = insert_steer_vel_input(timesteps, insert_at, duration, steer_vel, phi, phi_d)
    % Insert artificial steering

    num_timesteps = size(timesteps, 1);
    start_timestep = 0;
    end_timestep = 0;
    for i=1:num_timesteps
        if timesteps(i) > insert_at && start_timestep == 0
            start_timestep = i;
        end
        if timesteps(i) > insert_at+duration
            end_timestep = i;
            break
        end
    end
%     end_timestep = start_timestep+duration;

    insert_section = start_timestep:end_timestep;

    steer_t = timesteps(insert_section);    
    
    phi_d(insert_section) = steer_vel;
    phi(insert_section) = phi_d(insert_section).*steer_t;
end

function [x, y, varphi] = calculate_pose(theta_d, phi, phi_d, R, t)
    
    % heading of robot

    varphi_d = theta_d.*sin(phi);
    varphi = zeros(size(t, 1), 1);
    x = zeros(size(t, 1), 1);
    y = zeros(size(t, 1), 1);

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
end