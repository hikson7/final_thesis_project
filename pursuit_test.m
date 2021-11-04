function pursuit_test

% clc
clear
% clf
t_top = 10;
% t_top = 1.5;
% t_top = 0.01*20;
% t_top = 0.06;

t = 0:0.01:t_top;
tlen = size(t, 2);

x_des = t';
y_des = sin(t)';
xd_des = 1*ones(1, tlen);
yd_des = cos(t);

pose_des = [x_des y_des];

phi_des = zeros(tlen, 1);

cur_pose = [0; 0];

% heading of robot
varphi = zeros(tlen, 1);
varphi(1) = pi/4;

x = zeros(tlen, 1);
y = zeros(tlen, 1);

k0 = 1;
k1 = 5;
k2 = 1;
k3 = 2;

kp = 1;
ki = 1.1;
kd = 0.1;

R = 1;
epis = zeros(tlen, 1);
errors = zeros(tlen, 1);
sigmas = zeros(tlen, 1);
% epi_ds = zeros(tlen, 1);

phi_prev = 0;
epi_prev = 0;
epi_sum = 0;
t_prev = -0.01;

for i=1:tlen
    dt = t(i)-t_prev;
    
    error = cur_pose - pose_des(i, :)';

    x_len = x_des(i)-cur_pose(1);
    y_len = y_des(i)-cur_pose(2);

    if abs(x_len) < 0.00001
        x_len = 0;
    end
    if abs(y_len) < 0.00001
        y_len = 0;
    end
    sigma = atan2(y_len, x_len);    
    
    sigmas(i) = sigma*180/pi;
    sigmas(i, 2) = x_len;
    sigmas(i, 3) = y_len;
    
    if sigma == 0
        sigma = varphi(i);
    end
    
    epi = sigma - varphi(i);
    
    epi_sum = epi_sum + epi;
    k_epi = kp + ki*epi_sum + kd*(epi-epi_prev);

%     k_epi = 1;
%     k3 = 1;
    error_k = norm(error)/(k3+norm(error));
    
    % desired drive velocity
    theta_d_des = 1/R*norm([xd_des(i), yd_des(i)])*k0+k1*error_k*cos(epi);
    
    % ----------- desired steer position
    % Calculate change in heading
%     epi_d = (epi-epi_prev)/dt;
%     epi_ds(i) = epi_d*180/pi;
    
    % Calculate desired steer angle.
%     phi_des(i) = k2*error_k*sin(epi_d/theta_d_des);
%     phi_des(i) = k2*error_k*sin(epi/theta_d_des);
    phi_des(i) = -k2*error_k*sin(epi)*k_epi;
%     phi_des(i) = -asin(epi_d/theta_d_des);
    
%     phi_des(i) = -k2*sin(epi);
%     phi_d_des = (phi_des(i)-phi_prev)/dt;
    phi_d_des = phi_des(i)-phi_prev;
   
    % Heading
    varphi_d = theta_d_des*sin(phi_des(i));
    
    % find change
    xdot = R*(theta_d_des*cos(phi_des(i)).*cos(varphi(i))+phi_d_des*sin(varphi(i)));
    ydot = R*(theta_d_des*cos(phi_des(i)).*sin(varphi(i))-phi_d_des*cos(varphi(i)));
    
    if i+1 == tlen+1
        break;
    end
    
    % update current pose
    x(i+1) = x(i) + xdot*dt;
    y(i+1) = y(i) + ydot*dt;
    varphi(i+1) = varphi(i) + varphi_d*dt;
    
    cur_pose = [x(i+1); y(i+1)];
    
    % update derivative parameters
    t_prev = t(i);
    epi_prev = epi;
    phi_prev = phi_des(i);
    
    errors(i) = norm(error);
%     errors(i, 1) = error(1);
%     errors(i, 2) = error(2);
    epis(i) = epi*180/pi;
end

figure(1);
plot(x_des, y_des, 'b-');

hold on
plot(x, y, 'r-');
hold off

title("Desired path");
xlabel("x displacement [m]")
ylabel("y displacement [m]")
grid on;
grid minor;

figure(2);
subplot(2, 1, 1);

plot(t, epis, 'b-');
grid on;
grid minor;
title("Heading error (epislon)")
xlabel("time [s]")
ylabel("error")

subplot(2, 1, 2);
plot(t, errors, 'b-');
grid on;
grid minor;
title("xy norm error");
xlabel("time [t]")
ylabel("error")

end