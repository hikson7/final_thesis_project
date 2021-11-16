function [theta_d_des, phi_des, phi_d_des] = pursuit_kinematic_model(t, traj, trajd)

% design and control parameters.
global R epi_prev epi_sum phi_des_prev;
global cur_pose cur_varphi;

k0 = 1;
k1 = 1;
k2 = 2;
k3 = 1;

% [x_des, y_des] = traj(t);
% [xd_des, yd_des] = trajd(t);

[x_des, y_des] = trajsin(t);
[xd_des, yd_des] = trajdsin(t);

pose_des = [x_des; y_des];

[error_k, epi] = find_errors(cur_pose, pose_des, cur_varphi, k3);

theta_d_des = desired_theta_d(xd_des, yd_des, k0, k1, error_k, epi);

phi_des = desired_phi(error_k, epi, epi_prev, epi_sum, k2);
phi_d_des = phi_des - phi_des_prev;

% update control parameters.
epi_sum = epi_sum + epi;
epi_prev = epi;
phi_des_prev = phi_des;
end

function [error_k, epi] = find_errors(pose_cur, pose_des, varphi, k3)
    error = pose_cur - pose_des;

    x_len = pose_des(1)-pose_cur(1);
    y_len = pose_des(2)-pose_cur(2);

    if abs(x_len) < 0.00001
        x_len = 0;
    end
    if abs(y_len) < 0.00001
        y_len = 0;
    end
    
    sigma = atan2(y_len, x_len);    
    
    if sigma == 0
        sigma = varphi;
    end
    
    epi = sigma - varphi;        
    error_k = norm(error)/(k3+norm(error));
end

function theta_d_des = desired_theta_d(xd_des, yd_des, k0, k1, error_k, epi)
    global R;
    theta_d_des = 1/R*norm([xd_des, yd_des])*k0+k1*error_k*cos(epi);
end

function phi_des = desired_phi(error_k, epi, epi_prev, epi_sum, k2)
    % kp = 1;
    % ki = 1;
    % kd = 260;
    kp = 1;
    ki = 1;
    kd = 260;

    k_epi = kp + ki*epi_sum + kd*(epi-epi_prev);
    phi_des = -k2*error_k*sin(epi)*k_epi;
end
