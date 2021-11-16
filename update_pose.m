function update_pose(theta_d, phi, phi_d, R, dt)
    global cur_pose cur_varphi;
    varphi_d = theta_d*sin(phi);
    varphi = cur_varphi + varphi_d*dt;

    xdot = R*(theta_d*cos(phi)*cos(varphi)+phi_d*sin(varphi));
    ydot = R*(theta_d*cos(phi)*sin(varphi)-phi_d*cos(varphi));

    x = cur_pose(1) + xdot*dt;
    y = cur_pose(2) + ydot*dt;

    cur_pose(1) = x;
    cur_pose(2) = y;
    cur_varphi = varphi;
end