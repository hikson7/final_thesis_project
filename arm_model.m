function theta_out = arm_model(t, theta)

    global theta_d;
    m = 100;
    g = 9.81;
    lc = 1;
%     
%     Kp = 1;
%     
%     tau = Kp*(theta-theta_d);
    tau = 1;
    theta_out = acos(tau/m/g/lc);
end
