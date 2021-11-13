function simple_arm
    clc
    clear
    clf

    tspan = [0, 10];
    theta = pi/4;
    global theta_d;
    theta_d = pi/4;
    
    [t, theta_out] = ode45(@arm_model, tspan, theta);
    l = 2;
    
    x = l.*cos(theta_out);
    y = l.*sin(theta_out);
    figure(1);
    plot(x, y, '-b');

    title("Robot arm mode x-y");
    xlabel("x")
    ylabel("y")
    grid on;
    grid minor;
    
    figure(2);
    plot(t, theta_out-theta_d, '-b');

    title("Robot arm mode");
    xlabel("t")
    ylabel("error")
    grid on;
    grid minor;
end