function plane()
% Modeling a Boeing 727

    g = 9.81;       % acceleration due to gravity (m/s^2)
    m = 65000;      % mass of airplane (kg)
    rho = 1.3;      % density of air (kg/m^3)
    A = 330;        % cross-sectional area of plane (m^2)
    C_l = 0.56;     % coefficient of lift (dimensionless)
    C_D = 0.6;      % coefficient of drag (dimensionless)
    
    THETAS1 = 0;
    TIMES1 = 0;
    ACCELERATIONS1 = 0;
    THETAS2 = 0;
    TIMES2 = 0;
    ACCELERATIONS2 = 0;
    THETAS3 = 0;
    TIMES3 = 0;
    ACCELERATIONS3 = 0;

    F_thrust = 2000000;
%     F_thrust_2 = 2800000;
    F_thrust_2 = 1e6;
    initial_pos = [0, 0];
    initial_vel = [40, 10];
    
    options = odeset('events', @events);
    options2 = odeset('events', @events2);
    
    [TIME, Y] = ode45(@thrust_on_1, [0, 2000], [initial_pos, initial_vel], options);
    
    [TIME_2, Y_2] = ode45(@thrust_off, [TIME(end), 4000], [23560, 7300, 110, -182], options2);
    Y = [Y; Y_2];
    T = [TIME; TIME_2];
    
    [TIME_3, Y_2] = ode45(@thrust_on_2, [T(end), 2000], [Y(end, 1), Y(end, 2), Y(end, 3), Y(end, 4)], options);
    Y = [Y; Y_2];
    T = [T; TIME_3];
    
%     for i = 2:10
%         [TIME, Y_2] = ode45(@thrust_on_2, [T(end), 2000*(i+1)], [Y(end, 1), Y(end, 2), Y(end, 3), Y(end, 4)], options);
%         Y = [Y; Y_2];
%         T = [T; TIME];
%         
%         [TIME, Y_2] = ode45(@thrust_off, [T(end), 2000*(i+2)], [Y(end, 1), Y(end, 2), Y(end, 3), Y(end, 4)], options2);
%         Y = [Y; Y_2];
%         T = [T; TIME];
%     end
    
    clf;
    hold all;
    THETAS = [ THETAS1(2:end), THETAS2(2:end) ];
    TIMES = [ TIMES1(2:end), TIMES2(2:end) ];
    ACCELERATIONS = ( ([ACCELERATIONS1(2:end), ACCELERATIONS2(2:end)]) / g ) - 1;
%    plot(TIMES, ACCELERATIONS);
%     plot(TIMES, THETAS);
    
    plot(Y(:,1), Y(:,2), 'r', 'LineWidth', 2);
%    plot(TIME, Y_2(:,5), 'r', 'LineWidth', 2);
    xlabel('Time (Seconds)', 'FontSize', 14);
%     xlabel('Horizontal Position (m)', 'FontSize', 14);
%     ylabel('Vertical Position (m)', 'FontSize', 14);
    ylabel('Angle of Plane (degrees)', 'FontSize', 14);
    title('Trajectory of Zero-Gravity Aircraft', 'FontSize', 18);
    legend('Accelerations', 'Angle off ground');

    function res = thrust_on_1(t, W)
        P = W(1:2);
        V = W(3:4);
        
        v = norm(V);
        v_hat = V / v;
        s_hat = fliplr(v_hat);
        
        dPdt = V;
        dVdt = [0; -g] + ([F_thrust*cos(pi/4);F_thrust*cos(pi/4)]/m) + ((rho*A*v^2) / (2*m)) * (C_l*s_hat - C_D*v_hat);
        
        res = [dPdt; dVdt];
    end

    function res = thrust_on_2(t, W)
        P = W(1:2);
        V = W(3:4);
        
        v = norm(V);
        v_hat = V / v;
        s_hat = fliplr(v_hat);       
        
        dPdt = V;
        dVdt = [0; -g] + ([F_thrust_2*cos(pi/4);F_thrust_2*cos(pi/4)]/m) + ((rho*A*v^2) / (2*m)) * (C_l*s_hat - C_D*v_hat);
        
        theta = asind(V(2,1)/V(1,1));    % angle of plane
        THETAS1(end+1) = theta;
        TIMES1(end+1) = t;
        ACCELERATIONS1(end+1) = norm( g*cosd(theta) * s_hat + (-sind(theta) * dVdt(1, 1) + cosd(theta)*dVdt(2, 1) ) );
        disp(theta);
        disp(t);
        
        res = [dPdt; dVdt];
    end
    
    function res = thrust_off(t, W)
        P = W(1:2);
        V = W(3:4);
        
        v = norm(V);
        v_hat = V / v;
        s_hat = fliplr(v_hat);
        
        dPdt = V;
        dVdt = [0; -g] + ([0; 0]/m) + ((rho*A*v^2) / (2*m)) * (C_l*s_hat - C_D*v_hat);
        
        theta = asind(V(2,1)/V(1,1));    % angle of plane
        THETAS2(end+1) = theta;
        TIMES2(end+1) = t;
        ACCELERATIONS2(end+1) = norm( g*cosd(theta) * s_hat + (-sind(theta) * dVdt(1, 1) + cosd(theta)*dVdt(2, 1) ) );
        disp(theta);
        disp(t);
        
        res = [dPdt; dVdt];
    end

    function [value, isterminal,direction] = events(t, W) % kill thrust when height is increasing
        value = W(2) - 8500;
        
        direction = 1;
        isterminal = 1;
    end

    function [value, isterminal, direction] = events2(t, W) % when plane is too close to ground terminate
        value = W(2) - 7300;
        
        direction = -1;
        isterminal = 1;
    end
end