function plane()
% Modeling a Boeing 727

    g = 9.81;       % acceleration due to gravity (m/s^2)
    m = 65000;      % mass of airplane (kg)
    rho = 1.3;      % density of air (kg/m^3)
    A = 330;        % cross-sectional area of plane (m^2)
    C_l = 0.56;     % coefficient of lift (dimensionless)
    C_D = 0.6;      % coefficient of drag (dimensionless)
    
    F_thrust = 1000000;
    
    initial_pos = [0, 0];
    initial_vel = [40, 10];
    
    options = odeset('events', @events);
    options2 = odeset('events', @events2);
    
    [TIME, Y] = ode45(@thrust_on, [0, 2000], [initial_pos, initial_vel], options);
    t = TIME(end) - TIME(1);
    [TIME, Y_2] = ode45(@thrust_off, [2000, 4000], [Y(end, 1), Y(end, 2), Y(end, 3), Y(end, 4)], options2);
    
    Y = [Y; Y_2];
    
    %disp(Y);
    
    for i = 2:10
        [TIME, Y_2] = ode45(@thrust_on, [2000*i, 2000*(i+1)], [Y(end, 1), Y(end, 2), Y(end, 3), Y(end, 4)], options);
        Y = [Y; Y_2];
        t = t + (TIME(end) - TIME(1));
        [TIME, Y_2] = ode45(@thrust_off, [2000*(i+1), 2000*(i+2)], [Y(end, 1), Y(end, 2), Y(end, 3), Y(end, 4)], options2);
        Y = [Y; Y_2];
    end

    hold on;
    disp(t);
    plot(Y(:,1), Y(:,2), 'LineWidth', 2);
    %plot(Y_2(:,1), Y_2(:,2));
    %axis([0, 2e4, 0, 10000]);
    xlabel('Horizontal Position (m)', 'FontSize', 14);
    ylabel('Vertical Position (m)', 'FontSize', 14);
    title('Trajectory of Zero-Gravity Aircraft', 'FontSize', 18);

    function res = thrust_on(t, W)
        P = W(1:2);
        V = W(3:4);
        
        v = norm(V);
        v_hat = V / v;
        s_hat = fliplr(v_hat);
        
        %disp(t);
        %disp(F_thrust);
        %hold on;
        %plot(t, F_thrust, 'o');
        
        dPdt = V;
        dVdt = [0; -g] + ([F_thrust*cos(pi/4);F_thrust*cos(pi/4)]/m) + ((rho*A*v^2) / (2*m)) * (C_l*s_hat - C_D*v_hat);
        
        res = [dPdt; dVdt];
    end
    
    function res = thrust_off(t, W)
        P = W(1:2);
        V = W(3:4);
        
        v = norm(V);
        v_hat = V / v;
        s_hat = fliplr(v_hat);
        
        %disp(t);
        %disp(F_thrust);
        %hold on;
        %plot(t, F_thrust, 'o');
        
        dPdt = V;
        dVdt = [0; -g] + ([0; 0]/m) + ((rho*A*v^2) / (2*m)) * (C_l*s_hat - C_D*v_hat);
        
        res = [dPdt; dVdt];
    end

    function [value, isterminal,direction] = events(t, W) % kill thrust when height is increasing
        %value = W(2) - 9756;
        value = W(2) - 9000;
        direction = 1;
        isterminal = 1;
    end

    function [value, isterminal, direction] = events2(t, W) % when plane is too close to ground terminate
        value = W(2) - 8000;
        
        direction = -1;
        isterminal = 1;
    end

end