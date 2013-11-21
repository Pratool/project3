function plane()
% Modeling a Boeing 727

    g = 9.81;        % acceleration due to gravity (m/s^2)
    m = 65000;      % mass of airplane (kg)
    rho = 1.3;      % density of air (kg/m^3)
    A = 330;        % cross-sectional area of plane (m^2)
    C_l = 0.56;     % coefficient of lift (dimensionless)
    C_D = 0.6;      % coefficient of drag (dimensionless)
    
    %F_thrust = 750000;
    
    initial_pos = [0, 0];
    initial_vel = [40, 10];
    
    options = odeset('events', @events);
    
    [TIME, Y] = ode45(@differentials, [0, 2000], [initial_pos, initial_vel], options);
    
    plot(Y(:,1), Y(:,2));
    axis([0, 2e4, 0, 10000]);
    xlabel('Horizontal Position (m)');
    ylabel('Vertical Position (m)');
    title('Trajectory of Zero-Gravity Aircraft');

    function res = differentials(t, W)
        P = W(1:2);
        V = W(3:4);
        
        v = norm(V);
        v_hat = V / v;
        s_hat = fliplr(v_hat);
        
        % Different ways of varying thrust
        F_thrust = 1.75*7500000*exp(-0.139*t);
        %F_thrust = 7500000*( cos(2*pi*t / 4));
        %F_thrust = 750000*round(1.5*cos(2*pi*t));
        
        dPdt = V;
        dVdt = [0; -g] + ([F_thrust*cos(pi/4);F_thrust*cos(pi/4)]/m) + ((rho*A*v^2) / (2*m)) * (C_l*s_hat - C_D*v_hat);
        
        res = [dPdt; dVdt];
    end

    function [value, isterminal, direction] = events(t, W)
        value = W(2);
        
        direction = -1;
        isterminal = 1;
    end

    function [value, isterminal, direction] = events2(t, W)
        value = W(2) - 30;
        
        direction = -1;
        isterminal = 1;
    end

end