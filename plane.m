function plane()

    g = 9.8;        % acceleration due to gravity (m/s^2)
    m = 65000;      % mass of airplane (kg)
    rho = 1.3;      % density of air (kg/m^3)
    A = 330;        % cross-sectional area of plane (m^2)
    C_l = 0.2;     % coefficient of lift (dimensionless)
    C_D = 0.6;      % coefficient of drag (dimensionless)
    
    %F_thrust = 750000;
    
    initial_pos = [0, 0];
    initial_vel = [40, 10];
    
    [TIME, Y] = ode45(@differentials, [0, 10], [initial_pos, initial_vel]);
    
    plot(Y(:,1), Y(:,2), 'linewidth', 2);

    function res = differentials(t, W)
        P = W(1:2);
        V = W(3:4);
        
        v = norm(V);
        v_hat = V / v;
        s_hat = fliplr(v_hat);
        
        F_thrust = 750000*exp(-0.139*t);
        
        dPdt = V;
        dVdt = [0; -g] + ([F_thrust*cos(pi/4);F_thrust*cos(pi/4)]/m) + ((rho*A*v^2) / (2*m)) * (C_l*s_hat - C_D*v_hat);
        
        res = [dPdt; dVdt];
        disp(dVdt);
    end

end