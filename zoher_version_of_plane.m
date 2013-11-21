function zoher_version_of_place()

    g = 9.81;        % acceleration due to gravity (m/s^2)
    m = 65000;      % mass of airplane (kg)
    rho = 1.3;      % density of air (kg/m^3)
    A = 330;        % cross-sectional area of plane (m^2)
    C_l = 2.56;     % coefficient of lift (dimensionless)
    C_D = 0.6;      % coefficient of drag (dimensionless)
    
    %F_thrust = 750000;
    
    initial_pos = [10, 10];
    initial_vel = [30, 0];
    
    [TIME, Y] = ode45(@differentials, [0, 500], [initial_pos, initial_vel]);
    
    plot(Y(:,1), Y(:,2), 'linewidth', 2);

    function res = differentials(t, W)
        P = W(1:2);
        V = W(3:4);
        
        v = norm(V);
        v_hat = V / v;
        s_hat = fliplr(v_hat);
        
        F_thrust = 750000*exp(-P(2));
        
        dPdt = V;
        dVdt = [0; -g] + ([F_thrust;F_thrust]/m) + ((rho*A*v^2) / (2*m)) * (C_l*s_hat - C_D*v_hat);
        
        res = [dPdt; dVdt];
    end

end