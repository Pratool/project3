function punchline()

    k = 1:41;
    for j = k
        value = 1e5*(j+13);
        THRUSTS(j) = plane2( value );
    end
    
    plot(1e5*(k+13), THRUSTS, 'linewidth', 2.5);
    
    xlabel('Thrust Force (N)', 'FontSize', 14);

    ylabel('Length of Time of Weightlessness (s)', 'FontSize', 14);
    title('Trajectory of Zero-Gravity Aircraft', 'FontSize', 18);
%     legend('Accelerations', 'Angle off ground');

    function time_spent = plane2(F_thrust_2)
    % Modeling a Boeing 727

        g = 9.81;       % acceleration due to gravity (m/s^2)
        m = 65000;      % mass of airplane (kg)
        rho = 1.3;      % density of air (kg/m^3)
        A = 330;        % cross-sectional area of plane (m^2)
        C_l = 0.56;     % coefficient of lift (dimensionless)
        C_D = 0.6;      % coefficient of drag (dimensionless)

        F_thrust = 2000000;
        initial_pos = [0, 0];
%         initial_vel = [40, 10];
        intial_vel = [1, 1];

        options = odeset('events', @events);
        options2 = odeset('events', @events2);

        [TIME_2, Y_2] = ode45(@thrust_on_2, 0:2000, [23560, 7300, 110, -182], options);
        T = TIME_2(1:end-1);
        Y = Y_2;
        
        [TIME_3, Y_2] = ode45(@thrust_off, [T(end):4000], [Y(end, 1), Y(end, 2), Y(end, 3), Y(end, 4)], options2);
        
        Y = [Y; Y_2];
        T = [T; TIME_3];
        
        for i = 1:length(T)-2    % i is the index of the time
            THETA(i) = asind(Y(i+1, 2)/Y(i+1, 1));
            ACCELERATIONS(i) = ( norm( g*cosd(THETA(i)) + (-sind(THETA(i)) * (Y(i+1, 3)-Y(i, 3)/(T(i+1)-T(i))) + cosd(THETA(i))*(Y(i+1, 4)-Y(i, 4)/(T(i+1)-T(i))) ) ) ) / 9.8;
        end
        zerogs = ACCELERATIONS < 1;
        ZEROgs = T(zerogs);
        time_spent = ZEROgs(end)-ZEROgs(1);
        
%         hold all;
%         plot(T(1:end-2), ACCELERATIONS);
%         plot(T, Y(:,2));

%         TIMES1(i) = [ TIMES(i)(2:end), TIMES2(2:end) ];
%         ACCELERATIONS = ( ([ACCELERATIONS1(1:end), ACCELERATIONS2(1:end)]) / g ) - 1;

%         ZeroGs = ACCELERATIONS < 1;

%         T_at_ZeroGs = TIMES(ZeroGs);

%         time_spent = max(T_at_ZeroGs) - min(T_at_ZeroGs);
%         time_spent = T_at_ZeroGs(end-1) - T_at_ZeroGs(2);

%         plot(TIMES(ZeroGs), ACCELERATIONS(ZeroGs), 'o');
% 
%         xlabel('Time (Seconds)', 'FontSize', 14);
% 
%         ylabel('Angle of Plane (degrees)', 'FontSize', 14);
%         title('Trajectory of Zero-Gravity Aircraft', 'FontSize', 18);
%         legend('Accelerations', 'Angle off ground');

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
end