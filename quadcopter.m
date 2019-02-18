function [t,x] = quadcopter(u, x0, t, step)
    % Inputs
    w1 = u(1);
    w2 = u(2);
    w3 = u(3);
    w4 = u(4);

    % Parameters
    Ixx = 4.856e-3;
    Iyy = Ixx;
    Izz = 8.801e-3;

    k = 5.98e-5;        % cosntante de empuje
    b = 1.14e-7;        % constante de arrastre
    l = 0.225;          % distancia entre el motor el centro de masa del quadcopter.
    JR = 0.5e-5;        % momento de inercia del motor.
    m = 3.5;            % masa del quadcopter
    g = 9.8;            % constante de gravedad
    
    % Processing Inputs
    f1 = k*w1^2;
    f2 = k*w2^2;
    f3 = k*w3^2;
    f4 = k*w4^2;

    F = (f1+f2+f3+f4);

    t1 = b*w1^2;
    t2 = b*w2^2;
    t3 = b*w3^2;
    t4 = b*w4^2;

    t_phi = l*k*(-w2^2+w4^2);
    t_theta = l*k*(-w1^2+w3^2);
    t_psi = (t1-t2+t3-t4);

    w_r = w1-w2+w3-w4;
    
    % x = x(1)
    % y = x(2)
    % z = x(3)
    % phi = x(4)
    % theta = x(5)
    % psi = x(6)
    % d_x = x(7)
    % d_y = x(8)
    % d_z = x(9)
    % d_phi = x(10)
    % d_theta = x(11)
    % d_psi= x(12)
    
    %     F_PLANT = @(t,x) [
    %         d_x;
    %         d_y;
    %         d_z;
    %         d_phi;
    %         d_theta;
    %         d_psi;
    %         (F/m)*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi));
    %         (F/m)*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi));
    %         (F/m)*(cos(phi)*cos(theta)) - g;
    %         (t_phi/Ixx) + (JR*w_r*d_theta/Ixx) + (d_theta*d_psi*(Iyy - Izz)/Ixx);
    %         (t_theta/Iyy) + (JR*w_r*d_phi/Iyy) + (d_theta*d_psi*(Izz - Ixx)/Iyy);
    %         (t_psi/Izz) + (d_phi*d_theta*(Ixx - Iyy)/Izz);
    %     ];

    F_PLANT = @(t,x) [
        x(7);
        x(8);
        x(9);
        x(10);
        x(11);
        x(12);
        (F/m)*(cos(x(4))*sin(x(5))*cos(x(6))+sin(x(4))*sin(x(6)));
        (F/m)*(cos(x(4))*sin(x(5))*sin(x(6))-sin(x(4))*cos(x(6)));
        (F/m)*(cos(x(4))*cos(x(5))) - g;
        (t_phi/Ixx) + (JR*w_r*x(11)/Ixx) + (x(11)*x(12)*(Iyy - Izz)/Ixx);
        (t_theta/Iyy) + (JR*w_r*x(10)/Iyy) + (x(11)*x(12)*(Izz - Ixx)/Iyy);
        (t_psi/Izz) + (x(10)*x(11)*(Ixx - Iyy)/Izz);
    ];

    [t,x] = ode45(F_PLANT,[t,t+step],x0);
end