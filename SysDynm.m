function Xdot = SysDynm(tint,x)
    % States of the system
     %x = x(1);
     %y = x(2);
     theta = x(3);
    % Gains for the Target and obstacles
k1 =12;
k2 =6;
k3 = 6;

    
    % Vector to target
    r = sqrt((11-x(1))^2 + (12-x(2))^2);
    
    % Force on the robot due to target (x-direction)
    Fxt = (k1*(x(1) - 11))/(r);
    % Force on the robot due to taget (y-direction)
    Fyt = (k1*(x(2) - 12))/(r);
    
    % Vector to obstacle 1 (3,4)
    ro1 = sqrt((3-x(1))^2 + (4-x(2))^2);
    
    % Force on the robot due to obstacle 1 (3,4)(x-direction)
    Fxo1 = -k2*(x(1) - 3)/((ro1)^(3));
    % Force on the robot due to obstacle 1 (3,4)(x-direction)
    Fyo1 = -k2*(x(2) - 4)/((ro1)^(3));
    
    % Vector to obstacle 2 (9,7)
    ro2 = sqrt((9-x(1))^2 + (7-x(2))^2);
    
    % Force on the robot due to obstacle 2 (9,7)(y-direction)
    Fxo2 = -k3*(x(1) - 9)/((ro2)^(3));
    % Force on the robot due to obstacle 2 (9,7)(y-direction)
    Fyo2 = -k3*(x(2) - 7)/((ro2)^(3));
    
    % Total Forces on the robot (x & y direction)
    Fx = Fxt+Fxo1+Fxo2;
    Fy = Fyt+Fyo1+Fyo2;
    
    %   Calculating alpha (Theta desired)
    alpha = atan2(Fy,Fx);
    
    % Calculating phi using the proportional controller
    phi = 5*(alpha - x(3));
    
    %Assuming constant velocity of the robot to be 3.0
    
    % System Dynamics passed to ode solver
    Xdot = [(2*cos(phi)*cos(theta));(2*cos(phi)*sin(theta));(2*sin(phi)/4)];
