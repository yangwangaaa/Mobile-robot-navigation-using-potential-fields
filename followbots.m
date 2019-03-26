function Xdot = followbots(t,x)
   
    % States of the system
    
    %theta = x(3);
    % Gains for the Target and obstacles
   
     global k; 
    
    % distance from goal
    rg = sqrt((11-x(1))^2 + (12-x(2))^2);
    % distance from obstacle 1 (3,4)
    r1 = sqrt((3-x(1))^2 + (4-x(2))^2);
    % Distance from obstacle 2 (9,7)
    ro1 = sqrt((x(3)-x(1))^2 + (x(4)-x(2))^2);
    
    
    % x-direction Force on the robot due to goal 
    Fxg = -k(1)*(x(1) - 11)/(rg);
    % x-direction Force on the robot due to obstacle 1
    Fx1 = k(2)*(x(1) - 3)/((r1)^(3));
    
    Fxf = k(2)*(x(1) - x(3))/((ro1)^(3));
    
    % Total x direction Force on the robot 
    Fx = Fxg+Fx1+Fxf;
    
    
    
    % y-direction Force on the robot due to goal 
    Fyg = -k(1)*(x(2) - 12)/((rg));
    % x-direction Force on the robot due to obstacle 1
    Fy1 = k(2)*(x(2) - 4)/((r1)^(3));    
   
    Fyf = k(2)*(x(2) - x(3))/((ro1)^(3));
    
    % Total y direction Force on the robot
    Fy = Fyg+Fy1+Fyf;
    
    
   %force equation for follower robots
    
    
        % distance from goal
    rfg = sqrt(x(1)-x(3))^2 + (x(2)-x(4))^2;
    % distance from obstacle 1 (3,4)
    rf1 = sqrt((3-x(3))^2 + (4-x(4))^2);
    % Distance from obstacle 2 (9,7)
    rf2 = sqrt((9-x(3))^2 + (7-x(4))^2);
    
    
    % x-direction Force on the robot due to goal 
    Fxfg = -17*(x(3) - x(1))/(rfg);
    % x-direction Force on the robot due to obstacle 1
    Fxf1 = k(2)*(x(3) - 3)/((rf1)^(3));
    % x-direction Force on the robot due to obstacle 2
    FxfL = k(3)*(x(3) - x(1))/((rf2)^(3));
    
    % Total x direction Force on the robot 
    FxfT = Fxfg+Fxf1+FxfL;
    
    
    
    % y-direction Force on the robot due to goal 
    Fyfg = -17*(x(4) - x(2))/((rfg));
    % x-direction Force on the robot due to obstacle 1
    Fyf1 = k(2)*(x(4) - 4)/((rf1)^(3));    
    % y-direction Force on the robot due to obstacle 2
    FyfL = k(2)*(x(4) - x(2))/((rf2)^(3));
    
    FyfT=Fyfg+Fyf1+FyfL;
    
    
    %   Calculating alpha (Theta desired)
    %alpha = atan2(Fy,Fx);
    
    % Calculating phi using the proportional controller
    %phi = 6*(alpha - theta);
    
    % System Dynamics passed to ode solver
    Xdot = [Fx;Fy;FxfT;FyfT];
   
end