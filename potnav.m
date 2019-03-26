clear all
% 13X13 grid
[x,y] = meshgrid(0:0.26:13);
 
% Gain of target and obstacles
k1 =12;
k2 =6;
k3 = 6;
 
% Vector to the target
r = sqrt((11-x).^2 + (12-y).^2);
 
% Attractive potential for the Target
Vg = k1*r;
 
% Vector to the first Obstacle (3,4)
r1 = sqrt((3-x).^2 + (4-y).^2);
 
% Repulsive Potential of the first Obstacle
Vr1 = k2./r1;
 
%Vector to the second Obstacle (9,7)
r2 = sqrt((9-x).^2 + (7-y).^2);
 
% Repulsive potential to the second Obstacle
Vr2 = k3./r2;
 
% Total Potential /Potential Field
V = Vg+Vr1+Vr2;

 
 
 
% Time interval for simulation
tint = [0 20];
 
% Initial Conditions for x , y & theta
x0 = [0 0 0];
 
% System dynamics solution using ode23()
[t,Xdot] = ode23('SysDynm',tint,x0);
 
% Plotting Results (Path)     
 
 
figure(1)
h = plot(nan,nan);
hold on
grid on
axis([0 14 0 14])
for i = 1:length(Xdot)
      
 
    pause(0.1);
    set(h,'XData',Xdot(1:i,1),'YData',Xdot(1:i,2));
    drawnow
 
end
hold off 
 
figure(2)
meshc(x,y,V)
hold on
plot(Xdot(:,1),Xdot(:,2)) 
hold off
title('Path of robot')
xlabel('X-axis')
ylabel('Y-axis')
