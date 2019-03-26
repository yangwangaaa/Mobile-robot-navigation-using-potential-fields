clc
 
% 13X13 grid
[x,y] = meshgrid(0:0.26:13);
 
% Gain of target and the two obstacles
k=2.89;
k1=4.96;
k2=5.7;
 
% Vector to the target (11,12)
r = sqrt((11-x).^2 + (12-y).^2);
 
% Attractive potential for the Target
Vt = k*r;
 
% Vector to the first Obstacle (3,4)
ro1 = sqrt((3-x).^2 + (4-y).^2);
 
% Repulsive Potential of the first Obstacle
Vro1 = k1./ro1;
 
%Vector to the second Obstacle (9,7)
ro2 = sqrt((9-x).^2 + (7-y).^2);
 
% Repulsive potential to the second Obstacle
Vro2 = k1./ro2;
 
% Total Potential 
V = Vt+Vro1+Vro2;
 
% Plotting the Potentials
 
figure(1)
meshc(x,y,Vt)
title('Potential due to Target')
xlabel('X-axis')
ylabel('Y-axis')
 
figure(2)
meshc(x,y,Vro1)
title('Potential due to Obstacle 1')
xlabel('X-axis')
ylabel('Y-axis')
 
figure(3)
mesh(x,y,Vro2)
title('Potential due to Obstacle 2')
xlabel('X-axis')
ylabel('Y-axis')
figure(4)
meshc(x,y,V)
title('Potential Field')
xlabel('X-axis')
ylabel('Y-axis')
