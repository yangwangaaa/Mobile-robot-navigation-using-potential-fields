clear all
close all
clc
t=0:0.1:25;
x0=[1 ;1 ;0 ;0];
global k 
k=[7 4 6];
[t,Xdot] = ode23('followbots',t,x0);
 
[x,y] = meshgrid(0:0.26:13);
k= 7;

 
% Potential Field Calculation
r = sqrt((11-x).^2 + (12-y).^2);
Vg = 100*r;
r1 = sqrt((3-x).^2 + (4-y).^2);
Vr1 = 75./r1;
r2 = sqrt((9-x).^2 + (7-y).^2);
Vr2 = 75./r2;
V = Vg+Vr1+Vr2;
 
% Plotting Leader's path

figure(1)
h = plot(nan,nan);

grid on
%axis([0 14 0 14])
for i = 1:length(Xdot)
      
 
    pause(0.1);
    hold on
    set(h,'XData',Xdot(1:i,3),'YData',Xdot(1:i,4));
  
    drawnow
    hold off
 
end

figure(2)
  plot(Xdot(:,1),Xdot(:,2),Xdot(:,3),Xdot(:,4))
  hold on
  contour(x,y,V)
  hold off




