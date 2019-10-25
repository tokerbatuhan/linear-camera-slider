
d=1; %defines velocity 
j=1:d:length(thet1); 

for i=1:length(j)


hold off 
%quiver3(x1(i),y1(i),z1(i),0,0,w1(i),'DisplayName','Angular velocity of end of link1')

%quiver(x1(i),y1(i),f1(i,1)',0,0,'DisplayName','force at joint 1 in x direction')
%quiver(x1(i),y1(i),xdot1(i),ydot1(i),'DisplayName','velocity of link1')

hold
%quiver(x2(i),y2(i),xdot2(i),ydot2(i),'DisplayName','velocity of link2')
%quiver(x1(i),y1(i),0,f1(i,2)',0,'DisplayName','force at joint 1 in y direction')
%quiver3(x2(i),y2(i),z2(i),0,0,w2(i),'DisplayName','Angular velocity of end of link2')
plot([x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'o',[0 x1(j(i))],[0 y1(j(i))],'k',[x1(j(i)) x2(j(i))],[y1(j(i)) y2(j(i))],'k')
title('Motion of Camera Slider')
legend()  %%%' 'Connections','Link1','Link2' 'Torque at joint1','Torque at joint2'
%quiver3(0,0,0,0,0,T1(i),'DisplayName','Torque at joint1')
%quiver3(x1(i),y1(i),z1(i),0,0,T2(i),'DisplayName','Torque at joint2')
%hold on
%
%

%
%
axis([-0.7 0.7 -0.7 0.7]);
xlabel('x position(m)')
ylabel('y position(m)')
zlabel('Angular velocity (rad/s)')
legend('Show')
grid

MM(i)=getframe(gcf);
end
drawnow;

v = VideoWriter('Camera Movement.mp4','MPEG-4');
open(v)
writeVideo(v,MM)
close(v)
