clc
clear
m1=0.8; %defines mass of link1
m2=3.8; %defines mass of link2
l1=0.30; %defines length of link1
l2=0.30; %defines length of link2
t2dot1=0; %Second derivative of time of thet1 in other words acceleration
t2dot2=0; %Second derivative of time of thet2 in other words acceleration
tf=2; %minimum amount of time to do the action
tdot1= pi/tf; %first derivative of time of thet1 in other words velocity
tdot2=pi/tf; %first derivative of time of thet2 in other words velocity
thet1=linspace(0,tdot1*tf,200); %defines values of thet1 in the operation
thet2=linspace(0,tdot2*tf,200); %defines values of thet2 in the operation
for i=1:length(thet1) %defines directional components of force in x, y and z axis
    f1(i,:)=m1*[-l1*(cos(thet1(i)))*tdot1^2-l1*t2dot1*sin(thet1(i)) -l1*sin(thet1(i))*tdot1^2+l1*t2dot1*(cos(thet1(i))) 0]; %defines directional forces needed at link1 
    f2(i,:)=[(m2*(-l1*t2dot1*sin(thet1(i))-l1*tdot1^2*(cos(thet1(i)))-l2*t2dot2*sin(thet2(i))-l2*tdot2^2*(cos(thet2(i))))) 0 0]; %defines directional forces needed at link2
end
%f1i=sqrt(f1(:,1).^2+f1(:,2).^2);
%f2i(:,1)=abs(f2(:,1));
%T1=f1i*l1;
%T2=f2i*l2;



for i=1:length(thet1) %defines points 
    x1(i)=l1*cos(thet1(i)); %definition of x1 at given thet1
    y1(i)=l1*sin(thet1(i)); %definition of y1 at given thet1
    x2(i)=l1*cos(thet1(i))+l2*cos(thet2(i)); %definition of x2 at given thet1 and thet2
    y2(i)=0; %definition of y2 at given thet1 and thet2
    z1(i)=0; %defines z1
    z2(i)=0; %defines z2
end

T2(:)=-f2(:,1).*sin(thet2(:))*l2-f2(:,2).*cos(thet2(:))*l2; %torque needed to be applied at point (x1,y1)
T1(:)=T2(:)+(-f1(:,1)).*sin(thet1(:))*l1+(+f1(:,2)).*cos(thet1(:))*l1; %torque needed to be applied at joint
 
for i=1:length(thet1) %first derivative of time of positions in other words velocity
    xdot1(i)=-l1*sin(thet1(i))*tdot1; %first derivative of time of x1 in other words velocity
    ydot1(i)=l1*cos(thet1(i))*tdot1; %first derivative of time of y1 in other words velocity
    xdot2(i)=-l1*sin(thet1(i))*tdot1-l2*sin(thet2(i))*tdot2; %first derivative of time of x2 in other words velocity
    ydot2(i)=0; %first derivative of time of y2 in other words velocity
end
for i=1:length(thet1) %defines velocity of points (x1,y1) and (x2,y2)
    v1(i)=sqrt(xdot1(i).^2+ydot1(i).^2); %velocity of point (x1,y1)
    v2(i)=sqrt(xdot2(i).^2+ydot2(i).^2); %velocity of point (x2,y2)
end
%f1c(:)=m1*v1(:).^2/l1;
%f2c(:)=m2*v2(:).^2/l2;

for i=1:length(thet1) %defines angular velocity of points (x1,y1) and (x2,y2) 
    w1(i)=v1(i)/l1;  %angular velocity of point (x1,y1)
    w2(i)=v2(i)/l2;  %angular velocity of point (x2,y2)
end
for i=length(thet1) %defines power needed to be applied at points (x1,y1) and (x2,y2)
    p1=T1*w1(i); %power needed at point (x1,y1)
    p2=T2*w2(i); %power needed at point (x2,y2)
end
aa=linspace(1,length(thet1),5); %defines 5 points in the interval equidistantly
aa=round(aa,0); %round the points to the nearest integer
for i=1:length(aa) %defines force components and positions at selected points
    fv1(i,:)=f1(aa(i),:);
    fv2(i,:)=f2(aa(i),:);
    xv1(i)=l1*cos(thet1(aa(i)));
    yv1(i)=l1*sin(thet1(aa(i)));
    xv2(i)=l1*cos(thet1(aa(i)))+l2*cos(thet2(aa(i)));
    yv2(i)=0;
    zv1(i)=0;
    zv2(i)=0;
    fv1(i,3)=0;
    fv2(i,3)=0;
end
quiver3(xv1,yv1,zv1,fv1(:,1)',fv1(:,2)',fv1(:,3)') %plots force components
%%quiver(xv1,yv1,fv1(:,1)',fv1(:,2)')
P=p1+p2; %calculates total needed power
figure
plot(radtodeg(thet2),P) %plots power-position graph
xlabel({'Angle(degree)'},'FontSize',20);

% Create ylabel
ylabel({'Watt(W)'},'FontSize',20);
figure
plot(radtodeg(thet2),abs(T1*10.19)) %plots needed torque-position graph at origin
xlabel({'Angle(degree)'},'FontSize',20);

% Create ylabel
ylabel({'Torque at joint 1(kg*cm)'},'FontSize',20);
figure
plot(radtodeg(thet2),abs(T2*10.19)) %plots needed torque-position graph at point (x1,y1)
xlabel({'Angle(degree)'},'FontSize',20);

% Create ylabel
ylabel({'Torque at joint 2(kg*cm)'},'FontSize',20);
figure
plot(radtodeg(thet2),abs((T1+T2)*10.19)) %plots needed torque-position graph of the system
xlabel({'Angle(degree)'},'FontSize',20);

% Create ylabel
ylabel({'Total torque(kg*cm)'},'FontSize',20);


