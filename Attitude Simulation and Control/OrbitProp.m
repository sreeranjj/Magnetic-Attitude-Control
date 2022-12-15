clc
close all

e=0;                   %eccentricity
theta=30*(pi/180);     %true anomaly
i=30*(pi/180);         %inclination
Omega=25*(pi/180);     %right ascension of ascending node
w=15*(pi/180);         %argument of perifocal point
hperigee=800;
Re=6371;               %Radius of the earth in meter
a=(Re+hperigee)/(1-e); %Semi major axis


Rot=[cos(Omega)*cos(w)-sin(Omega)*sin(w)*cos(i),-cos(Omega)*sin(w)-sin(Omega)*cos(w)*cos(i) ,sin(Omega)*sin(i);
    sin(Omega)*cos(w)+cos(Omega)*cos(w)*cos(i), -sin(Omega)*sin(w)+cos(Omega)*cos(w)*cos(i),-cos(Omega)*sin(i);
    sin(w)*sin(i), cos(w)*sin(i), cos(i)];
Rot1=inv(Rot);
Rot

R1=[cos(Omega),sin(Omega),0;
    -sin(Omega),cos(Omega),0;
    0,0,1];

R2=[1,0,0;
    0,cos(i),sin(i);
    0,-sin(i),cos(i)];
R3=[cos(w),sin(w),0;
    -sin(w),cos(w),0;
    0,0,1];

R4=inv(R1);
R5=inv(R2);
R6=inv(R3);

Rot=R4*R5*R6
%pos=[x1,y1,z1];
%vel=[x2,y2,z2];

p=a*(1-e^2);

%mu=3.986*10^14;             %Specific for earth
mu=398600.4188;
r=p/(1+e*cos(theta))
z=[r*cos(theta);r*sin(theta);0]
LVLHVel=[-sqrt(mu/p)*e*sin(theta);sqrt(mu/p)*(e+cos(theta));0];
pos= Rot*z
vel=Rot*[-sqrt(mu/p)*sin(theta);sqrt(mu/p)*(e+cos(theta));0];
pos;
vel
xo=zeros(13,1);
xo(1,1)=pos(1,1);
xo(2,1)=pos(2,1);
xo(3,1)=pos(3,1);
%Calculation of initial quarternion matrix and angular velocity
xo(4,1)=vel(1,1);
xo(5,1)=vel(2,1);
xo(6,1)=vel(3,1);


AngVel=[0;-0.001;0];
xo(7,1)=AngVel(1,1);
xo(8,1)=AngVel(2,1);
xo(9,1)=AngVel(3,1);

N=[1,0,0;0,1,0;0,0,1];%ECI vector
%Orbital frame of reference unit vectors
a3=(-pos/sqrt(xo(1,1)^2+xo(2,1)^2+xo(3,1)^2))' ;        
a2=(cross(vel,pos)/norm(cross(vel,pos)))';
a1=cross(a2,a3);
A=[a1;a2;a3];
Ran=A*N;

%Theta1,Theta2 and Theta3 are Euler angles
Theta1=2*(pi/180);
Theta2=6*(pi/180);
Theta3=5*(pi/180);

Rba=[cos(Theta1)*cos(Theta3),sin(Theta3),-sin(Theta2)*cos(Theta3);
    -cos(Theta1)*cos(Theta2)*sin(Theta3)+sin(Theta1)*sin(Theta2),cos(Theta1)*cos(Theta3),cos(Theta1)*sin(Theta2)*sin(Theta3)+sin(Theta1)*cos(Theta2);
    sin(Theta1)*cos(Theta2)*sin(Theta3)+cos(Theta1)*cos(Theta3),-sin(Theta1)*cos(Theta3),-sin(Theta1)*sin(Theta2)*sin(Theta3)+cos(Theta1)*cos(Theta3)];

Rbn=Rba*Ran; %This is not actually required
%The quarternions can be dirctly multiplied
%Calculating initial quarternion values
q4a=0.5*sqrt(1+Ran(1,1)+Ran(2,2)+Ran(3,3));
q1a=(1/4*q4a)*(Ran(2,3)-Ran(3,2));
q2a=(1/4*q4a)*(Ran(3,1)-Ran(1,3));
q3a=(1/4*q4a)*(Ran(1,2)-Ran(2,1));

qa=[q1a,q2a,q3a];

q4b=0.5*sqrt(1+Rba(1,1)+Rba(2,2)+Rba(3,3));
q1b=(1/4*q4b)*(Rba(2,3)-Rba(3,2));
q2b=(1/4*q4b)*(Rba(3,1)-Rba(1,3));
q3b=(1/4*q4b)*(Rba(1,2)-Rba(2,1));

qb=[q1b,q2b,q3b];

Q=[ q4b, q3b,-q2b, q1b;
   -q3b, q4b, q1b, q2b;
    q2b,-q1b, q4b, q3b;
   -q1b,-q2b,-q3b, q1b];

Qo=Q*[qa';q4a];
xo(10,1)=Qo(1,1);
xo(11,1)=Qo(2,1);
xo(12,1)=Qo(3,1);
xo(13,1)=Qo(4,1);

%The actual simulation start here
xo;
xo=[ 3083.3 ; 5397.8 ; 3589 ; -6.4 ; 1.1 ; 3.7 ; 0.0 ; -1 ; 0 ; -12.3 ; -842.1 ; 451.3 ; 295.1 ];
tspan=[0 50000];

[time,x1] = ode45(@XDot,tspan,xo);
%r=sqrt(x1(:,1).^2+x1(:,2).^2+x1(:,3).^2);

%plot3(x1(:,1),x1(:,2),x1(:,3))
%grid on
 plot(time,x1(:,7));
% hold on
 %plot(time,x1(:,1));

  %plot(time,x1(:,7),'y',time,x1(:,8),'m',time,x1(:,9),'c')


%Getting Euler angles from Quarternions

% theta1=atan(2.*(x1(:,10).*x1(:,13)-x1(:,11).*x1(:,12))./(1-(2.*x1(:,10).*x1(:,10))-2.*x1(:,12).*x1(:,12)));
% 
% theta2=atan(2.*(x1(:,11).*x1(:,13)-x1(:,10).*x1(:,12))./(1-(2.*x1(:,11).*x1(:,11))-2.*x1(:,12).*x1(:,12)));
% 
% theta3=asin(2.*(x1(:,10).*x1(:,11)+x1(:,12).*x1(:,12)));

% plot(time,theta3)
% ylabel('w2');

%hold on
%plot(time,x1(:,11))
%ylabel('q2');
%hold on
%plot(time,x1(:,12))
%ylabel('q3');







