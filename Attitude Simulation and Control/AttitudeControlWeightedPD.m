%This is the code for weighted PD attitude control
close all
clc
%Orbital frame is considered here

theta1=30*pi/180;     %Phi
theta2=-40*pi/180;     %Theta
theta3=130*pi/180;     %Psi
w1=0.1;                %Omega_x
w2=0.1;                %Omega_y
w3=-.1;                %Omega_z

dt = 0.1;            %Time step
T = 140000.0;        %Duration of the simulation

n = T/dt;      %Number of trials
tk=zeros(1,n+1);     %Time vector
tk(1) = 0;           %First element of the time vector
x=zeros(6,n+1);      %Propagating state vector
Ttrue=zeros(3,n+1);
quat=zeros(n+1,4);
x(:,1)=[theta1,theta2,theta3,w1,w2,w3]';  %Initial state 

for k = 1:n
   
   tk(k+1)=k*dt;    %Propagation of the time vector
   
   a=6371+250;       %Height of orbit plus radius of Earth
   i=90*pi/180;    %Angle of inclination of orbit
   mu=398600.4188;  %Mu=G*Me

   wo=sqrt(mu/a^3);  %Average orbital angular rate 

   J1=152.2;
   J2=2690.8;        %Moment of inertia values
   J3=2652.6;   

   k1=(J2-J3)/J1;
   k2=(J3-J1)/J2;
   k3=(J1-J2)/J3;

%A is the system matrix after linearisation

%   x'=Ax(t)+Bu(t)

A=[   0        ,   0     ,   0   ,   1   ,   0   ,     0     ;
      0        ,   0     ,   0   ,   0   ,   1   ,     0     ;
      0        ,   0     ,   0   ,   0   ,   0   ,     1     ;
    -4*wo^2*k1 ,   0     ,   0   ,   0   ,   0   , wo*(1-k1) ;
      0        ,3*wo^2*k2,   0   ,   0   ,   0   ,     0     ;
      0        , 0       , wo^2*k3 , -wo*(1+k3), 0 , 0 ];

  
  Z=zeros(3,3);
  
J=[ J1, 0 , 0 ;
    0, J2 , 0 ;
    0, 0 , J3 ];

Jinv=inv(J);
 
B=[Z;Jinv];

%Earth's magnetic field vector which is a function of time 

Bmag=((7.9*10^15)/a^3)*[ cos(wo*tk(k+1))*sin(i);
                         -cos(i);
                         2*sin(wo*tk(k+1))*sin(i)];
                     
  
%Gain matrix calculated from pole placement method

K=[-0.000017,   0  , -2.7*10^-9 , 0.094, 0   , 0.17;
        0   , 0.055,   0        ,   0  , 15.6,   0 ;
  -4.8*10^-8,   0  ,-0.003      , -0.68,  0  , 1.6];

%Q is the diagonal weighting matrix

Q=[ 12 , 0 , 0 ;
    0 , 1 , 0 ;
    0 , 0 , 1 ];

quat(k,:)=eul2quat([x(1,k),x(2,k),x(3,k)]);

Qinv=inv(Q);
M=(Qinv*Bmag*Bmag')./(Bmag'*Qinv*Bmag);

    I=eye(3);
    Tideal=-K*x(:,k);
    Ttrue(:,k)=(I-M)*Tideal;

  dx=dt*(A*x(:,k)+B*Ttrue(:,k));      %Since dt is small,this numerical
  x(:,k+1)=x(:,k)+dx;                 %approximation can be made   
end  

figure('name','Torque Profile','numbertitle','off')
subplot(3,1,1)
plot(tk,Ttrue(1,:)); grid on;
%ylim([-0.005,0.005]);
title('Torque Profile');
xlabel('Time (in seconds)');
ylabel('Tx');

subplot(3,1,2)
plot(tk,Ttrue(2,:)); grid on;
xlabel('Time (in seconds)');
ylabel('Ty');

subplot(3,1,3)
plot(tk,Ttrue(3,:)); grid on;
xlabel('Time (in seconds)');
ylabel('Tz');


figure('name','Quarternions','numbertitle','off')
   subplot(4,1,1)
   plot(tk,quat(:,4));grid on; 
   title('Attitude Profile');
   xlabel('Time (in seconds)');
   ylabel('q1');

   subplot(4,1,2)
   plot(tk,quat(:,2));grid on; 
   xlabel('Time (in seconds)');
   ylabel('q2');

   subplot(4,1,3)
   plot(tk,quat(:,3));grid on; 
   xlabel('Time (in seconds)');
   ylabel('q3');
   
   subplot(4,1,4)
   plot(tk,quat(:,1));grid on; 
   xlabel('Time (in seconds)');
   ylabel('q4');
   
   
figure('name','Angular Velocity','numbertitle','off')
   subplot(3,1,1)
   plot(tk,x(4,:));grid on; 
   title('Angular Velocity Profile');
   xlabel('Time (in seconds)');
   ylabel('w1');

   subplot(3,1,2)
   plot(tk,x(5,:));grid on; 
   xlabel('Time (in seconds)');
   ylabel('w2');

   subplot(3,1,3)
   plot(tk,x(6,:));grid on; 
   xlabel('Time (in seconds)');
   ylabel('w3');


   
 %plot(tk,Ttrue(3,:))  