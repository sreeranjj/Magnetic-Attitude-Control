%This is the code for simple PD attitude control
close all
clc

theta1=8*pi/180;     %Phi
theta2=3*pi/180;     %Theta
theta3=8*pi/180;     %Psi
w1=0;                %Omega_x
w2=0;                %Omega_y
w3=0;                %Omega_z



dt = 0.1;            %Time step
T = 100000.0;        %Duration of the simulation
n = T/dt;            %Number of trials
tk=zeros(1,n+1);     %Time vector

tk(1) = 0;           %First element of the time vector
x=zeros(6,n+1);      %Propagating state vector
Ttrue=zeros(3,n+1);
x(:,1)=[theta1,theta2,theta3,w1,w2,w3]';  %Initial state 


for k = 1:n
    
    tk(k+1)=k*dt; %propogation of the time vector
   %[time,x1] = ode45('X_dot',[(k-1)*dt k*dt],xo);
   a=6371+250;
   i=90*pi/180;
   mu=398600.4188;

wo=sqrt(mu/a^3);

J1=152.2;
J2=2690.8;
J3=2652.6;

k1=(J2-J3)/J1;
k2=(J3-J1)/J2;
k3=(J1-J2)/J3;


A=[   0        ,   0     ,   0   ,   1   ,   0   ,     0     ;
      0        ,   0     ,   0   ,   0   ,   1   ,     0     ;
      0        ,   0     ,   0   ,   0   ,   0   ,     1     ;
    -4*wo^2*k1 ,   0     ,   0   ,   0   ,   0   , wo*(1-k1) ;
      0        ,3*wo^2*k2,   0   ,   0   ,   0   ,     0     ;
      0        , 0       , wo^2*k3 , -wo*(1+k3) , 0 , 0 ];

  
  Z=zeros(3,3);
J=[ J1, 0 , 0 ;
    0, J2 , 0 ;
    0, 0 , J3 ];

Jinv=inv(J);

B=[Z;Jinv];

Bmag=((7.9*10^15)/a^3)*[cos(wo*tk(k+1))*sin(i);
                         -cos(i);
                         2*sin(wo*tk(k+1))*sin(i)];
                     
                     
K=[-0.000017,   0  , -2.7*10^-9 , 0.094, 0   , 0.17;
        0   , 0.055,   0        ,   0  , 15.6,   0 ;
  -4.8*10^-8,   0  ,-0.003      , -0.68,  0  , 1.6];

M=(Bmag*Bmag')./(Bmag'*Bmag);
I=eye(3);
Tideal=-K*x(:,k);
Ttrue(:,k)=(I-M)*Tideal;

dx=dt*(A*x(:,k)+B*Ttrue(:,k));

    x(:,k+1)=x(:,k)+dx;
end  

%plot(tk,x(3,:))
plot(tk,Ttrue(3,:))
   
   


  
   
   

    
  