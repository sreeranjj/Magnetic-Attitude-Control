%This is the code for weighted PD attitude control
close all
clc
%Orbital frame is considered here
dt = 0.1;            %Time step
T = 50000.0;        %Duration of the simulation

n = T/dt;      %Number of trials
tk=zeros(1,n+1);     %Time vector
tk(1) = 0;           %First element of the time vector
x=zeros(6,n+1);      %Propagating state vector
q=zeros(n+1,4);
w=zeros(n+1,3);
Ttrue=zeros(3,n+1);
%x(:,1)=[theta1,theta2,theta3,w1,w2,w3]';  %Initial state 
q(1,:)=[0.86,0.08,0.402,0.303];
w(1,:)=[0.001,0.001,-0.001];
Tmag=zeros(3,n+1);
Kp=300;
Kv=290;
e=0.001;

for k = 1:n
   
    SW=[0  -w(k,3)  w(k,2);
        w(k,3)  0  -w(k,1);
       -w(k,2)  w(k,1)  0];
  
   Aq=[q(k,4)^2+q(k,1)^2-q(k,2)^2-q(k,3)^2    2*(q(k,1)*q(k,2)+q(k,4)*q(k,3))      2*(q(k,1)*q(k,3)-q(k,4)*q(k,2)) ;
         2*(q(k,1)*q(k,2)-q(k,4)*q(k,3))    q(k,4)^2-q(k,1)^2+q(k,2)^2-q(k,3)^2    2*(q(k,2)*q(k,3)+q(k,4)*q(k,1)) ;
         2*(q(k,1)*q(k,3)+q(k,4)*q(k,2))      2*(q(k,2)*q(k,3)-q(k,4)*q(k,1))     q(k,4)^2-q(k,1)^2-q(k,2)^2+q(k,3)^2];
     
     
    Rq=0.5*[ q(k,4)  -q(k,3)  q(k,2);
             q(k,3)   q(k,4) -q(k,1);
             -q(k,2)  q(k,1)  q(k,4);
             -q(k,1)  -q(k,2) q(k,3)];
         
    tk(k+1)=k*dt;    %Propagation of the time vector
   
   a=7000;       %Height of orbit plus radius of Earth
   i=97.8*pi/180;    %Angle of inclination of orbit
   mu=398600.4188;   %Mu=G*Me

   w0=sqrt(mu/a^3);  %Average orbital angular rate 
   
   W0=-w0*Aq(:,3);   %Orbital angular velocity with respect to inertial frame
   
   wr=w(k,:)'-W0;     %Angular velocity of the satellite wrt orbital reference frame
   
   
   

   J1=40;
   J2=120;        %Moment of inertia values
   J3=30;  


  
  Z=zeros(3,3);
  
J=[ J1, 0 , 0 ;
    0, J2 , 0 ;
    0, 0 , J3 ];

qr=[q(k,1),q(k,2),q(k,3)]';
%Control Law
u=-(e^2*Kp*qr + e*Kv*wr);

%Earth's magnetic field vector which is a function of time 
Bmag=((7.9*10^15)/a^3)*[ 2*sin(w0*tk(k+1))*sin(i);
                        cos(w0*tk(k+1))*sin(i);
                         -cos(i) ];

 b=Bmag/norm(Bmag);

 Sb=[0   -b(3)   b(2);
     b(3)  0    -b(1);
     -b(2) b(1)   0  ];

Tmag(:,k)=(Sb*Sb')*u;
 
 
 Jinv=inv(J);
 Je=J*Aq(:,1);
 SIex=  [0  -Je(3)  Je(2);
        Je(3)  0  -Je(1);
       -Je(2)  Je(1)  0];
 
 
 dw=dt*(Jinv*(SW*J*w(k,:)'+3*w0^2*SIex*Aq(:,1)+Tmag(:,k)));
 dq=dt*Rq*wr;
 
 w(k+1,:)=w(k,:)+dw';         %Since dt is small,this numerical
 q(k+1,:)=q(k,:)+dq';         %approximation can be made
                   
end  

figure('name','Torque Profile','numbertitle','off')
subplot(3,1,1)
plot(tk,Tmag(1,:)); grid on;
ylim([-0.005,0.005]);
title('Torque Profile');
xlabel('Time (in seconds)');
ylabel('Tx');

subplot(3,1,2)
plot(tk,Tmag(2,:)); grid on;
xlabel('Time (in seconds)');
ylabel('Ty');

subplot(3,1,3)
plot(tk,Tmag(3,:)); grid on;
xlabel('Time (in seconds)');
ylabel('Tz');


figure('name','Quarternions','numbertitle','off')
   subplot(4,1,1)
   plot(tk,q(:,1));grid on; 
   title('Attitude Profile');
   xlabel('Time (in seconds)');
   ylabel('q1');

   subplot(4,1,2)
   plot(tk,q(:,2));grid on; 
   xlabel('Time (in seconds)');
   ylabel('q2');

   subplot(4,1,3)
   plot(tk,q(:,3));grid on; 
   xlabel('Time (in seconds)');
   ylabel('q3');
   
   subplot(4,1,4)
   plot(tk,q(:,4));grid on; 
   xlabel('Time (in seconds)');
   ylabel('q4');
   
   
figure('name','Angular Velocity','numbertitle','off')
   subplot(3,1,1)
   plot(tk,w(:,1));grid on; 
   title('Angular Velocity Profile');
   xlabel('Time (in seconds)');
   ylabel('w1');

   subplot(3,1,2)
   plot(tk,w(:,2));grid on; 
   xlabel('Time (in seconds)');
   ylabel('w2');

   subplot(3,1,3)
   plot(tk,w(:,3));grid on; 
   xlabel('Time (in seconds)');
   ylabel('w3');



   
 %plot(tk,Ttrue(3,:)) 