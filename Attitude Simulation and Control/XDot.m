function x1_dot=XDot(time,x1)
x1_dot=zeros(13,1);

x1_dot(1,1)=x1(4,1);%x2
x1_dot(2,1)=x1(5,1);%y2
x1_dot(3,1)=x1(6,1);%z2

I1=80;
I2=82;
I3=4;

mu=398600.4188;
r=sqrt(x1(1,1)^2+x1(2,1)^2+x1(3,1)^2);

%External Torque profile
M1=0;
M2=0;
M3=0;

x1_dot(4,1)=-mu*x1(1,1)/r^3;
x1_dot(5,1)=-mu*x1(2,1)/r^3;
x1_dot(6,1)=-mu*x1(3,1)/r^3;

w1=x1(7,1);
w2=x1(8,1);
w3=x1(9,1);

x1_dot(7,1)=(1/I1)*((I2-I3)*w2*w3+M1);
x1_dot(8,1)=(1/I2)*((I3-I1)*w1*w3+M2);
x1_dot(9,1)=(1/I3)*((I1-I2)*w1*w2+M3);

q1=x1(10,1);
q2=x1(11,1);
q3=x1(12,1);
q4=x1(13,1);

x1_dot(10,1)=0.5*(w3*q2-w2*q3+w1*q4);
x1_dot(11,1)=0.5*(w1*q3+w2*q4-w3*q1);
x1_dot(12,1)=0.5*(w2*q1-w1*q2+w3*q4);
x1_dot(13,1)=0.5*(-w1*q1-w2*q2-w3*q3);

end


