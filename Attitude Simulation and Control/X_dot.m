function Dx = X_dot(~,x)

global tk k

a=6371+250;
i=pi/2;
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
Tideal=-K*x;
Ttrue=(I-M)*Tideal;

Dx=A*x+B*Ttrue;

end
















