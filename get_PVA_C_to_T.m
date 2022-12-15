function PVA_C_relative_to_T = get_PVA_C_to_T(XT,XC)

%PVA_C_relative_T = get_PVA_C_to_T(XT,XC)
%this function will Help to change initial state vector of Target and
%Chaser (for rendezvous purpose) which are in geocentric frame into the Target reference frame 
%
%XT = State Matrix of Target
%XC = State Matrix of Chaser
%Ra_vec= Position VEctor of TARGET in GEocentric Frame in Km
%VT_vec= Velocity VEctor of TARGET in GEocentric Frame in Km/sec
%Rb_vec= Position VEctor of CHASER in GEocentric Frame in Km
%Vb_vec= Velocity VEctor of CHASER in GEocentric Frame in Km/sec 

%NOTATION: [I J K] for Geocentric Frame
%NOTATION: [i j k] for Target reference Frame

%GM is constant for earth and relativly small sized satellites
global GM  ;  %in Km^3/sec^2

%Geocentric Frame
%I=[1 0 0];
%J=[0 1 0];
%K=[0 0 1];

RT_vec=XT(1:3)';
VT_vec=XT(4:6)';

RC_vec=XC(1:3)';
VC_vec=XC(4:6)';

RT=norm(RT_vec);
RC=norm(RC_vec);

%because we need to find Reference frame wrt target , we use its orbit to find [i j k]  
hT_vec=cross(RT_vec,VT_vec);
hT=norm(hT_vec);

i=RT_vec/RT;
k=hT_vec/hT;
j=cross(k,i);

%hence Transformation Matrix from Geo to T-Frame = QGT is given by
QGT = [i;j;k];

%Angular Rotation of Target around EARTH CENTRE =Angular Rotation Of T
%about its own Frame =Omega which is NOT RAAN (omg)
%Angular Accerelation=Omega_dot

Omega=hT_vec/(RT^2);
Omega_dot=(-2)*(dot(VT_vec,RT_vec))*hT_vec/(RT^4);

%Calculating Absolute Acceleration of A and B using the Orbit Equation
AccT_vec=keplerian_acc(RT_vec')';
AccC_vec=keplerian_acc(RC_vec')';

%r_rel=Relative Position Vector of B(Chaser) wrt A(Target) 
%v_rel=Relative Velocity Vector of B(Chaser) wrt A(Target)
%a_rel=Relative Accerelation Vector of B(Chaser) wrt A(Target)

r_rel_G=RC_vec-RT_vec;
v_rel_G=VC_vec-VT_vec-cross(Omega,r_rel_G);
a_rel_G=AccT_vec-AccC_vec-cross(Omega_dot,r_rel_G)-cross(Omega,cross(Omega,r_rel_G))-2*(cross(Omega,v_rel_G));

%But above results r_vel, v_rel and a_rel are in Geocentric Frame 
%we Need to transform them to T-frame hence the following

r_rel_T=QGT*(r_rel_G');
v_rel_T=QGT*(v_rel_G');
a_rel_T=QGT*(a_rel_G');

PVA_C_relative_to_T=[r_rel_T;v_rel_T;a_rel_T];

end