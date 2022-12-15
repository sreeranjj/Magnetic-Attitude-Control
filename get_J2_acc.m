function J2_acc=get_J2_acc(R)
%R is Position vector of Space craft in ECI
%R is in Row Matrix Form 1 by 3
GM = 3.986004418*(10^5);
J2= 1082.6*(10^-6);
Re=6371;
r=norm(R);
z=R(3);

J2_constant=GM*J2*(Re^2);
A1=15*(z^2)/(r^7);
A2=-3*[1;1;3]/(r^5);

J2_acc=J2_constant*(A1*R+A2.*R);

end