function J3_acc = get_J3_acc(R_vec) 
%This Function Gives J3 Acceleration due to oblateness of earth 

Re = 6378.137; %in Kms
GM = 39860.04418; %in km3/sec2
J3 = -2.53265648533*(10^-6); %J3 acc constant
r=norm(R_vec);
x=R_vec(1);
y=R_vec(2);
z=R_vec(3);

C1 = -(J3*GM*(Re^3)/(r^5))/2;
C2 = 5*(7*((z/r)^3)-3*z/r)/r;
C3 = 3*(10*((z/r)^2)-35*((z/r)^4)/3-1);

J3_acc = C1*[ C2*x ;C2*y ; C3];

end