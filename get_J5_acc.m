function J5_acc = get_J5_acc(R_vec) 
%This Function Gives J5 Acceleration due to oblateness of earth 

Re = 6378.137; %in Kms
GM = 39860.04418; %in km3/sec2
J5 = -2.27296082869*(10^-7); %J3 acc constant
r=norm(R_vec);
x = R_vec(1);
y = R_vec(2);
z = R_vec(3);
K = z/r;
C1 = -(J5*GM*(Re^5)/(r^7))/8;
C2 =3*(35*K-210*(K^3)+231*(K^5))/r;
C3 = (15-315*(K^2)+945*(K^4)-693*(K^6));

J5_acc = C1*[ C2*x ;C2*y ; C3];

end