function J6_acc = get_J6_acc(R_vec) 
%This Function Gives J6 Acceleration due to oblateness of earth 
%input :: R_vec = position Column vector
Re = 6378.137; %in Kms
GM = 39860.04418; %in km3/sec2
J6 = 5.40681239107*(10^-7); %J3 acc constant
r=norm(R_vec);
% x = R_vec(1);
% y = R_vec(2);
z = R_vec(3);
K = z/r;
C1 = (J6*GM*(Re^6)/(r^8))/16;
C2 =(35-945*(K^2)+3465*(K^4)-3003*(K^6))/r;
C3 = (2205*(K^2)-4851*(K^4)+3003*(K^6)-315)/r;

J6_acc = C1*[ C2 ;C2 ; C3].*R_vec;

end