function J4_acc = get_J4_acc(R_vec)
%This Function Gives J4 Acceleration due to oblateness of earth 
%input :: R_vec = position Column vector
Re = 6378.137; %in Kms
GM = 39860.04418; %in km3/sec2
J4 = -1.61962159137*(10^-6); %J4 acc constant
r=norm(R_vec);

z=R_vec(3);
K=z/r;
C1 = -5*(J4*GM*(Re^4)/(r^6))/8;

C2 = (3-42*(K^2)+63*(K^4))/r;
C3 = -(15-70*(K^2)+63*(K^4))/r;

J4_acc = C1*[ C2 ; C2 ; C3].*R_vec;

end
