function Con_output = Controller_Rot(X_des,X_curr)
%This function gives output torque U based on error from desired (obvio!!!)

q_Diff = qmult(X_curr(1:4),qinv(X_des(1:4)));
w_Diff =-X_des(5:7)+X_curr(5:7);
kp=50;
kd=50;

U = -kp*q_Diff(1:3)-kd*w_Diff;
Con_output=[U',0 ; q_Diff'];
end