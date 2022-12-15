function Q_dot = Rot_Kinematics(omega,Q)
%this function converts body rates ,omega, and current quarterion state
%,Q,to current quarterion rates Q_dot
%Omega is angular velocity of the body frame wrt Body Frame

%Column Vector Omega=[ w1;
%                      w2;
%                      w3]
%Quarternion must be of form:
%Column Vector Q = [  q1;
%                     q2;
%                     q3;
%                     q4] where q4 is scaler

Q_dot = zeros(4,1);

Q_dot(1:3,1) = (Q(4,1)*omega-cross(omega,Q(1:3,1)))/2;

Q_dot(4,1)=-(omega')*Q(1:3,1)/2;

end