function U = conv_to_ECI(input_u,X)
%We need to find Reference frame wrt target, we use its orbit to find [i j k]
%And hence we find Rotation Matrix
R_vec  = X(1:3)';
V_vec = X(4:6)';

R=norm(R_vec);

h_vec=cross(R_vec,V_vec);
h=norm(h_vec);

i=R_vec/R;
k=h_vec/h;
j=cross(k,i);

%Transformation Matrix from Geo to T-Frame = QGT is given by
QGT = [i;j;k];

%Transformation Matrix from Geo to T-Frame = QGT is given by
QTG = QGT';
U = QTG*input_u;
end