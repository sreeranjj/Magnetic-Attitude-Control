function Rates=Rates_all(X,U,J_all)

% This Function Will return the slope of state variables at that state 
% Currently only Keplerian Accerelations are Added
% This Function works for Both Target and Chaser States  

    J    = J_all(1:3,1:3); %inertia Tensor
    invJ = J_all(4:6,1:3); %Inverse Inertia Tensor
    
    R=X(1:3,1);
    V=X(4:6,1);
    q=X(7:10,1);
    w=X(11:13,1);
    
    Tr_U = U(1:3,1);
    Ro_U = U(4:6,1);
    
%% Translation    
%Acceleration Calculations :

     %
    J2_Acc = get_J2_acc(R);
    J3_Acc = get_J3_acc(R);
    J4_Acc = get_J4_acc(R);
    J5_Acc = get_J5_acc(R);
    J6_Acc = get_J6_acc(R);
    %}
    Kep_acc  = keplerian_acc(R);
    Atm_Drag = get_atm_drag(X);
    A= Kep_acc + Tr_U + Atm_Drag+J2_Acc+J6_Acc+J3_Acc+J4_Acc+J5_Acc;
    

%% Rotation
    
 %q_dot = Kinematic equation ...
 q_dot = Rot_Kinematics(w,q);
 
 %w_dot = Dynamic equation ...
 w_dot = invJ*(-cross(w,J*w)) + Ro_U; 
 
%% return.... 
 Rates =[ V ; A ; q_dot ; w_dot];
 
end