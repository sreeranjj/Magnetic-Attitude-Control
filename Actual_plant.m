function [ XT_Next, XC_Next, RatesT, RatesC] = Actual_plant(Tr_XT,Tr_XC,Rot_XT,Rot_XC,U_T,U_C,T,J_all)
% This plant will be using State Vector of Target And Chaser Satellites in ECI
% AND input from Controller to give next State Vector IN ECI

% Input Arguments   ::
%   Tr_XT       = State Vector of Target in ECI
%   Tr_XC       = State Vector of Chaser in ECI
%   Tr_U_Chaser = Input vector of Forces ( 3 X 1); 
%   rot_XT      = Rotation State of target
%   rot_X_rel   = Relative Rotation of Chaser wrt hill Frame
%   rot_input_U = Input vector of Torques ( 3 X 1)


%Now ,Below, getting Next XT and XC using RK4 solver  
XT =  [Tr_XT;Rot_XT];
XC =  [Tr_XC;Rot_XC];

[XT_Next,RatesT]  = Rk4_all(XT,U_T,T,J_all);
[XC_Next,RatesC]  = Rk4_all(XC,U_C,T,J_all);

end