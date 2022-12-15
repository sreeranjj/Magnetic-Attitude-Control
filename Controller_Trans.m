function input_u = Controller_Trans(Pre_Rel_X,Ref_Rel_X)
%{
    This Function is for simple regulator input_u
    It will determine difference from Reference and Predicted states
    and give input according to gain Matrix K
    
    u = K*(diff_X)
    diff_X = 6
%}

% Required Natural Frequency   = wn 
% Required Damping Coefficient = z
% Above requirements are achieved from Required Settling Time and Maximum Overshoot   
wn = .001*[.02 .02 .02];
z  = [.9 .9 .9];

% Hence Kd = Derivative Controller
%       Kp = Proportional Controller 
% These Gains multiply with error states(wrt Ref state) to give required input
Kp = wn.^2;
Kd = 2*wn.*z;

% Error State :: which is nothing but how far are we from reference state
diff_X = Ref_Rel_X-Pre_Rel_X;
%{
Kp =  zeros( 1,3);
Kd =  zeros( 1,3);
%}
% ~~~~~~~~~Remember This output is in Relative Frame (wrt Target)~~~~~~~~~~
input_u = Kp'.*diff_X(1:3)+Kd'.*diff_X(4:6) ;

end