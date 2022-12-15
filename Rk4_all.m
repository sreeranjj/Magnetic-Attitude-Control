function [X_next , Rates]= Rk4_all(X,U,t,J_all)

%This Function is ODE Solver using Runge Kutta
%this is Particularly for Actual Plant
%U is constant for one step
    K1 =t*Rates_all(X,U,J_all);
    K2 =t*Rates_all(X+K1/2,U,J_all);
    K3 =t*Rates_all(X+K2/2,U,J_all);
    K4 =t*Rates_all(X+K3,U,J_all);

    X_next = X+(K1+2*K2+2*K3+K4)/6;
    Rates = (K1+2*K2+2*K3+K4)/6;
    
    
end