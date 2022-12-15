function rho = get_density(h,~)
%This function Gives density depending on current altitude h
global M
for i = 1:35
    h0=M.EXPAtmModel(i,1);
    rho0=M.EXPAtmModel(i,2);
    H=M.EXPAtmModel(i,3);
    if h < M.EXPAtmModel(i+1,1)   
        break;
    end
end
rho=rho0*exp(-(h-h0)/H);
end