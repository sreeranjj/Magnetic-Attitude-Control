function [input_U , Uprev ] = PWPFM(input_u_dir,dt, km, tm, Uon, Uoff, Um ,n,Uprev)

%{ 
Testing
r = 0:1/80:1; 
r = sin(2*r*pi);
%}

r= input_u_dir*ones(n,1); 
l=length(r);
Flag=0;
f=zeros(l,1);
U=zeros(l,1);
U(1,1)=Uprev;
alpha=dt/(dt+tm);

e    = 5*r(1)-U(1);
f(1) = alpha*e;

for i=2:l
    e=5*r(i-1)-U(i-1);
    %z=km*(5*r(i)-Um(i))*(1-exp(-dt/tm))+pz*exp(-dt/tm);
    %pz=z;
    
    %Low Pass Filter
    f(i)=f(i-1)+alpha*(km*e-f(i-1));
    
    %%schmitt Trigger
    if ( Flag ==0)         
          U(i)=0;
      elseif (Flag == 1)
          U(i)=Um;
    end  
       
      if ( abs(f(i)) <= Uoff)
           Flag=0; 
           U(i)=0; 
      elseif( abs(f(i))>= Uon)         
           Flag=1;  
           U(i)=Um*sign(f(i));    
      end
    

end
Uprev = U(l);
input_U=U;
%{
plot(f);
figure;
plot(r,U,'b-o');
%}
end