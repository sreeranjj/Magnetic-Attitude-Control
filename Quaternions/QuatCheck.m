function Check = QuatCheck(Q)
%this func will check that if given input Q satisfies the single
%constraint 
%
%Quarternion must be of form:
%Column Vector Q = [  q1;
%                     q2;
%                     q3;
%                     q4]
if (Q'*Q)==1
    Check=true;
else
    Check=false;
end
end