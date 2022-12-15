function Kep_Acc=keplerian_acc(R)
%Function returns Keplerian Accerelation In ECI frame
%Here R is Position Vector of SpaceCraft in Earth's Frame (ECI)
%R is column Vector = [Rx ; Ry ; Rz];

GM=3.98*(10^5);
r=norm(R);

Kep_Acc = -GM*R/(r^3);

end