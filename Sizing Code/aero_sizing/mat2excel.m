function mat2excel(wing,name)

Max_lift = wing.L_max;
segments = 300;

b_chunks = (wing.b/2)/segments;
G = 1;
for i = 0:b_chunks:wing.b/2,
    C_straight = wing.croot;
    C_elliptic = (4*wing.S/(pi*wing.b))*(1-(2*i/wing.b)^2)^0.5;
    L_ellip    = ((C_elliptic*b_chunks)/(wing.S))*Max_lift;
    L_straight = C_straight*b_chunks*Max_lift/(wing.S);
    Lwing(G,2)   = (L_ellip+L_straight)/2;
    Lwing(G,1)   = i;
    G = G + 1;
end
Header = {'Position From Root','Lift Force (N)'};
xlswrite(['OUTPUTS\',name,'_output.xls'],Header,'Lift_Distr','A1');
xlswrite(['OUTPUTS\',name,'_output.xls'],Lwing,'Lift_Distr','A2');

end