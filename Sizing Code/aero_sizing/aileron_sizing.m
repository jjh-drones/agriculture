function aileron = aileron_sizing(wing,cg,hstab,vstab,mission,atm)
%%ASSUMPTIONS
da_max            = 25;   %+- 25 degrees for ailerons
b_end_max_perc    = 0.90; %maximum external position of aileron
aileronC_ratio    = 0.2;  %percentage of aileron chord with respect to wing
b_beggininga_perc = 0.6;  %start of aileron withing semi span in percentage
b_begginingb_perc = 0.8;  %maximum start offset from wing span in percentage

delta      = b_begginingb_perc - b_beggininga_perc;
iterations = 1000;
diff       = delta/iterations;
G = 1;
for i = b_beggininga_perc:diff:b_begginingb_perc,
   [a,b,c] = aileron_setting(wing.CLalpha,wing.croot*aileronC_ratio,wing.croot,i,b_end_max_perc,wing.S,wing.b,da_max,mission.q,atm.rho,hstab.S,vstab.S_v,cg.Ixx,mission.turn_angle); 
    aileron.t_roll(G) = a;
    aileron.Pss(G) = b;
    aileron.bank_angle_till_steady(G) = c;
    G = G + 1;
end
end