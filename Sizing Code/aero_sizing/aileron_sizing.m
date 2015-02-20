function aileron = aileron_sizing(wing,cg,hstab,vstab,mission,assumptions)
%%ASSUMPTIONS
da_max            = assumptions.da_max;             %+- 25 degrees for ailerons
b_end_max_perc    = assumptions.b_end_max_perc;     %maximum external position of aileron
aileronC_ratio    = assumptions.aileronC_ratio;     %percentage of aileron chord with respect to wing
b_beggininga_perc = assumptions.b_beggininga_perc;  %start of aileron withing semi span in percentage
b_begginingb_perc = assumptions.b_begginingb_perc;  %maximum start offset from wing span in percentage

delta      = b_begginingb_perc - b_beggininga_perc;
iterations = 1000;
diff       = delta/iterations;
G = 1;
for i = b_beggininga_perc:diff:b_begginingb_perc,
   [a(G),b(G),c(G)] = aileron_setting(wing.CLalpha,wing.croot*aileronC_ratio,wing.croot,i,b_end_max_perc,wing.S,wing.b,da_max,1.3^2*mission.q_second,mission.rho_second,hstab.S,vstab.S,cg.Ixx,mission.turn_angle); 
%     aileron.t_roll(G) = a;
%     aileron.Pss(G) = b;
%     aileron.bank_angle_till_steady(G) = c;
    size_ail(G) = b_end_max_perc - i;
    ail_start(G) = i;
    G = G + 1;
end
min_t = min(a);
if mission.turn_sec < min_t,
    aileron.t_roll = 0;
    aileron.Pss = 0;
    aileron.bank_angle_till_steady = 0;
else
    aileron.t_roll      = mission.turn_sec;
    aileron.Pss         = interp1(a(:),b(:),aileron.t_roll);
    aileron.bank_angle_till_steady = interp1(a(:),c(:),aileron.t_roll);
    aileron.chord       = aileronC_ratio*wing.croot;
    aileron.chord_perc  = aileronC_ratio;
    aileron_b_perc      = interp1(a(:),size_ail(:),aileron.t_roll);
    aileron.b_perc      = interp1(a(:),size_ail(:),aileron.t_roll);
    aileron.span        = aileron_b_perc*wing.b/2;
    aileron.start       = interp1(a(:),ail_start(:),aileron.t_roll)*wing.b/2;
end
end