function pos = aero_pos(assumptions,wing)

x0           = [0.3;0.6;0.2];
estFcn       = @(x) syst(x,wing,assumptions);
opt          = fsolve(estFcn,x0);

cg              = aero_balance(assumptions.Df,assumptions.Lf_nose,opt(2),assumptions.Lf_rear,assumptions.battery_m);
pos.xAC         = opt(1);
pos.xCG         = cg.x;
pos.Lf_body     = opt(2);

end

