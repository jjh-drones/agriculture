function atm = get_atm(h)
    
    [atm.Ta , atm.a, atm.Pa, atm.rho] = atmosisa(h);
    atm.wind                          = [0 0 0]';
    
end

