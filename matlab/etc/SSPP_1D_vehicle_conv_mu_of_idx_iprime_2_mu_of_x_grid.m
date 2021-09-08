function mu_of_x_grid = SSPP_1D_vehicle_conv_mu_of_idx_iprime_2_mu_of_x_grid(mu_of_idx_iprime)
% mu_of_x_grid = mu_of_x_grid

loadVarsFromBaseWS  x1_grid x2_grid  iprime_grid 

%ini: malloc
m = length(x2_grid); %dim1 associated to y; not. (y,m) cf doc interp2: same arguments compatible with doc surf  
n = length(x1_grid); %dim2 associated to x; not. (x,n) cf doc interp2: same arguments compatible with doc surf  
mu_of_x_grid = nan(m,n);

for idx_iprime=1:length(iprime_grid)
    iprime = iprime_grid(idx_iprime) ;
    
    xk = SSPP_1D_vehicle_i2xk(iprime); %since iprime_grid \subset i_grid, we can effectively make use of function SSPP_1D_vehicle_i2xk(.) which needs an argument belonging to i_grid  
    idx_x_grid = SSPP_1D_vehicle_xk_2_idx_x_grid(xk) ; 
    
    %conseq
    idx_x1_grid = idx_x_grid(1);
    idx_x2_grid = idx_x_grid(2);
    
    mu_of_x_grid(idx_x2_grid, idx_x1_grid) = mu_of_idx_iprime(idx_iprime);
end %for 

end %function
