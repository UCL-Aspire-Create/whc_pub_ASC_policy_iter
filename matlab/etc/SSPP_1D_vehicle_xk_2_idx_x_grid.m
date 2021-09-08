function idx_x_grid = SSPP_1D_vehicle_xk_2_idx_x_grid(xk) 
% idx_x_grid =def= [idx_x1_grid idx_x2_grid]
% Ex:  eps_temp=1e3*eps; SSPP_1D_vehicle_xk_2_idx_x_grid([x1_grid(3)+eps_temp x2_grid(5)-eps_temp]) 


loadVarsFromBaseWS x1_grid x2_grid M

i = SSPP_1D_vehicle_xk2i(xk);
idx_x_grid = [M(i,1) M(i,2)];

end