function hs2 = SSPP_1D_vehicle_visu_motion_time_evol(traj, info_policy)
% visualize the motion (i.e. evolution) in time of the vehicle given a policy (e.g. the optimal policy)

loadVarsFromBaseWS expr_title driver_type  delta_x1 delta_x2 delta_u delta_w deltat  u_grid  deltat_simu  

%choose
N_fig = 4;

k_grid = 0:length(traj.u)-1;

figure;

idxSbp=1; hs2(idxSbp) = subplot(N_fig,1,idxSbp);
plot(k_grid,traj.J,'bo', 'linewidth',3); grid on;
ylabel('J_\mu(x(t)) [sec]');
eval(expr_title);

idxSbp=idxSbp+1; hs2(idxSbp) = subplot(N_fig,1,idxSbp);
plot(k_grid,traj.u,'bo', 'linewidth',3); grid on;
ylabel(['Control action [N]\newline{}(using ',info_policy,')']);
ylim([min(u_grid) max(u_grid)] + .05*(max(u_grid)-min(u_grid))*[-1 1] ); 

idxSbp=idxSbp+1; hs2(idxSbp) = subplot(N_fig,1,idxSbp);
plot(k_grid,traj.x1,'bo', 'linewidth',3); hold on; grid on;
ylabel('Position x_1 [m]');

idxSbp=idxSbp+1; hs2(idxSbp) = subplot(N_fig,1,idxSbp);
plot(k_grid,traj.x2,'bo', 'linewidth',3); hold on; grid on;
ylabel('Velocity x_2 [m/s]');
xlabel(['Time index k [-];   \Delta{}t_{simu}=',num2str(deltat_simu,'%.2f'),'s']); 

%link x-axes
%linkprop(mrv(hs2),'XLim'); %zoom/pan simultaneously on the x-axes; alternative code: linkaxes(mrv(hs2),'x');   
xlim([min(k_grid) max(k_grid)]);

end %function
