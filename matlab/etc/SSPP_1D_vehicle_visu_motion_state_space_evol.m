function hs1 = SSPP_1D_vehicle_visu_motion_state_space_evol(traj, info_policy)
% plot evolution in state-space of trajectory above

loadVarsFromBaseWS expr_plot_surf_mu_opt expr_plot_term_state_t expr_title driver_type  x1_grid x2_grid  mu_opt_of_x_grid J_opt_of_x_grid  t xterm  delta_x1 delta_x2 delta_u delta_w deltat  deltat_simu 

f = figure;
f.Position = [100 100 2*560 420];

hs1(1) = subplot(1,2,1);
eval(expr_plot_surf_mu_opt); hold on;
hp1 = plot3(traj.x1,traj.x2,traj.u, 'o-', 'color',.4*[1 1 1], 'linewidth',3, 'displayname',['sys traj using ',info_policy,'\newline{}\Delta{}t_{simu}=',num2str(deltat_simu,'%.2f'),'s']); grid on;
hp2 = plot3(traj.x1(1),traj.x2(1),traj.u(1),'m^', 'linewidth',3, 'displayname','departing state'); grid on;
eval(expr_plot_term_state_t);
legend([hp1 hp2 hl1], 'location','southwest');
eval(expr_title);
axis tight;
view(2);

shading interp; %shading(gca,'interp')
set(s1,'edgecolor',[0 0 0.4],'meshstyle','both','linewidth',.15); %set(s1,'LineStyle','none');
set(s1,'FaceAlpha',0.7); 

ax=gca; ax.BoxStyle = 'full';
%rotate3d on;

hs1(2) = subplot(1,2,2);
s2=surf(x1_grid,x2_grid,J_opt_of_x_grid); grid on; box on; xlabel('x_1 [m]'); ylabel('x_2 [m/s]'); zlabel('Cost  $$J_\mu(x)$$ [sec]', 'interpreter','latex'); hold on;
hp1 = plot3(traj.x1,traj.x2,traj.J, 'o-', 'color',.4*[1 1 1], 'linewidth',3, 'displayname',['sys traj using ',info_policy,'\newline{}\Delta{}t_{simu}=',num2str(deltat_simu,'%.2f'),'s']); grid on;
hp2 = plot3(traj.x1(1),traj.x2(1),traj.J(1),'m^', 'linewidth',3, 'displayname','departing state'); grid on;
eval(expr_plot_term_state_t);
legend([hp1 hp2 hl1], 'location','southwest');
view(2);
axis tight;  
 shading interp; %shading(gca,'interp')
 set(s2,'edgecolor',[0 0 0.4],'meshstyle','both','linewidth',.15); %set(s1,'LineStyle','none');
ax=gca; ax.BoxStyle = 'full';

drawnow;

%linkprop(mrv(hs1),'CameraPosition'); %alternative code: linkaxes(mrv(hs2),'x');   
rotate3d on;


end %function