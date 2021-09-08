function traj = SSPP_1D_vehicle_simu_motion_time_evol(mu_of_x_grid, J_of_x_grid, fct_sysDyn)
% simulate the motion (i.e. evolution) in time of the vehicle given a policy (e.g. the optimal policy)
% Input: 
% Output: k is the last simulated discrete time when the state variables get very close to the terminal state t (i.e. state variable xterm) 

loadVarsFromBaseWS xterm w_grid x1_min x1_max x2_min x2_max  delta_x1 delta_x2  x1_grid x2_grid  deltat_simu 

%ini: choose departing point
xk = [0 0]; %[m m/s]

%assume wk fixed: at the value with highest proba of occurrence
wk = w_grid(2);

%choose nr of sampling steps
Nk = round(10/deltat_simu); %absolute value in sec 

%%% store traj: ini: malloc
traj.x1 = nan(1,Nk);
traj.x2 = traj.x1;
traj.u  = traj.x1;

for k=0:Nk-1 %time index
    id_k = k+1; %index associated to k on k_grid
    
    %% specify control action
    %{
    %var1 [control action]: map xk to a fixed iprime iotbat use mu=mu(iprime), where iprime \in iprime_grid
    i = SSPP_1D_vehicle_xk2i(xk); %effect: given xk, map it to the closest point i \in i_grid       
    iprime = i; %i == iprime since same (convention) numbers are used; however idx_i might be different than idx_iprime, where idx_i corresp to index on i_grid, and idx_iprime corresp to index on iprime_grid   
    uk = mu_opt(iprime_grid == iprime) ;
    %}
    
    %{-
    %var2 [control action]: keep xk as-is/unchanged and compute mu=mu(xk) by interpolation on mu=mu(x_grid)    
    %Step0: store sys dyn state value
    xk_temp = xk;
    
    %Step1 [prereq interp2(.)]: ensure xk stays within x_grid: redundant here since function SSPP_1D_vehicle_sysDyn(.) ensures this too     
    xk(1) = max(x1_min,min(x1_max,xk(1))) ; %V&V: xk(1)=-10; xk(1)=10;  
    xk(2) = max(x2_min,min(x2_max,xk(2))) ; %V&V: xk(2)=-10; xk(2)=10;  
    
    %Step2: handle x-area around t given that mu_opt(t)=nan (i.e. undefined): apply saturation in that area: keep state on the boundary  
    expr_west  = 'xterm(1)-delta_x1 < xk(1) && xk(1) <= xterm(1)'; %left of t 
    expr_east  = 'xterm(1) < xk(1) && xk(1) < xterm(1)+delta_x1';  %right of t
    expr_south = 'xterm(2)-delta_x2 < xk(2) && xk(2) <= xterm(2)'; %below of t
    expr_north = 'xterm(2) < xk(2) && xk(2) < xterm(2)+delta_x2';  %above of t
     expr_x1_west  = 'max(x1_min, xterm(1)-delta_x1)'; 
     expr_x1_east  = 'min(x1_max, xterm(1)+delta_x1)';
     expr_x2_south = 'max(x2_min, xterm(2)-delta_x2)';
     expr_x2_north = 'min(x2_max, xterm(2)+delta_x2)';
     
    if eval(expr_west) && eval(expr_north)      %Quadrant northwest: V&V:  xk = xterm + .5*[-1  1].*[delta_x1 delta_x2]       
        xk(1) = eval(expr_x1_west); 
        xk(2) = eval(expr_x2_north);
    elseif eval(expr_east) && eval(expr_north)  %Quadrant northeast: V&V:  xk = xterm + .5*[1  1].*[delta_x1 delta_x2]       
        xk(1) = eval(expr_x1_east); 
        xk(2) = eval(expr_x2_north);
    elseif eval(expr_east) && eval(expr_south)  %Quadrant southeast: V&V:  xk = xterm + .5*[1  -1].*[delta_x1 delta_x2]            
        xk(1) = eval(expr_x1_east);
        xk(2) = eval(expr_x2_south);
    elseif eval(expr_west) && eval(expr_south)  %Quadrant southwest: V&V:  xk = xterm + .5*[-1  -1].*[delta_x1 delta_x2]            
        xk(1) = eval(expr_x1_west);
        xk(2) = eval(expr_x2_south);
    end %if
        
    %Step3 [use interp2(.)] compute mu=mu(xk)
    uk = interp2(x1_grid,x2_grid,mu_of_x_grid, xk(1),xk(2), 'linear') ; %interp2 and surf have the same arguments (x,y,M(y,x))    
    Jk = interp2(x1_grid,x2_grid, J_of_x_grid, xk(1),xk(2), 'linear') ; %interp2 and surf have the same arguments (x,y,M(y,x))    
    
    %Step4. restore xk to sys dyn state value
    xk = xk_temp;
    %}
    
    %% store traj
    traj.x1(id_k) = xk(1);
    traj.x2(id_k) = xk(2);
    traj.u(id_k)  = uk;
    traj.J(id_k)  = Jk;
    
    %% check if xk is very close to t: decision to stop this for-loop
    if xterm(1)-delta_x1 < xk(1) && xk(1) < xterm(1)+delta_x1 && ...
       xterm(2)-delta_x2 < xk(2) && xk(2) < xterm(2)+delta_x2
        
        %trim accordingly to the right size
        traj.x1 = traj.x1(1:id_k);
        traj.x2 = traj.x2(1:id_k);
        traj.u  = traj.u(1:id_k);
        traj.J  = traj.J(1:id_k);
        
        break;
    end
    %otherwise continue hereafter
    
    %% vehicle dyna at next step/stage
    xkp1 = fct_sysDyn(xk,wk,uk, deltat_simu) ;
        
    %% update
    xk = xkp1;
end %for k=

end %function