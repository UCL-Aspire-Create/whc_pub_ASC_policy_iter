%% This is a STOCH SHORTEST PATH PB within scope of Prop7.2.1(a)-(e) and Value Iteration eq(7.16) and Policy Iteration eq(7.18)-(7.19) 
% whc advancing in strainght line (1D in Cartesian space) cf eqs described in "function xkp1 = SSPP_1D_whc_sysDyn(xk,wk,uk)": compute policy associated to time-optimal motion to reach a given end point 
clear all
%close all

%% set properties of this code
%choose
doSaveIntermResu     = false; %do save intermediate results (in case of long computations) 
doPrintFigures       = true;
doDisplayMsgOnScreen = true;
platform             = 1; %1='laptop',2='myriad' 


%% inteface with Myriad server: Cisco AnyConnect, apoi virtualbox > Ubuntu > $ssh -X rejacst@myriad.rc.ucl.ac.uk   
%Step1. test if launch/run of code file is feasible (otherwise next cdes useless due to permissions forbidding matlab so save/store files in current folder): $cd ~/Scratch/MATLAB_ctdr/; module load xorg-utils/X11R7.7; module load matlab/full/r2018b/9.5; matlab -nosplash -nodesktop -nodisplay < ./1D_vehicle_standalone/1D_vehicle/PP_1D_vehicle_stochShortPathPb_v0.m & ; top -u rejacst    
%Step2: launch via qsub: $cd ~; qsub qsub_script_run_matlab_principal.sh; qstat; #qdel <job-ID-shown-on-> 

if isequal(platform,2)  %1='laptop',2='UCL-HPC-myriad'  
    %% add folders to path
    path_temp = path;
    path_temp = path('./1D_vehicle_standalone/1D_vehicle',path_temp); %overwrite
    path_temp = path('./1D_vehicle_standalone/matlab_Dropbox',path_temp); %overwrite
    path_temp = path('./1D_vehicle_standalone/other',path_temp); %overwrite
end

%% Physical parameters
P = whc_parameters1() ;

%choose accordingly
kp = 0.1; % gain low level P-controller; artificial (non-physical) parameter since a Tranjectory Gen block is missing in the sys dyn model 

%interface
vmax = P.Profile_vmax; %[m/s] low-level safety block acts beyond this value bypassing any control
vmin = P.Profile_vmin; %[m/s] idem above
SSPP_1D_vehicle_sysDyn = @SSPP_1D_whc_sysDyn;

%choose: absolute values
u_min = vmin; %[m/s] u = vdk [m/s]; 
u_max = vmax; %[m/s] u = vdk [m/s];

%choose: physical bumpers on the Cartesian space: physically they corresp e.g. to a wall   
x1_bmp_min = -.001; %[m]   
x1_bmp_max = P.US_draymax + 0.1; %[m] slightly more than what the US are set to sense  


%% define grids
%choose
deltat = .5; %[sec]

%conseq
sigma1 = 1+deltat*(P.xi1-P.xi4*kp+P.xi2-P.xi5*kp);
sigma2 = deltat*kp*(P.xi4+P.xi5);

%choose (delta_u,delta_w) s.t. their ratio (namely delta_u/delta_w or delta_w/delta_u) is an integer => defining delta_x2 to be a multiple of min(delta_u,delta_w) is meaningful    
delta_u = .15; %[N=kg*/s^2]
delta_w = delta_u; % 

%conseq
delta_x2 = sigma1*min(delta_u,delta_w)*sigma2 ; %[m/s] min(delta_u,delta_w)*sigma2 is the smallest increment of the 2nd term "sigma2*(vdk+vuk)" of eq(2); assuming vk is on the grid exactly at vk=min(delta_u,delta_w)*sigma2 then the 1st term of eq(2) is "sigma1*vk" (with sigma1<1) giving an even lower value i.e. step increment   
delta_x1 = delta_x2*deltat ; %[m]

%def grid bounds: make sure they belong-to/can-be-placed-on the grid 
x1_min =  ceil(x1_bmp_min/delta_x1)*delta_x1; %[m]   
x1_max = floor(x1_bmp_max/delta_x1)*delta_x1; %[m] 

%conseq: place vmin,vmax on x2_grid
vmin =  ceil(vmin/delta_x2)*delta_x2 ;
vmax = floor(vmax/delta_x2)*delta_x2 ;

%conseq: def x2_grid bounds: bounds on x2_grid are a relation betw x1_grid bounds and (vmin,vmax)
x2_min = max(vmin, (x1_min-x1_max)/deltat); %imposing x1 to always stay inside x1_grid results in a condition on x2 to stay within some bounds: eq1 from sys dyna becomes (x1kp1-x1k)/deltat = x2k, and we know that   x1_min <= x1kp1 <= x1_max   and   -x1_max <= -x1k <= -x1_min ;  max(qtt1,qtt2) stands for requesting 2 conditions to be fulfilled simultaneously (usage of AND between: (i) avoid going into sat(.) region; (ii) avoid x1 to go beyond x1_grid )    
x2_max = min(vmax, (x1_max-x1_min)/deltat); % idem above

%conseq: place on grids
u_min =  ceil(u_min/delta_u)*delta_u; %[N=kg*/s^2]  
u_max = floor(u_max/delta_u)*delta_u; %[N=kg*/s^2]  

%[finally]conseq: def grids
x1_grid = x1_min:delta_x1:x1_max; %physical meaning of the bounded x1_grid: analogy/think of a toy vehicle circuit surrounded by protective pads that ensure the toy vehicle stays with those bounds without jumping off  
x2_grid = x2_min:delta_x2:x2_max; %physical meaning of the bounded x2_grid: search "safety block" in this file 
u_grid  = u_min:delta_u:u_max;    %physical meaning of the bounded u_grid: torque = km * ele_current [SpHuVi2005,p208] and there is a limit on max ele_current    

%% driver's demand (or intention) $v_\text{d}$ is modeled as a stochastic process

%{-
%var1: blind driver
driver_type = 'blind driver';
w_grid = u_grid;
Pw = ones(size(w_grid))/length(w_grid) ; %uniform distribution  
%}

%{
%var2: naughty child: make sure SC has the capacity to counteract the user's demand at all times: will mirror values around vmin since vmin<vmax     
driver_type = 'naughty child driver';
w_grid = u_grid( round( sort([fzero_curve(1:length(u_grid),u_grid,[0 u_grid(1)])  fzero_curve(1:length(u_grid),u_grid,[0 u_grid(2)]) ]) )) ; %flip/associate largest negative u_grid values onto positive ones; round(.) has the role of converting double to int because indices are expected to be int 
Pw     = [.2 .8];  
%}

%{
%var3: expert driver: make sure SC has the capacity to counteract the user's demand at all times: will mirror values around vmin since vmin<vmax     
driver_type = 'expert driver';
w_grid = delta_u:delta_u:u_grid( round( fzero_curve(1:length(u_grid),u_grid,[0 u_grid(1)]))) ; %flip/associate largest negative u_grid values onto positive ones; round(.) has the role of converting double to int because indices are expected to be int 
Pw     = ones(size(w_grid))/length(w_grid) ; %uniform distribution   
%}

%V&V
epsTol = 1e-14;
if abs(sum(Pw) - 1) > epsTol
    error('ctdr: Please check Pw');
end

%% define i_grid: states i associated to each point on (x1_grid,x2_grid)
% define all combinations ito x1_grid x x2_grid and store them in M
M = perm_wRep_Nlen([length(x1_grid) length(x2_grid)]) ; %pairs (M(i,1),M(i,2)) = (idx_x1_grid,idx_x2_grid) ; states i are the rows numbers of matr M; the associated content on each row represents indices on x_grid:=(x1_grid,x2_grid);  

%conseq  
i_grid = 1:size(M,1); %this def of i: (i) includes the terminal state t: be careful that def of i,j within sum_i and sum_j in Prop7.2.1(a)-(e) and Value Iteration eq(7.16) and Policy Iteration eq(7.18)-(7.19), do NOT include the terminal state t    
%                                     (ii) is s.t. idx_i =same= i, whereas for iprime_grid only some idx_iprime =sometimes= iprime           
j_grid = i_grid; %this def of j includes the terminal state t: be careful that def of i,j within sum_i and sum_j in Prop7.2.1(a)-(e) and Value Iteration eq(7.16) and Policy Iteration eq(7.18)-(7.19), do NOT include the terminal state t   

%% define terminal state t = (xf,dotxf) \in i_grid, but t \notin iprime_grid
doPlot = 0; %choose

%choose
xf    = x1_max; %[m] sat(.) iot avoid value outside grid  
dotxf = max(x2_min,min(x2_max,  0)); %[m/s] 0 = standstill; sat(.) iot avoid value outside grid

%conseq
t = SSPP_1D_vehicle_xk2i([xf dotxf]) ;

if isempty(t)
    error('ctdr: Please check t');
end %if isempty(.) 

%conseq
iprime_grid = setdiff(i_grid,t) ; %this def of i does NOT include the terminal state t, and thus can be used in place of i,j within sum_i and sum_j in Prop7.2.1(a)-(d) and Value Iteration eq(7.16) and Policy Iteration eq(7.18)-(7.19)    
% ^iprime_grid consists of states selected among i_grid and as such can be used in expressions M(iprime,:)  
jprime_grid = iprime_grid ; %this def of i does NOT include the terminal state t, and thus can be used in place of i,j within sum_i and sum_j in Prop7.2.1(a)-(d) and Value Iteration eq(7.16) and Policy Iteration eq(7.18)-(7.19)    

%conseq
iprimeAndt_grid = union(iprime_grid,t); % union implicitly sorts ascending
jprimeAndt_grid = iprimeAndt_grid;
      
%% misc
%%% print/show statistics on screen
%choose
perc_show_stat = 5; %[percentage] \in (0,100) show statistics with evolution algo in multiples of this qtt

SS_1D_vehicle_misc

if doDisplayMsgOnScreen, tic, end

%% def transition proba matrix p_ij(u)
%%% var1: use code to generate p_ij_u and p_ij_u_sparse
SS_gen_transProbaMatr_v0

%{-
%[testing-purpose only] visualize transitions starting from an individual state 
%visualTrans([x1_grid(end-1) x2_grid(end)], 2);
%i=3; SSPP_1D_vehicle_visualTrans(SSPP_1D_vehicle_i2xk(i), 5);  
%}

if doSaveIntermResu  % #1
    save(strcat('results_1D_vehicle.',datestr(now,'yyyy.mmmm.dd.HH.MM.SS.FFF'),'.mat')) ;
end %if
%load <results_1D_vehicle...mat>   %proceed with computations from here onwards


%% start-from/define a feasible stationary policy muUk (complies w Constraint Satisfaction) 
SSPP_1D_vehicle_displayMsgOnScreen('Generating ini policy');

%ini: memory alloc
muUk = nan(length(iprime_grid),1) ; %muUk=muUk(idx_iprime_grid)

%{-
%var3: Greedy approach to finding a feasible iteration 
muUk = findAFeasibleStatioPolicy() ;
%}

if any(isnan(muUk))
    error('ctdr: Greedy not successful in generating ini policy');
end

if doSaveIntermResu  % #2
    save(strcat('results_1D_vehicle.',datestr(now,'yyyy.mmmm.dd.HH.MM.SS.FFF'),'.mat'));
end %if
%load <results_1D_vehicle...mat>   %proceed with computations from here onwards


%% [check ini policy] identify the largest states space formed of states from which the terminal state t can be reached in a finite nr of transitions, namely in any k<=Ntr transitions 
SSPP_1D_vehicle_displayMsgOnScreen("\nChecking validity of ini policy\n");

Ntr =100; %number of transitions 
[~,idx_iprimeAndt_cum] = computeTPMandSpaceReachTot_mu(muUk, Ntr) ;

if length(idx_iprimeAndt_cum) == length(iprimeAndt_grid) &&  all( iprimeAndt_grid == iprimeAndt_grid(idx_iprimeAndt_cum) ) % "&&" is logical operation with short-circuit: see https://uk.mathworks.com/help/matlab/ref/logicaloperatorsshortcircuit.html?searchHighlight=%26%26&s_tid=srchtitle#responsive_offcanvas 
    SSPP_1D_vehicle_displayMsgOnScreen('\n  Successful checked: feasible ini policy\n');
else
    %% [WIP-attempt] adjust the reachable states space for which we seek an optimal mu=mu(idx_iprime)
    iprime_grid = setdiff( iprimeAndt_grid(idx_iprimeAndt_cum), t) ; %overwrite iprime_grid by shrinking it to a subset of iprimeAndt_grid that: (i) accounts only for indices idx_iprimeAndt_cum, (ii) excludes terminal state t
    jprime_grid = iprime_grid; 
end

if doSaveIntermResu  % #3
    save(strcat('results_1D_vehicle.',datestr(now,'yyyy.mmmm.dd.HH.MM.SS.FFF'),'.mat'));
end %if
%load <results_1D_vehicle...mat>   %proceed with computations from here onwards


%% policy iteration algo
%choose
Niter = 5;

%def
k_grid = 0:Niter-1; %nr of iterations to run policy iteration algo (convergence is not guaranteed to happen when reaching k_grid(end), however we'll use visually inspect that later on) 

%ini
metric_JLmuUk = nan(length(k_grid),1);

for id_k=1:length(k_grid)
    k=k_grid(id_k); 
    
    %% Step 2) policy evaluation
    %%% Least-Squares A*J=b for Policy Evaluation
    SSPP_1D_vehicle_displayMsgOnScreen('\nk=%d: Policy Evaluation step',k);
    JLmuUk = policyEvaluation_LS_StochShortPathPb(muUk) ; %JLmuUk=JLmuUk(idx_iprime_grid)  
    
    %store
    metric_JLmuUk(id_k) = norm(JLmuUk,1)/length(iprime_grid) ; %interpretation:  average time per state \in iprime_grid to reaching the terminal state t; departing from i \in iprime_grid, in average (because (i) we have multiple traj depending on occurring w; (ii) we lumped together the effect of all departing states when forming the sum) we spent this amount of time towards reaching the terminal state t 
        
    %% Step 3) policy improvement
    SSPP_1D_vehicle_displayMsgOnScreen('\n     Policy Improvement step');
    muUkp1 = policyImprovement_StochShortPathPb(JLmuUk) ;
        
    %update: prepare next iteration
    muUk = muUkp1;
    
end %for k=

J_opt = JLmuUk ; %J_opt = J_opt(idx_iprime) is the total cost associated to using mu_opt; most probably this is the same as J*(iprime) from [B2005Bertsekas,Prox7.2.1(b)]  
mu_opt = muUk ;  %mu_opt = mu_opt(idx_iprime)

if doSaveIntermResu  % #4
    save(strcat('results_1D_vehicle.',datestr(now,'yyyy.mmmm.dd.HH.MM.SS.FFF'),'.mat'));
end %if
%load <results_1D_vehicle...mat>   %proceed with computations from here onwards

%% misc
SSPP_1D_vehicle_displayMsgOnScreen('\nRunning hereabove code necessitated this amount of time:\n\t ');
if doDisplayMsgOnScreen, toc, end

if doPrintFigures    
    %% visualise convergence of policy iteration algo
    figure;
    plot(k_grid, metric_JLmuUk, 'bo-'); grid on;
    expr_title = "title(['1D Wheelchair: Policy Iteration: ',driver_type,'; grid: $$\Delta{}x_1=',num2str(delta_x1,'%.4f'),'$$m, $$\Delta{}x_2=',num2str(delta_x2,'%.4f'),'$$m/s, $$\Delta{}t=',num2str(deltat,'%.2f'),'$$s, $$\Delta{}u=',num2str(delta_u,'%.2f'),'$$N, $$\Delta{}w=',num2str(delta_w,'%.2f'),'$$N'],  'interpreter','latex'); ";
    eval(expr_title);
    xlabel('Iteration k [-]');
    ylabel(['|| J_{\mu^k}(i^\prime) ||_1/N, with i^\prime accounting for N=',num2str(length(iprime_grid),'%d'),' states \newline{} (study convergence of J_{\mu^k} [sec])'], 'interpreter','tex');
    
    
    %% prepare iotbat visualise the Optimal Policy and associated Cost 
    mu_opt_of_x_grid = SSPP_1D_vehicle_conv_mu_of_idx_iprime_2_mu_of_x_grid(mu_opt) ;
    expr_plot_surf_mu_opt = "s1=surf(x1_grid,x2_grid,mu_opt_of_x_grid); grid on; box on; xlabel('x_1 [m]'); ylabel('x_2 [m/s]'); zlabel('Optimal policy  $$\mu(x)$$ [N]', 'interpreter','latex') ;" ;
    
    xterm = SSPP_1D_vehicle_i2xk(t);
    expr_plot_term_state_t = "hl1 = plot3(xterm(1),xterm(2),min(mu_opt_of_x_grid(:)),'rd', 'linewidth',3, 'displayname','terminal state t') ;";
    
    J_opt_of_x_grid = SSPP_1D_vehicle_conv_mu_of_idx_iprime_2_mu_of_x_grid(J_opt) ;    
    
    
    %% simulate & visualize the motion (i.e. evolution) in time of the vehicle given a policy (e.g. the optimal policy)
    %choose
    deltat_simu = 0.1; %[sec]
    
    J_of_x_grid = SSPP_1D_vehicle_conv_mu_of_idx_iprime_2_mu_of_x_grid(J_opt) ;
    
    traj = SSPP_1D_vehicle_simu_motion_time_evol(mu_opt_of_x_grid, J_of_x_grid, @SSPP_1D_whc_sysDyn);
    hs2  = SSPP_1D_vehicle_visu_motion_time_evol(traj,'optimal policy');
    %linkprop(mrv(hs2),'XLim'); %zoom/pan simultaneously on the x-axes
    linkaxes(mrv(hs2),'x');
    
    hs1  = SSPP_1D_vehicle_visu_motion_state_space_evol(traj,'optimal policy');
    linkprop(mrv(hs1),'CameraPosition');
    
end %if doPrintFigures


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%% Utility functions %%%%%%%%%%%%
function y = g_i_mui(idx_i)
% compute g(i,mu(i)): see def in [B2005Bertsekas, attached file "Prop7.3.1(c) appl to ex1.3.2_invent ctrl pb.pdf"]
%  see example code in PP_exo1_3_2_Bertsekas_inventCtrlPb_discPb.m : "energy"-optimal pb    
%  see example code in PP_exo7_2_2_Bertsekas_spiderAndFly_stochShortPathPb.m : time-optimal pb    

loadVarsFromBaseWS t i_grid x1_min deltat

y = deltat; %time-optimal pb

%{-
%% include soft constraint implemented as penalty in the objective fct: https://en.wikipedia.org/wiki/Constrained_optimization
%reconstruct terminal state  t = (xf,dotxf)
xterm = SSPP_1D_vehicle_i2xk(t) ;

%compute current state
xk = SSPP_1D_vehicle_i2xk(i_grid(idx_i));

if (xk(1) == xterm(1) && xk(2) > xterm(2)) || ... %the obstacle is located at xterm; using the analogy with the toy vehicle circuit, the protective pads were hit while vehicle advances fwd and we want to discourage this from happening      
   (xk(1) == x1_min && xk(2) < 0) %using the analogy with the toy vehicle circuit, the protective pads were hit while vehicle advances bwd and we want to discourage this from happening
    y = 1e1*deltat; %overwrite; high penalty  
end %if
%}

end %function


%% 
function  [pUNtrLij,idx_iprimeAndt_cum] = computeTPMandSpaceReachTot_mu(mu, Ntr) 
%for a given policy mu=mu(iprime), compute Trans Proba Matrix (TPM) and reachability space ending-in/reaching t in any number of transitions k<=Ntr 
% Goal1: Compute/Build the Transition Proba Matrix after Ntr transitions for a given stationary policy mu=mu(iprime); I made use of Trans Proba Matrices properties cf [B2005Bertsekas,AppxD,p478]     
% Goal2: Identify all states from which the terminal state can be reached in any number of stages k, with k<=Ntr; lump (cumulate) all those state into this vector of their indices idx_iprimeAndt_cum     
% Inputs: candidate control policy mu=mu(iprime) ; Ntr = number of transitions to be computed     
% Output: pUNtrLij = pUNtrLij(idx_iprimeAndt,idx_jprimeAndt) is the TPM associated to using mu; notation inspired from [B2005Bertsekas, Appendix D, p478] 

%% load vars
loadVarsFromBaseWS typeTPM  doDisplayMsgOnScreen perc_show_stat  i_grid iprime_grid j_grid t  u_grid   iprimeAndt_grid jprimeAndt_grid  
SS_loadTPM

%% main algo
%ini: malloc
pUNtrLij = sparse(length(iprimeAndt_grid),length(iprimeAndt_grid)) ; %pUNtrLij = pUNtrLij(idx_iprimeAndt_grid,idx_jprimeAndt_grid)    

%%% print/show statistics on screen
%choose
%perc_show_stat = 1; %[percentage] \in (0,100) show statistics with evolution algo in multiples of this qtt

%ini
counter = 1;

%%% Transition k=1 i.e. stage k=1
for idx_iprimeAndt = 1:length(iprimeAndt_grid)
    idx_i = find(i_grid == iprimeAndt_grid(idx_iprimeAndt)) ; %idx_i associated to i_grid
    
    %%% print/show statistics on screen (contd)
    %herebelow code is adapted to handle linear growth of the monitored variable idx_iprimeAndt    
    if doDisplayMsgOnScreen && idx_iprimeAndt > (counter*perc_show_stat*length(iprimeAndt_grid))/100
        fprintf('\nGenerating the TPM associated to the given policy mu: %6.2f%% ready',counter*perc_show_stat);
        counter = counter + 1;
    end %if
        
    if iprimeAndt_grid(idx_iprimeAndt) ~= t %then it makes sense to seek idx asociated on iprime_grid
        idx_iprime = find(iprime_grid == iprimeAndt_grid(idx_iprimeAndt)) ; %idx_iprime associated to iprime_grid        
        
        idx_u = find(u_grid == mu(idx_iprime)) ;
        
        %%% fill in each row associated (aferent) to i by looking at p_ij_u(id_uk)
        %{
        %var1: slow code: for-loop along all jprimeAndt-axis elements: work with individual pUNtrLij-matrix elements at a time
        for idx_jprimeAndt = 1:length(jprimeAndt_grid)
            idx_j = find(j_grid == jprimeAndt_grid(idx_jprimeAndt)) ; %idx_j associated to j_grid  
            
            pUNtrLij(idx_iprimeAndt,idx_jprimeAndt) = p_ij_u_mat(idx_i,idx_j,idx_u); %V&V: p_ij_u(idx_i,:,id_uk)   
        end %for 
        %}
        
        %{-
        %var2: faster code: use find(.) instead of rather slow for-loop: work with multiple pUNtrLij-matrix elements at a time    
        idxs_j = find(~isnan(p_ij_u_mat(idx_i,':',idx_u))); %legacy code for compatibility with regular matrix p_ij_u; line of code not needed for the sparse matrix p_ij_u_sparse    
        ids_idxs_j = find( p_ij_u_mat(idx_i,idxs_j,idx_u) ~= 0); 
        idxs_j = idxs_j(ids_idxs_j); %overwrite; a subset of the original idxs_j   
        %once this line of code is reached, we've identified all non-nan and non-zero p_ij_u_mat(idx_i,idxs_j,idx_u): iow p_ij_u_mat(idx_i,idxs_j,idx_u) are not nan and not zero
        
        [~,~,idxs_jprimeAndt] = intersect(j_grid(idxs_j),jprimeAndt_grid); % match/form associated pairs (idxs_jprimeAndt(kk),idxs_j(kk))  
        
        %conclude
        pUNtrLij(idx_iprimeAndt,idxs_jprimeAndt) = p_ij_u_mat(idx_i,idxs_j,idx_u) ;        
        %}
        
    else %i.e. iprimeAndt_grid(idx_iprimeAndt) == t
        %p_tt(u)=1 \forall u \in U(t) necessary for Stoch Short Path Pb cf [B2005Bertsekas, p405]  
        pUNtrLij(idx_iprimeAndt,idx_iprimeAndt) = 1; %and leave all the other values on the row to NaN    
    end %if   
    
    %V&V: compare:  pUNtrLij(1:5,:)  vs  p_ij_u(1:5,:,1)  vs  p_ij_u(1:5,:,2)  vs  p_ij_u(1:5,:,3)   
    
end %for idx_iprimeAndt 

%%% print/show statistics on screen (contd)
SSPP_1D_vehicle_displayMsgOnScreen('\nGenerating the TPM associated to the given policy mu: %6.2f%% ready',100.); 

%%% for the purpose of matrix multiplication, we need to replace NaN with zero
pUNtrLij(isnan(pUNtrLij)) = 0;

%def
expression = 'union( idx_iprimeAndt_cum, find(pUNtrLij(:,find(iprimeAndt_grid == t)) ~= 0) )';  %calc idx_iprimeAndt associated to iprimeAndt_grid  

%ini
idx_iprimeAndt_cum = [];
idx_iprimeAndt_cum = eval(expression);  %V&V: pUNtrLij(:,find(iprimeAndt_grid == t))   

%%% Transitions k=2 to Ntr i.e. stage k=2 to Ntr
for k=2:Ntr
    pUNtrLij = pUNtrLij*pUNtrLij; %overwrite
    
    idx_iprimeAndt_cum = eval(expression);  %idx_iprimeAndt associated to iprimeAndt_grid  
end %for k=

SSPP_1D_vehicle_displayMsgOnScreen('\nDone computing idx_iprimeAndt_cum');

end %function

%%
function [J, J_vpa] = policyEvaluation_LS_StochShortPathPb(mu_eval) 
% [exact solution] cf [B2005Bertsekas,Prop7.2.1(c), eq(7.7.a)]
% Implement, with numerically high precision, Least-Squares A*J=b for Policy Evaluation
% Input: mu_eval = mu_eval(idx_iprime) 
% Output: J=J(idx_iprime_grid), J_vpa=J_vpa(idx_iprime_grid)   
%         J is double precision: numerical robust implementation (avoid pb singularities) 
%         J_vpa is variable precision (vpa) i.e. w arbitrarily high precision

%% load vars
loadVarsFromBaseWS doDisplayMsgOnScreen  i_grid iprime_grid j_grid jprime_grid u_grid  
SS_loadTPM

%% %%% Preliminary: check that mu_eval complies with Constraint Satisfaction
% assume that mu_eval already complies with Constraint Satisfaction

%% %%% Initializations
%choose A matrix type: regular vs sparse
typeMatrA = 1; %0: regular; 1: sparse;

%ini
switch typeMatrA
    case 0
        A = zeros(length(iprime_grid),length(iprime_grid)); %A=A(idx_iprime,idx_iprime), where idx_iprime is associated to iprime_grid     
    case 1
        A = sparse(length(iprime_grid),length(iprime_grid));
end

%ini
J = nan(length(iprime_grid),1); %J=J(idx_iprime), where idx_iprime is associated to iprime_grid 
b = J; %b=b(idx_iprime), where idx_iprime is associated to iprime_grid 

%ini
[~,idxs_j,~] = intersect(j_grid,jprime_grid) ; %consequently  j_grid(idxs_j) == jprime_grid(idxs_jprime_grid) cover all values of jprime_grid 
length_jprime_grid = length(iprime_grid);


%% %%% main algo: form (A,b)
SSPP_1D_vehicle_displayMsgOnScreen('\n        Forming (A,b)'); 
for idx_iprime = 1:length(iprime_grid)
    idx_i = find(i_grid == iprime_grid(idx_iprime)) ; %idx_i associated to i_grid 
    
    %%% build vector b=b(idx_iprime_grid)
    b(idx_iprime) = g_i_mui(idx_i);    
    
    %%% build matrix A=A(idx_iprime,idx_iprime), where idx_iprime is associated to iprime_grid     
    idx_u = find(u_grid == mu_eval(idx_iprime))  ; %este vb de a 3a dimensiune a matr p_ij_u 
    
    %{
    %var1: slow for-loop along all jprime-axis elements: work with individual A-matrix elements at a time    
    for idx_jprime = 1:length_jprime_grid
        idx_j = find(j_grid == jprime_grid(idx_jprime)); %idx_j aferent/asociat j_grid
        if (idx_iprime == idx_jprime) %the diagonal of matrix A
            if ~isnan(p_ij_u_mat(idx_i,idx_j,idx_u)) %then it makes sense to account for the term "p_ij_u*J(j)" appearing in Prop7.2.1(c) eq(7.7.a) 
                A(idx_iprime,idx_jprime) = 1 - p_ij_u_mat(idx_i,idx_j,idx_u);
            else %then it does NOT make sense to account for the term "p_ij_u*J(j)" appearing in Prop7.2.1(c) eq(7.7.a) 
                A(idx_iprime,idx_jprime) = 1;
            end %if ~isnan(.)
        else %i.e. in all other cases where idx_iprime \neq idx_jprime  
            if ~isnan(p_ij_u_mat(idx_i,idx_j,idx_u)) %then it makes sense to account for the term "p_ij_u*J(j)" appearing in Prop7.2.1(c) eq(7.7.a)
                A(idx_iprime,idx_jprime) = - p_ij_u_mat(idx_i,idx_j,idx_u);
            end %end %if ~isnan(.)
            %otherwise do nothing and leave A(idx_i,idx_j) = 0
            
        end %if (idx_i == idx_j)
    end %for idx_jprime =
    %}
    %V&V: Arow_var1 = A(idx_iprime,:); 
    
    %{-
    %var2: faster code: use find(.) instead of rather slow for-loop: work with multiple A-matrix elements at a time    
    A(idx_iprime,idx_iprime) = 1; %diagonal term cf Prop7.2.1(c) eq(7.7.a)       
    
    ids_idxs_j = find(~isnan(p_ij_u_mat(idx_i,idxs_j,idx_u))); %legacy code for compatibility with regular matrix p_ij_u; line of code not needed for the sparse matrix p_ij_u_sparse    
    idxs_j_temp = idxs_j(ids_idxs_j); 
    ids_idxs_j_temp = find( p_ij_u_mat(idx_i,idxs_j_temp,idx_u) ~= 0); 
    idxs_j_temp = idxs_j_temp(ids_idxs_j_temp); %overwrite; a subset of the original idxs_j_temp   
    %once this line of code is reached, we've identified all non-nan and non-zero p_ij_u_mat(idx_i,idxs_j,idx_u): iow p_ij_u_mat(idx_i,idxs_j,idx_u) are not nan and not zero
    
    [~,~,idxs_jprime_grid_temp] = intersect(j_grid(idxs_j_temp),jprime_grid) ; %overwrite; match/form associated pairs (idxs_j_temp(kk),idx_jprime_grid_temp(kk))   
    
    %conclude
    A(idx_iprime,idxs_jprime_grid_temp) = A(idx_iprime,idxs_jprime_grid_temp) - p_ij_u_mat(idx_i,idxs_j_temp,idx_u);
    %}
    %V&V: Arow_var2 = A(idx_iprime,:) and compare with Arow_var1: they should match:  find(Arow_var2-Arow_var1) should be empty             
    
end %for idx_iprime = 


%% %%% apply LS
%%% compute J_vpa (arbitrarily high precision) 
%increase precision beyond double cf PP_matrixInvArbitraryPrecision.m and PP_expId_YTheta.m  
digits(100) %set significant decimal digits (to be used by vpa) := cifre inainte de "." + 1 (punctul) + cifre dupa "."    

SSPP_1D_vehicle_displayMsgOnScreen('\n        Forming (A_vpa,b_vpa)'); 
A_vpa = vpa(A);
b_vpa = vpa(b);

%conclude 
SSPP_1D_vehicle_displayMsgOnScreen('\n        Solving system of linear eqs ito vpa'); 
J_vpa = A_vpa\b_vpa; %J_vpa=J_vpa(idx_iprime_grid) 

%%% compute J 
%{
%var1 [to-be-avoided]: (double precision): low precision: possible pb singularities
J = A\b; %J=J(idx_iprime_grid) 
%}

%{-
%var2 [better]: (vpa precision): high precision: cf https://uk.mathworks.com/help/symbolic/double.html
J = double(J_vpa); %J=J(idx_iprime_grid) 
%}


end %function


%%
function muUkp1 = policyImprovement_StochShortPathPb(JLmuUk)
% cf [B2005Bertsekas,Prop7.2.1(e)]

%% load vars
loadVarsFromBaseWS i_grid iprime_grid j_grid jprime_grid u_grid 
SS_loadTPM

%% main algo
%ini
muUkp1 = nan(length(iprime_grid),1) ;

%compute indices idxs_j associated to j_grid corresp to values on j_grid common to jprime_grid 
[~,idxs_j,~]=intersect(j_grid,jprime_grid) ; %find indices within j_grid with common values of both j_grid and jprime_grid   

for idx_iprime = 1:length(iprime_grid)
    idx_i = find(i_grid==iprime_grid(idx_iprime)) ; %idx_i associated to (aferent) i_grid
    
    %ini
    h_cand = nan(length(u_grid),1); %h_cand = h_cand(idx_u_grid) 
    
    %{
    %debug-purpose only
    if 29 == iprime_grid(idx_iprime) 
        disp('ctdr: Place a breakpoint here');
    end
    %}
    
    for idx_u=1:length(u_grid) % !! Constraint Satisfaction to-be-checked/implemented here: see e.g. "function muUkp1 = policyImprovement_discountedPb(JLmuUk)" in PP_exo1_3_2_Bertsekas_inventCtrlPb_discPb.m    
        u=u_grid(idx_u);
    
        h_cand(idx_u) = g_i_mui(idx_i); % h_cand(u) := g(i,u)+sum_j p_ij(u) JLmuUk(j), namely the qtt between square brackets in [B2005Bertsekas,Prop7.2.1(e)]
        
        %[copy-paste from PP_exo1_3_2_Bertsekas_inventCtrlPb_discPb.m with change/adaptation to exclude terminal state info] var2: use vector (matrix) multiplication in place of sum_j in [B2005Bertsekas,Prop7.3.1(e) for speedier numerical computation   
        p_row_temp = p_ij_u_mat(idx_i,idxs_j,idx_u);
        p_row_temp(isnan(p_row_temp)) = 0; %replace nan with zero inside transition proba matrix  
        
        h_cand(idx_u) = h_cand(idx_u) + mrv(p_row_temp)*mcv(JLmuUk) ;
    end %for idx_u=
    
    %% sele optimum ctrl law 
    idx_u_grid_opt = find( h_cand == min(h_cand) ) ; %might yield multiple solutions; cf matlab: min([1 -2 nan]) = -2
    
    %%%% handle multiple optima - Part1: decision to select the control with lowest absolute value (physical interpretation: use as little energy as possible to push the vehicle given the fact that the effect ito cost h_cand is the same)  
    temp = abs(u_grid(idx_u_grid_opt)) ;  
    id_idx_u_grid_opt = find(temp == min(temp)) ; 
    
    %%%% handle multiple optima - Part2 
    if (length(id_idx_u_grid_opt)>=2)
        SSPP_1D_vehicle_displayMsgOnScreen('\n       Multiple optima found, suggesting multiple optimal ctrl laws !');
    end %if
    
    %handle multiple optima: just select the 1st one
    muUkp1(idx_iprime) = u_grid(idx_u_grid_opt(id_idx_u_grid_opt(1))) ;
    
end %for i=

end %function

%%
function mu = findAFeasibleStatioPolicy()  %mu=mu(idx_iprime_grid)  
% Using relation Parent -> Child, iteratively find all Parents associated to Child = t (i.e.) terminal state, then seek all Grandparents -> Parents, etc.   
% Definition: parent = a state from which a child state can be reached with proba: (i) [for all child states \neq t] \in (0,1); (ii) [when the child state is the terminal state t] \in (0,1]   
% Ex:  

loadVarsFromBaseWS doDisplayMsgOnScreen  perc_show_stat t iprime_grid  x1_grid x2_grid w_grid u_grid 

%ini
mu = nan(length(iprime_grid),1); 
visitedStates = [t]; %visitedStates consists of states (values, numbers) belonging to i_grid   
nrNewParentStates = 1; %count how many new states were added to vector visitedStates from one iteration to another when seeking the relation Parent -> Child

%%% print/show statistics on screen
%choose
%perc_show_stat = 1; %[percentage] \in (0,100) show statistics with evolution algo in multiples of this qtt

%ini
counter = 1;

while nrNewParentStates >= 1 %actually this stands for an iteration-loop
    %reset counter
    nrNewParentStates = 0;
    
    %%% print/show statistics on screen (contd)
    %ini
    doReiter = true; %do reiterate 
    doPrntMsgScr = false; %do print message on screen 
    
    %herebelow code is adapted to handle nonlinear growth of the monitored variable length(visitedStates): the purpose of the while-loop is to adjust/identify the right value of counter by iteratively increasing it     
    while (doReiter)
        doReiter = false; %do reiterate 
        if length(visitedStates) > (counter*perc_show_stat*length(iprime_grid))/100
            counter = counter + 1;
            doReiter = true;
            doPrntMsgScr = true; 
        end %if
    end %while 
    
    %print msg on screen
    if doPrntMsgScr 
        SSPP_1D_vehicle_displayMsgOnScreen('\nGenerating a feasible stationary policy: %6.2f%% ready',(counter-1)*perc_show_stat); 
    end %if
    
    %%% main algo
    for idx_visitedStates = 1:length(visitedStates)
        i_xkp1 = visitedStates(idx_visitedStates);
        
        %{
        %%% var1 [slow code] using SSPP_1D_vehicle_invSysDyn_xkp1(.) relying on many for-loops  
        xkp1 = SSPP_1D_vehicle_i2xk(i_xkp1);
        
        % seek all combinations (xk,uk,wk) that lead to xkp1
        Mparent_idxs_xk_wk_uk = SSPP_1D_vehicle_invSysDyn_xkp1(xkp1) ;
        
        for id_Midx_parent_xk_wk_uk=1:size(Mparent_idxs_xk_wk_uk,1)
            %form pair (xk,wk,uk)
            xk = [x1_grid(Mparent_idxs_xk_wk_uk(id_Midx_parent_xk_wk_uk,1)) x2_grid(Mparent_idxs_xk_wk_uk(id_Midx_parent_xk_wk_uk,2))];
            %wk = w_grid(Midx_parent_xk_wk_uk(id_Midx_parent_xk_wk_uk,3)); %not needed here 
            uk = u_grid(Mparent_idxs_xk_wk_uk(id_Midx_parent_xk_wk_uk,4));
            
            %check if xk not already present in visistedStates and if not, add it
            i_xk = SSPP_1D_vehicle_xk2i(xk);
            if isempty(find(visitedStates == i_xk)) %by this code, we are sure to avoid t
                visitedStates = [ visitedStates i_xk ];
                
                %update accordingly
                mu(find(iprime_grid == i_xk) ) = uk;
                nrNewParentStates = nrNewParentStates+1;
            end %if isempty(.)            
        end %for id_parent_xk_wk_uk=        
        %}
        
        %{-
        %%% var2 [faster code] using SSPP_1D_vehicle_invSysDyn_j(.) relying on Trans Proba Matr   
        M_iprime_idxs_w_u = SSPP_1D_vehicle_invSysDyn_j(i_xkp1) ;
        
        for id_M_iprime_idxs_w_u=1:size(M_iprime_idxs_w_u,1)
            i_xk = M_iprime_idxs_w_u(id_M_iprime_idxs_w_u,1) ;
            uk = u_grid(M_iprime_idxs_w_u(id_M_iprime_idxs_w_u,3));
            
            %check if i_xk not already present in visistedStates and if not, add it
            if isempty(find(visitedStates == i_xk)) %by this code, we are sure to avoid t
                visitedStates = [ visitedStates i_xk ];
                
                %update accordingly
                mu(find(iprime_grid == i_xk) ) = uk;
                nrNewParentStates = nrNewParentStates+1;
            end %if isempty(.)
            
        end %for id_M_iprime_idxs_w_u=        
        %}
        
    end %for idx_visitedStates =
end %while 

%%% print/show statistics on screen (contd)
%avoid printing two times the same msg corresp to 100% ready     
if (counter-1)*perc_show_stat<100 
    SSPP_1D_vehicle_displayMsgOnScreen('\nGenerating a feasible stationary policy: %6.2f%% ready',100.); 
end

end %function

%%


%%
function hs2 = visu_motion_time_evol(traj, info_policy)
% visualize the motion (i.e. evolution) in time of the vehicle given a policy (e.g. the optimal policy)

loadVarsFromBaseWS expr_title delta_x1 delta_x2 delta_u delta_w deltat  u_grid  deltat_simu  

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

%%
function hs1 = visu_motion_state_space_evol(traj, info_policy)
% plot evolution in state-space of trajectory above

loadVarsFromBaseWS expr_plot_surf_mu_opt expr_plot_term_state_t expr_title  x1_grid x2_grid  mu_opt_of_x_grid J_opt_of_x_grid  t xterm  delta_x1 delta_x2 delta_u delta_w deltat  deltat_simu 

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

%%




%% 

