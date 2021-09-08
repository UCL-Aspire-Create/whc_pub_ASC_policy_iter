function xk = SSPP_1D_vehicle_i2xk(i, doPlot) %xk=(x1k,x2k) 
% Identify state i's associated point (state variable) xk on (x1_grid,x2_grid). i \in i_grid (via matrix M's row nr)  
% Input: doPlot is optional parameter
% Note: i should belong to i_grid and xk will belong to (x1_grid,x2_grid)  
% Ex:   doPlot=true; SSPP_1D_vehicle_i2xk(1, doPlot)


if nargin <=1 %then doPlot was not defined at all
   doPlot = false; 
end

loadVarsFromBaseWS x1_grid x2_grid M 

%% check input consistency
if i<1 || i>size(M,1)
    error('ctdr: state i falls outside i_grid');
end

%% ensure input consistency
i = round(i);

%% main algo
xk = [x1_grid(M(i,1))  x2_grid(M(i,2))];

%% plotting
if doPlot
    %choose
    lw = 3; %linewidth
    cw = 5; %circle width
    lcs = 'b-'; %line color style
    
    figure;
    h1 = plot([min(x1_grid) min(x1_grid) max(x1_grid) max(x1_grid) min(x1_grid)], ...
              [min(x2_grid) max(x2_grid) max(x2_grid) min(x2_grid) min(x2_grid)],lcs, 'linewidth',lw, 'displayname','admissible domain boundary'); hold on; grid on;  
    h2 = plot(xk(1),xk(2),'ro', 'linewidth',cw, 'displayname','x_k');
    %axis equal;     
    xlabel('x_1 [m]');
    ylabel('x_2 [m/s]');
    
    legend([h1 h2]);
end %do Plot


end