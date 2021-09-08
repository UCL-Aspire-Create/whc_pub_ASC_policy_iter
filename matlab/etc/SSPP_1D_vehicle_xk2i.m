function i = SSPP_1D_vehicle_xk2i(xk, doPlot) %xk=(x1k,x2k) 
% Identify state variable xk's correspondent state i \in i_grid (via matrix M's row nr). xk is forcedly placed on (x1_grid,x2_grid)  
% Input: doPlot is optional parameter
% Note: xk does not necessarily need to belong exactly on (x1_grid,x2_grid) (a closest match approach will be used herebelow; iow extrapolation banned beyond the bounds of x_grid), and i will belong to i_grid    

if nargin <=1 %then doPlot was not defined at all
   doPlot = false; 
end

loadVarsFromBaseWS x1_grid x2_grid M 

%% ensure input consistency
% enforce xk should stay within bounds of x_grid;
xk(1) = min(max(x1_grid), max(min(x1_grid), xk(1)) ) ;  
xk(2) = min(max(x2_grid), max(min(x2_grid), xk(2)) ) ;  


%% main algo
%%%%% find closest match on (x1_grid,x2_grid)
%{
%%%var1: tolerate small deviation of xk wrt (x1_grid, x2_grid)
%choose
eps_tol = 1e2*eps ; 

id_x1_grid = mintersect( find(x1_grid >= xk(1)-eps_tol), find(x1_grid <= xk(1)+eps_tol) ) ;
id_x2_grid = mintersect( find(x2_grid >= xk(2)-eps_tol), find(x2_grid <= xk(2)+eps_tol) ) ;
%}

%{-
%%%var2: tolerate large deviation of xk wrt (x1_grid, x2_grid)     
doPlot = 0; %choose


% find indices on x_grid
id_x1_grid = round( fzero_curve(1:length(x1_grid),x1_grid,[0 -xk(1)], doPlot) ); %index on x1_grid; round(.) is used for numerical robustness reasons
id_x2_grid = round( fzero_curve(1:length(x2_grid),x2_grid,[0 -xk(2)], doPlot) ); %index on x1_grid; round(.) is used for numerical robustness reasons
%}

if (isempty(id_x1_grid) || isempty(id_x2_grid))
   error('ctdr: Please check xk'); 
end
%otherwise continue hereafter

%%% find i within M
i = mintersect( find(M(:,1) == id_x1_grid),find(M(:,2) == id_x2_grid) ) ; %i represents a row nr within matrix M   

%%% post-check
if (length(i) ~= 1)
    disp('ctdr: Please check i');
end


%% plotting
if doPlot
    %choose
    lw = 3; %linewidth
    cw = 5; %circle width
    lcs = 'b-'; %line color style
    
    figure;
    h1 = plot([min(x1_grid) min(x1_grid) max(x1_grid) max(x1_grid) min(x1_grid)], ...
              [min(x2_grid) max(x2_grid) max(x2_grid) min(x2_grid) min(x2_grid)],lcs, 'linewidth',lw, 'displayname','admissible domain boundary'); hold on; grid on;  
    h2 = plot(x1_grid(id_x1_grid),x2_grid(id_x2_grid),'ro', 'linewidth',cw, 'displayname','x_k');
    %axis equal;     
    xlabel('x_1 [m]');
    ylabel('x_2 [m/s]');
    
    legend([h1 h2]);
end %do Plot

end %function
