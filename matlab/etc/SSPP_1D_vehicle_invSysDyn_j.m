function M_iprime_idxs_w_u = SSPP_1D_vehicle_invSysDyn_j(j)
% Solve Inverse system dynamics pb using/based on Trans Proba Matrix: given destination state j, find causating (i,u,w) that lead to j
% Input: j \in j_grid
% Output: M_iprime_idxs_w_u = [iprime idx_w_grid idx_u_grid]
% Note that according to its def, iprime EXCLUDES the terminal state t (this is to accommodate for StochShortPathPb where the terminal state t is only a destination, not a source: if we start at t, we can only end up at t, nowhere else).
% Ex1:   SSPP_1D_vehicle_invSysDyn_j(t)    
% Ex2:   SSPP_1D_vehicle_invSysDyn_j( ceil(t/2) ) 

%% check input data consistency
if (length(j) ~= 1)
    error('ctdr: Please check j');
end %if


%% ensure data consistent format
% not necessary here

%% load vars
loadVarsFromBaseWS i_grid iprime_grid u_grid w_grid  Pw     
SS_loadTPM
    
%% main algo
%ini
M_iprime_idxs_w_u = [];

for idx_u=1:length(u_grid)     
    u=u_grid(idx_u);
        
    idxs_i = find(ismember(p_ij_u_mat(':',j,idx_u),Pw)) ; %find indices within i_grid with common values of both p_ij_u(:,j,idx_u) and Pw (algo with repetition, whereas [~,id]=intersect(.) yields results wo repetition); other values might include P=1 for SSPP    
    %V&V:  p_ij_u(idxs_i,j,idx_u)    
    
    for iprime = intersect(iprime_grid,i_grid(idxs_i))
        i = iprime; %same values/numbers used
        
        for idx_w = find(Pw == p_ij_u_mat(i,j,idx_u))
            M_iprime_idxs_w_u = [ M_iprime_idxs_w_u; ...
                                  iprime idx_w idx_u ];
        end %for idx_w =
    end %for i=
end %for idx_u=


