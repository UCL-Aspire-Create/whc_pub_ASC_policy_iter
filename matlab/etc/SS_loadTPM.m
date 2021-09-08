%% load Trans Proba Matrix depending on the value of typeTPM: this can be p_ij_u and/or p_ij_u_sparse 

loadVarsFromBaseWS typeTPM

if ~isempty(intersect(typeTPM,[0,2])) %0: use regular p_ij_u matrix
    loadVarsFromBaseWS p_ij_u 
    
    %def
    p_ij_u_mat = @(i,j,id_u) p_ij_u(i,j,id_u); 
end %if ~isempty(intersect(typeTPM,.))

if ~isempty(intersect(typeTPM,[1,2])) %1: use sparse p_ij_u matrix
    loadVarsFromBaseWS p_ij_u_sparse 
    
    %def
    p_ij_u_mat = @(i,j,id_u) full(p_ij_u_sparse{id_u}(i,j));
end