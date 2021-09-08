%% def transition proba matrix p_ij(u): see method detailed in [B2005Bertsekas, file attached inside book called "Prop7.3.1(b) appl to ex1.3.2_invent ctrl pb.pdf"]
%%                                      2 variants: regular and sparse matrices     
%choose
typeTPM = 1; %0: use regular matrix p_ij_u; 1: use sparse matrix p_ij_u_sparse; 2: case 0 + case 1; 


%%% ini p_ij_u (i.e. the regular matrix)
if ~isempty(intersect(typeTPM,[0,2])) %0: use regular p_ij_u matrix   
    p_ij_u = nan(length(i_grid),length(j_grid),length(u_grid)) ; % p_ij_u=p_ij_u(id_i,id_j,idx_u), where id_i is an index inside i_grid; id_j is an index inside j_grid; idx_u is an index inside u_grid 
end %if ~isempty(intersect(typeTPM,.))

%%% ini vectors that shall define p_ij_u_sparse (i.e. the sparse matrix)  
if ~isempty(intersect(typeTPM,[1,2])) %1: use sparse p_ij_u matrix   
    L1 = length(i_grid)*length(u_grid)*length(w_grid); % =repres= max size of p_ij_u_sparse in case no overlap ito destination j
    L2 = power(length(i_grid),2)*length(w_grid); % =repres= the size of a full p_ij_u_sparse matrix
    
    i_sparse = nan(1, min(L1,L2) ); % for large (length(u_grid),length(w_grid)) there will most likely be overlap ito destination j, but surely no more than the full matrix capacity
    j_sparse = i_sparse;
    v_sparse = i_sparse;
    id_u_sparse = i_sparse;
    id_elem = 0; %index of new element to store inside all vectors above
end %if ~isempty(intersect(typeTPM,.))

%%% print/show statistics on screen - part1
%choose
%perc_show_stat = 1; %[percentage] \in (0,100) show statistics with evolution algo in multiples of this qtt

%ini
counter = 1; 

%%% main algo 
for idx_i = 1:length(i_grid)
    i=i_grid(idx_i); % here: this def of i \in i_grid

    %%% print/show statistics on screen - part2
    if i == 1
        SSPP_1D_vehicle_displayMsgOnScreen('\nStarting Generating TPM'); 
    elseif i > (counter*perc_show_stat*length(i_grid))/100
        if ~isempty(intersect(typeTPM,[0,2])) %0: use regular p_ij_u matrix   
            SSPP_1D_vehicle_displayMsgOnScreen('\nGenerating matrix p_ij_u: ......................... %6.2f%% ready',counter*perc_show_stat); 
        end %if ~isempty(intersect(.)) 
        
        if ~isempty(intersect(typeTPM,[1,2])) %1: use sparse p_ij_u matrix   
            SSPP_1D_vehicle_displayMsgOnScreen('\nGenerating vectors that shall define p_ij_u_sparse: %6.2f%% ready',counter*perc_show_stat); 
        end %if ~isempty(intersect(.)) 
        
        %update
        counter = counter + 1;
    end %if
    
    %% find state i's associated point on (x1_grid,x2_grid) 
    xk = SSPP_1D_vehicle_i2xk(i);
    
    for id_uk=1:length(u_grid)
        uk = u_grid(id_uk);
        
        for id_wk=1:length(w_grid)
            wk = w_grid(id_wk);
                        
            %% calc next state
            xkp1 = SSPP_1D_vehicle_sysDyn(xk,uk,wk, deltat) ; %xkp1 occurs with associated proba Pw(id_wk)
            
            %% find associated j to xkp1: closest match on (x1_grid,x2_grid)
            j = SSPP_1D_vehicle_xk2i(xkp1) ;            
            
            if isempty(j)
                error('ctdr: Please check j as sth went wrong');
            end
            
            %% assign proba
            if ~isempty(intersect(typeTPM,[0,2])) %0: use regular p_ij_u matrix   
                    %var1: using regular (not sparse) matrix
                    if isnan(p_ij_u(i,j,id_uk))
                        p_ij_u(i,j,id_uk) = Pw(id_wk) ; %%getting to this j is possible via at least one path ito combinations (wk,uk) for this fixed i   
                    else
                        p_ij_u(i,j,id_uk) = p_ij_u(i,j,id_uk) + Pw(id_wk); %getting to this j is possible via multiple paths ito combinations (wk,uk) for this fixed i   
                    end %if isnan(.)
            end %if ~isempty(intersect(.))
                    
            if ~isempty(intersect(typeTPM,[1,2])) %1: use sparse p_ij_u matrix   
                    %var2: using sparse matrix
                    %% seek whether (i,j) already present in <name>_sparse
                    idx_i_sparse = mintersect(find(i_sparse == i),find(j_sparse == j),find(id_u_sparse == id_uk) ); %seek if (i,j) already reached via another path; find common indices in both i_sparse and j_sparse that match criteria                    
                    if isempty(idx_i_sparse) %then insert a new element inside p_ij_u_sparse
                        id_elem = id_elem+1;
                        i_sparse(id_elem) = i; %1st argument of doc sparse
                        j_sparse(id_elem) = j; %2nd argument of doc sparse
                        id_u_sparse(id_elem) = id_uk; %
                        v_sparse(id_elem) = Pw(id_wk); %3rd argument of doc sparse
                    elseif length(idx_i_sparse) == 1
                        v_sparse(idx_i_sparse) = v_sparse(idx_i_sparse) + Pw(id_wk); %V&V: v_sparse(1:10) 
                    else %e.g. if length(idx_i_sparse) >=2 %then sth went wrong so signal it
                        error('ctdr: Please check idx_i_sparse');
                    end %if isempty(.) 
            end %if ~isempty(intersect(.))
            
            %{
            %% testing-purpose
            a = [1 0 1 0];
            b = [1 0 1 0];
            idx = intersect(find(a==1),find(b==1)) %will return idx s.t. a(idx) == b(idx) == 1           
          
            a = [7 0 1 0];
            b = [4 0 1 0];
            idx = intersect(find(a==7),find(b==4))
            
            %}
            
        end %for id_wk
    end %for id_uk
end %for idx_i=

%%% print/show statistics on screen (contd)
if ~isempty(intersect(typeTPM,[0,2])) %0: use regular p_ij_u matrix
    SSPP_1D_vehicle_displayMsgOnScreen('\nGenerating matrix p_ij_u: ......................... %6.2f%% ready',counter*perc_show_stat); 
end %if ~isempty(intersect(.))

if ~isempty(intersect(typeTPM,[1,2])) %1: use sparse p_ij_u matrix
    SSPP_1D_vehicle_displayMsgOnScreen('\nGenerating vectors that shall define p_ij_u_sparse: %6.2f%% ready',counter*perc_show_stat); 
end %if ~isempty(intersect(.))


%{
%% ZZ(wrong interpretation of Assumption7.2.1's meaning: there it speaks ito pUk_ij_u, whereas here I've applied it to p_ij_u) post-process p_ij_u: ensure Assumption7.2.1 holds: avoid P=1 for states other than t (i.e. the terminal state) 
% Decision to set/mark those elements with NaN inside Trans Proba Matrix translating the fact that we exclude current u=u_grid(id_uk) from the list of feasible or candidates u_cand associated (aferente) to state i 

%choose
eps_tol = 1e2*eps ; 

if ~isempty(intersect(typeTPM,[0,2])) %0: use regular p_ij_u matrix
    for id_uk=1:length(u_grid)
        uk = u_grid(id_uk);
        
        %%% identify matrix elemenents where P=1
        %{
        %var1: invalidate only that matrix element where P=1
        P_temp = p_ij_u(setdiff(i_grid,t),:,id_uk); %V&V: P_temp
        P_temp(find(P_temp >= 1-eps_tol)) = nan ;
        p_ij_u(setdiff(i_grid,t),:,id_uk) = P_temp;
        %}
        
        % %{-
        %var2: invalidate the entire row where an element is P=1
        for idx_iprime = 1:length(iprime_grid)
            iprime=iprime_grid(idx_iprime); % here: this def of i \in i_grid
                         
            if ~isempty(find(p_ij_u(iprime,:,id_uk) >= 1-eps_tol)) 
                p_ij_u(iprime,:,id_uk) = nan(1,length(j_grid)); %V&V: p_ij_u(iprime,:,id_uk)    
            end %if            
        end %for idx_i
        % %}
        
    end %for id_uk
end %if

if ~isempty(intersect(typeTPM,[1,2])) %1: use sparse p_ij_u matrix
    id_j_sparse_mark_for_removal = []; %ini
    
    for id_v_sparse = find(v_sparse >= 1-eps_tol)
        if (i_sparse(id_v_sparse) ~= t) %if it's not a terminal state
            %{
            %var1: invalidate only that element where P=1
            id_v_sparse_mark_for_removal = [id_v_sparse_mark_for_removal id_v_sparse];
            %}
            
            % %{-
            %var2: invalidate the entire row where an element is P=1
            id_j_sparse_mark_for_removal = [id_j_sparse_mark_for_removal  mintersect(find(i_sparse == i_sparse(id_v_sparse)), find(id_u_sparse == id_u_sparse(id_v_sparse))) ] ;
            % %}
        end %if
    end %for
    
    %%% remove content associated to id_i_sparse_mark_for_removal
    i_sparse(id_j_sparse_mark_for_removal)    = [];
    j_sparse(id_j_sparse_mark_for_removal)    = [];
    id_u_sparse(id_j_sparse_mark_for_removal) = [];
    v_sparse(id_j_sparse_mark_for_removal)    = [];
    
end %if
%}

%% [optional ito code operation/needs] enforce condition p_tt(u)=1 \forall u \in U(t) necessary for Stoch Short Path Pb cf [B2005Bertsekas, p405]: bypass sys dyna iot set up this Stoch Short Path Pb 
% Actually for the purpose of running "policy iteration" it does not really matter this row within p_ij_u(t,.,.) as this state does not belon to iprime and in "policy iteration" mu=mu(idx_iprime) and we are only interested in transitions happening between iprime   

if ~isempty(intersect(typeTPM,[0,2])) %0: use regular p_ij_u matrix
    %herebelow code ensures setting p_ij(u)=1 is done ONLY for feasible/admissible ctrl laws u \in U(t): look at the associated row content inside matrix p_ij_u     
    for idx_u = 1:length(u_grid)
        %check if this u=u_grid(id_u) is a feasible cde starting-from/associated-to state t  i.e.  u \notin U(t)
        if all(isnan(p_ij_u(t,:,idx_u))) %then this u=u_grid(idx_u) is not a feasible cde starting-from/associated-to state t  i.e.  u \notin U(t)
            %do nothing
        else %then this u=u_grid(idx_u) is a feasible cde starting-from/associated-to state t  i.e.  u \in U(t)
            %act on the row of the Transition Proba Matrix: p_tt(u) =should-be=1 and p_{t,iprime}(u)=should-be=nan
            p_ij_u(t,t,idx_u) = 1;
            p_ij_u(t,jprime_grid,idx_u) = nan;   %V&V: p_ij_u(t,jprime_grid,idx_u)    
        end %if
    end %for id_u =
end %if ~isempty(intersect(.))   

if ~isempty(intersect(typeTPM,[1,2])) %1: use sparse p_ij_u matrix
    %herebelow code ensures setting p_ij(u)=1 is done ONLY for feasible/admissible ctrl laws u \in U(t)   
    id_i_sparse_mark_for_removal = []; %ini
    for id_i_sparse = find(i_sparse == t)
        if j_sparse(id_i_sparse) ~= t 
            id_i_sparse_mark_for_removal = [id_i_sparse_mark_for_removal id_i_sparse]; 
        else %i.e. j_sparse(id_i_sparse) == t 
            %enforce p_tt(u)=1
            v_sparse(id_i_sparse) = 1;   
        end %if
    end %for
    
    %%% remove content associated to id_i_sparse_mark_for_removal
    i_sparse(id_i_sparse_mark_for_removal)    = [];
    j_sparse(id_i_sparse_mark_for_removal)    = [];
    id_u_sparse(id_i_sparse_mark_for_removal) = [];
    v_sparse(id_i_sparse_mark_for_removal)    = [];
end %if ~isempty(intersect(.))


%% refine iprime used for designing optimal ctrl law mu=mu(idx_iprime): exclude states from running "policy iteration" for which no feasible ctrl law exists    
% This comes down to checking "horizontal" slices in the matrix p_ij_u(iprime,jprime,id_uk) whether they consist solely of NaN; we are only interested in iprime since "policy iteration" runs along iprime iotbat identify mu=mu(idx_iprime)    

%ini: code common to typeTMP \in {0,1,2}
idx_iprime_mark_for_removal = [];

if ~isempty(intersect(typeTPM,[0])) %0: use regular p_ij_u matrix
    %                          ^ the case typeTMP==2 is handled once (not twice) in the next if-clause, using the sparse matrix approach     
    for idx_iprime = 1:length(iprime_grid)
        idx_i = find(i_grid == iprime_grid(idx_iprime)) ; %idx_i associated to i_grid
        
        P_temp = squeeze(p_ij_u(idx_i,:,:)); %remove the dimension of length 1 
        if all(isnan(P_temp(:))) %then we have indeed identified a state from which there is no possibility to advance anywhere (regardless of the cde applied); decision to exclude it
            idx_iprime_mark_for_removal = [idx_iprime_mark_for_removal  idx_iprime];
        end %if all(.)       
    end %for    
end %if ~isempty(intersect(.)) 

if ~isempty(intersect(typeTPM,[1,2])) %1: use sparse p_ij_u matrix
    %%%var1 [using p_ij_u_sparse] [WIP]: after having generated p_ij_u_sparse, do the same as above by looking at p_ij_u_sparse{id_uk} and select/mark idx_iprime_mark_for_removal, then eventually re-adjust iprime_grid = setdiff(iprime_grid,iprime_grid(idx_iprime_mark_for_removal)) ;   

    %%%var2 [wo using p_ij_u_sparse]: before having generated p_ij_u_sparse: work with vectors i_sparse, j_sparse, etc.  
    for idx_iprime = 1:length(iprime_grid)
        idx_i_sparse = find(i_sparse == iprime_grid(idx_iprime)) ;
        
        if isempty(idx_i_sparse) %then we have indeed identified a state from which there is no possibility to advance anywhere (regardless of the cde applied); decision to exclude it
            idx_iprime_mark_for_removal = [idx_iprime_mark_for_removal  idx_iprime];
        end %if isempty(.)        
    end %for    
end %if ~isempty(intersect(.))

%%% conclude: code common to typeTMP \in {0,1,2}: remove content associated to idx_iprime_mark_for_removal
if ~isempty(idx_iprime_mark_for_removal)
    fprintf('\nRemoving states from which there is no possibility to advance anywhere (regardless of the cde applied)\n');
    iprime_grid = setdiff(iprime_grid,iprime_grid(idx_iprime_mark_for_removal)) ;
end %if


%% housekeeping: free unnecessarily used memory
if ~isempty(intersect(typeTPM,[1,2])) %1: use sparse p_ij_u matrix
    i_sparse(id_elem+1:end)    = []; %\equiv   i_sparse = i_sparse(1:id_elem)  
    j_sparse(id_elem+1:end)    = [];
    id_u_sparse(id_elem+1:end) = [];
    v_sparse(id_elem+1:end)    = [];
end %if


%% construct sparse matrix p_ij_u_sparse
if ~isempty(intersect(typeTPM,[1,2])) %1: use sparse p_ij_u matrix
    for id_uk=1:length(u_grid)
        idx_id_u_sparse = find(id_u_sparse == id_uk) ;
        if ~isempty(idx_id_u_sparse) %then it makes sense to do sth with idx_id_u_sparse  
            p_ij_u_sparse{id_uk} = sparse(i_sparse(idx_id_u_sparse), j_sparse(idx_id_u_sparse), v_sparse(idx_id_u_sparse), length(i_grid),length(j_grid) );
        else
            error('\nctdr: There is sth wrong with idx_id_u_sparse and cannot generate p_ij_u_sparse{id_uk}');
        end %if ~isempty(.)
    end %for id_uk=
end %if

%%% housekeeping
clear i_sparse j_sparse v_sparse id_u_sparse

%%% save
%save(strcat('results_1D_vehicle.',datestr(now,'yyyy.mmmm.dd.HH.MM.SS.FFF'),'.mat'),  'p_ij_u','p_ij_u_sparse') ;

%{-
%% V&V0: check that content (i.e. elements) of p_ij_u are \in [0,1]
if ~isempty(intersect(typeTPM,[0,2])) %0: use regular p_ij_u matrix
   if ~isempty(union(find(p_ij_u(:)<0),find(p_ij_u(:)>1))) 
      error('ctdr: Please check values inside p_ij_u'); 
   end
end %if ~isempty(intersect(.))

if ~isempty(intersect(typeTPM,[1,2])) %1: use sparse p_ij_u matrix
    for id_uk=1:length(u_grid)
        if ~isempty(union(find(p_ij_u_sparse{id_uk}(:)<0),find(p_ij_u_sparse{id_uk}(:)>1)))
            error('ctdr: Please check values inside p_ij_u');
        end
    end %for
end %if ~isempty(intersect(.))

SSPP_1D_vehicle_displayMsgOnScreen('\nSuccessfully checked that all p_ij_u values (elements) are \\in [0,1]\n');  

%{
%% V&V1 p_ij(u): check (by visual inspection) the property that the sum of elements on each line of p_ij_u should be 1: show content of each line of matrix p_ij_u
if ~isempty(intersect(typeTPM,[0,2])) %0: use regular p_ij_u matrix
    %choose
    idx_u = 1;
    for i=1:size(p_ij_u,1)
        aux = p_ij_u(i,find(~isnan(p_ij_u(i,:,idx_u))), idx_u);
        if ~isempty(aux)
            isPrintedOnce = false;
            for value = aux
                if ~isPrintedOnce
                    fprintf("i=%d: ",i);
                    isPrintedOnce = true;
                end %if
                fprintf("%.2f ",value);
            end %for
            fprintf("\n");
        end %if ~isempty(.)
    end %for
end %if ~isempty(intersect(.))

if ~isempty(intersect(typeTPM,[1,2])) %1: use sparse p_ij_u matrix
        %WIP
end %if intersect(.)
%}

%% V&V2: check equality between the regular vs sparse matrices: check whether p_ij_u == p_ij_u_sparse  
if ~isempty(intersect(typeTPM,[2]))
    disp('Check equality between the regular matrix vs sparse matrix:');
    for id_uk=1:length(u_grid)
        %for comparison puspose: remove nan from p_ij_u(:,:,id_uk)
        A_slice = p_ij_u(:,:,id_uk);
        A_slice(find(isnan(A_slice))) = 0.0; %V&V:  A_slice(1:30,1:10)
        %p_ij_u(:,:,id_uk) = A_slice; %overwrite
        
        %{
        %var1
        temp = (p_ij_u_sparse{id_uk} == A_slice) ; %V&V:  full(p_ij_u_sparse{id_uk}(1:10,1:10))
        find(~temp)
        %}
        
        % %{-
        %var2
        if full(all(p_ij_u_sparse{id_uk} == A_slice,'all'))
            SSPP_1D_vehicle_displayMsgOnScreen('\n slice id_uk=%d: ok (equality confirmed)',id_uk) ; 
        end %if
        % %}
    end %for
    SSPP_1D_vehicle_displayMsgOnScreen('\n'); 
end %if ~isempty(intersect(.))

%% V&V3: check p_tt(u)=1 \forall u \in U(t) necessary for Stoch Short Path Pb cf [B2005Bertsekas, p405]    
if ~isempty(intersect(typeTPM,[0,2])) %0: use regular p_ij_u matrix
    %identify those u belonging to the set of admissible cdes U(t)  (i.e. u \in U(t))  by looking at associated row content inside matrix p_ij_u
    for idx_u = 1:length(u_grid)
        %check if this u=u_grid(idx_u) is a feasible cde starting-from/associated-to state t  i.e.  u \notin U(t)
        if all(isnan(p_ij_u(t,:,idx_u))) %then this u=u_grid(idx_u) is not a feasible cde starting-from/associated-to state t  i.e.  u \notin U(t)
            %do nothing
        else %then this u=u_grid(idx_u) is a feasible cde starting-from/associated-to state t  i.e.  u \in U(t)
            % p_tt(u) =should-be=1 and p_{t,iprime}(u)=should-be=nan
            if ( p_ij_u(t,t,idx_u) ~= 1 || ~all(isnan(p_ij_u(t,jprime_grid,idx_u))) )
                error('\n ctdr: Please enforce condition p_tt(u)=1 \forall u \in U(t)');
            end %if
        end %if
    end %for idx_u =
    
    %once this line of code was reached it means that this check was successful
    SSPP_1D_vehicle_displayMsgOnScreen('Successful check p_tt(u)=1 \forall u \in U(t)'); 
end %if ~isempty(intersect(.))


%{
%% V&V4: check Assumption7.2.1
for idx_iprime = 1:length(iprime_grid)
    idx_i = find(i_grid == iprime_grid(idx_iprime)) ; %idx_i associated to i_grid     
    
    rho_pi(idx_i) = max(p_ij_u(idx_i,:,));
    
end %for idx_iprime
%}


%}

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% %%%%%%%%% Utility functions %%%%%%%%%%%%

