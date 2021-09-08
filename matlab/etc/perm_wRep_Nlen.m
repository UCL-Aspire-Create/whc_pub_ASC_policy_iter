function M = perm_wRep_Nlen(Nlen) % by ctdr
% compute all permutations with repetition; Nlen is a vector with stack's max capacity at each position in the stack 
% ex: M = perm_wRep_Nlen([4 3 2]): the stack has 3 positions [_ _ _] and can hold all permutations between [1 1 1] to max capacity which is [4 3 2]  
%

%ini
slider = 1; %represents position (i.e. index) inside the stack
stack = zeros(1,length(Nlen));
M = nan(prod(Nlen),length(Nlen)); 
id_M = 1;

while (slider > 0)  

    if stack(slider) < Nlen(slider) %then stay on that level and augment stack's value  
        %augment value in the stack
        stack(slider) = stack(slider) + 1;
        
        %advance in the stack
        if slider < length(Nlen)
           slider = slider + 1; 
        end
        %otherwise i.e. if slider==length(Nlen) stay on the same level (the last level)
    else %i.e. if stack(slider) == Nlen(slider) %then drop level in stack  
        %re-ini stack's content
        stack(slider) = 0;
        
        slider = slider-1;
    end %if stack(.)
    
    %store
    if ((slider == length(Nlen)) ... %if we're on the last level
        && (stack(slider) ~= 0))
        M(id_M,:) = stack(:);
    
        %update
        id_M = id_M+1;
    end %if (.)
    
end %while