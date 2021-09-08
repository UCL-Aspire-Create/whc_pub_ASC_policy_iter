function linkaxesall(option)
% [WIP]
% Execute linkaxes(.,option) over ALL opened figures, including their subplots.

% Inputs:
%   option     1 x 1     string value, can be 'x', 'y' or 'xy', namely the same options allowed for linkaxes.m  

% Examples:
% linkaxesall('x') this is the default action if no argument is specified at all, i.e. linkaxesall() 


%% set default 
if ~exist('option','var')
    option = 'x';
end

%% Main    
all_ha = findobj('type', 'axes', 'tag', '' );
linkaxes( all_ha, option );