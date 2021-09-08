function y = mcv(x)
% makeColumnVect: The output y is a column vector that retains the same succession of elements
% as the input vector x. 
% Special cases:
% 1. [handling isempty] if x=[] then y returns y=[] 
% 2. [handling nan] nan are treated like any regular (e.g. numerical) data 
%
% Example:
%   mcv([]) 
%   mcv([1 2 3]) 
%   mcv(nan(1,3)) 

y = x; %ini

if isempty(x)
    return; %i.e. keep ini info
end
%otherwise continue hereafter

if ~isvector(x)
    error('!! ctdr: Please provide a vector input !!');
end %if length(size(x)) >..

if ~iscolumn(x) 
    y = x';
end