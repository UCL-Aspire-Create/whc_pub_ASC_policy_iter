function y = mrv(x)
% makeRowVect: The output y is a row vector that retains the same succession of elements
% as the input vector x. 
% Special cases:
% 1. [handling isempty] if x=[] then y returns y=[] 
% 2. [handling nan] nan are treated like any regular (e.g. numerical) data 
%
% Example:
%   mrv([]) 
%   mrv([1; 2; 3]) 
%   mrv(nan(3,1)) 

y = x; %ini

if isempty(x)
    return; %i.e. keep ini info
end
%otherwise continue hereafter

if ~isvector(x)
    error('!! ctdr: Please provide a vector input !!');
end %if length(size(x)) >..

if ~isrow(x)
    y = x';
end