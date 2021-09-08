function loadVarsFromBaseWS(varargin)
% Load variables from 'base' workspace into a function's own 'caller' workspace. 
% Usage: e.g., given a,b,c matlab variables declared into 'base' WS, call inside any function: 
% loadVarsFromBaseWS a b c

%code cf. http://www.mathworks.com/matlabcentral/answers/196466-how-to-load-many-variables-form-base-environment#answer_174352 -> post by "per isakson"  
%code similar to alternatives.m 

%Example: 
%  a=1; b=magic(3); c=dir; testLoadVarsFromBaseWS a b c;   see http://www.mathworks.com/matlabcentral/answers/196466-how-to-load-many-variables-form-base-environment#answer_174352 -> post by "per isakson"    

try
    for jj = 1 : length( varargin )
        val = evalin( 'base', varargin{jj} );
        assignin( 'caller', varargin{jj}, val )
    end %for jj =..
catch
    fprintf( 2, '!! File loadVarsFromBaseWS.m : Something went wrong\n' ) %#ok<PRTCAL>
    error('eom'); % error(.) is necessary iot capture the error effect by an on outer try/catch statement 
end %try

end %function loadVarsFromBaseWS(.)

