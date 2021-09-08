function SSPP_1D_vehicle_displayMsgOnScreen(varargin)
loadVarsFromBaseWS doDisplayMsgOnScreen

if doDisplayMsgOnScreen
    switch nargin
        case 1
            str = varargin{1};
            fprintf(str);
        case 2
            str = varargin{1};
            val = varargin{2};
            fprintf(str,val);
    end %switch
end %if

end %function