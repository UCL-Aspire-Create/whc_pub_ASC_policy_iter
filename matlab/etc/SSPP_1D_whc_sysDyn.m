function xkp1 = SSPP_1D_whc_sysDyn(xk,wk,uk, deltat)
% System dynamics at next step (i.e. next sampling time): cf eq(2) art_whc3-v4.pdf  
% System in discrete-time:  x1kp1 = sat_[x1_min,x1_max] x1k + x2k*deltat;            where x1_min,x1_max correspond to locations of walls (bumpers): think of a toy car moving inside a delimited circuit 
%                           x2kp1 = sat_[x2_min,x2_max] sigma1*x2k + sigma2*(uk+wk); where x2_min,x2_max correspond to the action of the low-level safety block bypassing any control, and instead kicking in and ensuring velocity-limitations are met (e.g. vmax=6mph for whc) 
% V&V:  xkp1 = SSPP_1D_whc_sysDyn([0;0],0,.53, .1)

%% check input data consistency
if (length(xk) ~=2)
    error('ctdr: Please check xk');
end %if

%% ensure data consistent format
xk = mcv(xk);

%% main
loadVarsFromBaseWS  x1_min x1_max  x2_min x2_max  sigma1 sigma2  vmin vmax;
P = whc_parameters1() ;

A = [1 deltat; ...
     0 sigma1];
g = [0; sigma2];

%unsaturated dyna
xkp1 = A*xk + g*max(vmin,min(vmax,uk+wk));

%apply saturation (physical constraints)
xkp1(1) = max(x1_min,min(x1_max,xkp1(1))) ; %V&V: xkp1(1)=-10; xkp1(1)=10;  
xkp1(2) = max(x2_min,min(x2_max,xkp1(2))) ; %V&V: xkp1(2)=-10; xkp1(2)=10;  

end %function
