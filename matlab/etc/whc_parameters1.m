function P = wc_parameters1()


%% %%%%%%%%% CHAPTER I. Specify/calculate parameters of 'nominal' plant model %%%%%%%%%
%% Outside environment
ga = 9.81;  %[m/s^2]   gravitational acceleration 

%% wc: link1:= right drive wheel; link2 := left drive wheel
mw = 2.6; %[kg] mass of one drive wheel
r = inch2m(13)/2 ; %[m] drive wheels radius

Iw_zz = 0.5*mw*power(r,2); %[kg*(m^2)] mass moment of inertia of the disk about the axis perpendicular to the plane of the disk, passing through the center of the disk



%% wc: link3 := wc body+driver sitting in fixed position
lw = 0.55;         %[m] length between the two drive wheels along the shaft that interconnects them

mb = 120 + 80; %[kg] mass of wc body (wo drive wheels; including batteries(~40kg), metalic frame as well as item24, seat, electronics ) + wc driver 
Izz = (mb/12)*(power(0.45,2)+power(0.25,2)); %[kg*(m^2)] [*] formula Uniform Rectangular Solid cf [SpHuVi2005,p.253]

hCM3 = 0.40; %[m] [*] 

%% pulleys
pitch_bigPulley_rightMainWh = 0.0025; %[m]
N_smallPulley_rightMainWh = 30; %[nr-teeth] small pulley attached to encoder
N_bigPulley_rightMainWh = 110; %[nr-teeth] [*] 3D printed big pulley attached to drive wheel

pitch_bigPulley_leftMainWh = 0.002; %[m]
N_smallPulley_leftMainWh = 36; %[nr-teeth] small pulley attached to encoder
N_bigPulley_leftMainWh  = 153; %[nr-teeth] 3D printed big pulley attached to drive wheel

%% encoders 
N_oneRot = 500*4; %%[counts in one rotation] 4 comes from the quadrature

%% motors + associated gearboxes
rG = 26; %[-]

%%% values below are copy-paste from the manufactured model "SP+ 075 MF 1-stage ratio=7" from file "Wittenstein gearboxes SP+SP+ high speed.pdf"         
etaG = 0.97; %[-]
Jg = .13*1e-4; %[kg*(m^2)] gear moment of inertia 
Bg = 7.383476741376615e-05; %[Nm*s/rad] lumped damping coefficient of all succesive gears: it should be experimentally calculated based on input-output relation from the input G1p --> the output G3s; in other words all gears seen as one single input-output block

%%% values below are copy-paste from Maxon EC60-167132 specs  
Ja = 831*1e-7; %[kg*(m^2)] actuator moment of inertia, terminology used by [SpHuVi2006book,pp.208]; 
Ba = 1.106646108604682e-04; %[Nm*s/rad] rotational viscous damping coefficient of motor;

%%% conclude
Jm = Ja + Jg; %[kg*(m^2)] cumulated moment of inertia of actuator (i.e. motor) + all successive gears
Bm = Ba + Bg; %[Nm*s/rad] cumulated friction coefficient


%% def: Profile ADAPT joystick+PM: max characteristics
Profile_vmax     = mph2mps(1.2); %[m/s]
Profile_omegamax = 2*mph2mps(.8)/lw ; %[rad/s] I applied formula angular velo dotphi=(P.r/P.lw)*(dotpsi1-dotpsi2), given that at omegamax: dotpsi2=-dotpsi1, and I make assumption that what joy screen shows is max(|vlin_wh1|,|vlin_wh2|) => dotpsi1-dotpsi2=2*dotpsi1=2*mph2mps(vlin_wh1)/P.r   
Profile_vmin     = mph2mps(-0.8); %[m/s]
%V&V:   whc_saturV([0,0], Profile_vmax,Profile_omegamax,Profile_vmin, 1)  

%% derived qtts/parameters
%%% Start: copy-paste wc_demo2_rewritten_eqs_sysdyn_v0.slx -> Model5
% def 
a = ( mw+0.25*mb+Izz/power(lw,2) )*power(r,2) + Iw_zz+etaG*Jm*power(rG,2) ;
b = 0.25*mb*power(r,2)-Izz*power(r/lw,2);
c = etaG*Bm*power(rG,2);
d = (mw+mb/2)*ga*r;
e = etaG*rG;

%conseq
xi0 = power(b,2)-power(a,2);
xi1 = a*c/xi0;
xi2 = -b*c/xi0;
xi3 = d/(a+b);
xi4 = -a*e/xi0;
xi5 = b*e/xi0;

%% def: passage velocity vector V(t)=(v,omega) <-> dotpsi(t)
A    = r*[.5 .5; 1/lw -1/lw];
Ainv = [1 lw/2; 1 -lw/2]/r;


%% ultrasonic sensors param/settings
US_draymax = 2.838; %[m] cf calibration, namely chosen (paramUS_range_hex,paramUS_analogueGain_hex): ultrasonic sensor max firing beam 
US_draymin = 0.040; %[m] cf datasheet
US_deltaThetaWidth = deg2rad(45); %[rad]


%% %%%%%%%%% CHAPTER II. specify output variable P
P.ga = ga;

P.mw = mw;
P.r = r;
P.Iw_zz = Iw_zz;

P.lw = lw;
P.mb = mb;
P.Izz = Izz;

P.hCM3 = hCM3;

P.N_oneRot = N_oneRot;

P.rG = rG;
P.etaG = etaG;
P.Jm = Jm;
P.Bm = Bm;

P.xi0 = xi0;
P.xi1 = xi1;
P.xi2 = xi2;
P.xi3 = xi3;
P.xi4 = xi4;
P.xi5 = xi5;

P.Profile_vmax = Profile_vmax;
P.Profile_omegamax = Profile_omegamax;
P.Profile_vmin = Profile_vmin;

P.US_draymax = US_draymax;
P.US_draymin = US_draymin;
P.US_deltaThetaWidth = US_deltaThetaWidth;

%def
P.A    = A;
P.Ainv = Ainv; 

end

