%% Simscape multibody model og Regbot in balance

% initial setup with motor velocity controller 
% this is intended as simulation base for balance control.

close all
clear


%% Initialise variables used in Simulink

% parameters for tilt stabilisation
w_i_post = 1;
tau_i_post = 1;

% parameters for tilt controller
alpha = 1;
K_P = 1;
tau_i = 1;
tau_d = 1;

% parameters for velocity controller
alpha2 = 1;
K_P2 = 1;
tau_i2 = 1;
tau_d2 = 1;

% parameters for position controller
alpha3 = 1;
K_P3 = 1;
tau_i3 = 1;
tau_d3 = 1;


%% Simulink model name

model = 'regbot_1mg';

%% Parameters for REGBOT

% motor
RA = 3.3/2;    % ohm (2 motors)
JA = 1.3e-6*2; % motor inertia
LA = 6.6e-3/2; % rotor inductor (2 motors)
BA = 3e-6*2;   % rotor friction
Kemf = 0.0105; % motor constant
Km = Kemf;

% køretøj
NG = 9.69; % gear
WR = 0.03; % wheel radius
Bw = 0.155; % wheel distance

% model parts used in Simulink
mmotor = 0.193;   % total mass of motor and gear [kg]
mframe = 0.32;    % total mass of frame and base print [kg]
mtopextra = 0.97 - mframe - mmotor; % extra mass on top (charger and battery) [kg]
mpdist =  0.10;   % distance to lit [m]

% disturbance position (Z)
pushDist = 0.1; % relative to motor axle [m]


%% Wheel velocity controller (no balance) PI-regulator

% sample (usable) controller values
Kpwv = 15;     % Kp
tiwv = 0.05;   % Tau_i
Kffwv = 0;     % feed forward constant
startAngle = 10;  % tilt in degrees at time zero
twvlp = 0.005;    % velocity noise low pass filter time constant (recommended)


%% Estimate transfer function for base system using LINEARIZE

% Motor voltage to wheel velocity (wv)
load_system(model);
open_system(model);
% define points in model
ios(1) = linio(strcat(model,'/Limit9v'),1,'openinput');
ios(2) = linio(strcat(model, '/wheel_vel_filter'),1,'openoutput');
% attach to model
setlinio(model,ios);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys_tilt = linearize(model,ios,op);
% get transfer function
[num,den] = ss2tf(sys_tilt.A, sys_tilt.B, sys_tilt.C, sys_tilt.D);
Gwv = minreal(tf(num, den))


%% Bodeplot

h = figure(100);
bode(Gwv)
grid on
title('Transfer function from motor voltage to velocity')
% saveas(h, 'motor to velocity.png');


%% TILT CONTROLLER (TO BE STABILISED)


% Reference velocity to tilt angle
load_system(model);
open_system(model);
% define points in model
ios_tilt(1) = linio(strcat(model,'/vel_ref'),1,'openinput');
ios_tilt(2) = linio(strcat(model, '/tilt_angle'),1,'openoutput');
% attach to model
setlinio(model,ios_tilt);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys_tilt = linearize(model,ios_tilt,op);
% get transfer function
[num,den] = ss2tf(sys_tilt.A, sys_tilt.B, sys_tilt.C, sys_tilt.D);
G_tilt = minreal(tf(num, den))


h = figure(910);
nyquist(G_tilt)
% 
% h = figure(110);
% bode(G_tilt);
% grid on
% title('Transfer function from vel ref to tilt angle')
% [mag,w]=bode(G_tilt);
% [pks,locs]= findpeaks(squeeze(mag));
% peak_mg = pks(1);
% peak_freq = w(locs(1));
% saveas(h, 'vel ref to tilt angle.png');
% 
% gain_freq = mag(find(mag == peak_mg, 1));
% display(gain_freq);


% GET w_i_post FROM BODE PLOT FIRST!!
w_i_post = 8.38;
tau_i_post = 1 / w_i_post;


%% TILT CONTROLLER (STABILISED)


% Vel ref 2 to tilt angle
load_system(model);
open_system(model);
% define points in model
ios_tilt_s(1) = linio(strcat(model,'/vel_ref2'),1,'openinput');
ios_tilt_s(2) = linio(strcat(model, '/tilt_angle'),1,'openoutput');
% attach to model
setlinio(model,ios_tilt_s);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys_tilt_s = linearize(model,ios_tilt_s,op);
% get transfer function
[num,den] = ss2tf(sys_tilt_s.A, sys_tilt_s.B, sys_tilt_s.C, sys_tilt_s.D);
G_tilt_s = minreal(tf(num, den));

h = figure(911);
nyquist(G_tilt_s)

h = figure(111);
bode(G_tilt_s);
grid on
title('Transfer function from vel ref 2 to tilt angle')


% Designing tilt controller

N_i = 2;
alpha = 0.07;

w_c = 65.6;  % new desired crossover frequency (read from old bode plot 111)

tau_i = N_i / w_c;
tau_d = 1/((w_c*(alpha)^0.5));

K_P = 1.5;


% Ref angle to tilt angle
load_system(model);
open_system(model);
% define points in model
ios_tilt_pid(1) = linio(strcat(model,'/ref_angle'),1,'openinput');
ios_tilt_pid(2) = linio(strcat(model, '/tilt_angle'),1,'openoutput');
% attach to model
setlinio(model,ios_tilt_pid);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys_tilt_pid = linearize(model,ios_tilt_pid,op);
% get transfer function
[num,den] = ss2tf(sys_tilt_pid.A, sys_tilt_pid.B, sys_tilt_pid.C, sys_tilt_pid.D);
G_tilt_pid = minreal(tf(num, den))

pole(G_tilt_pid)


h = figure(911);
nyquist(G_tilt_pid)

h = figure(112);
bode(G_tilt_pid);
grid on
title('Transfer function from ref angle to tilt angle')

h = figure(510);
step(G_tilt_pid)


%% VELOCITY CONTROLLER


% Ref angle to velocity
load_system(model);
open_system(model);
% define points in model
ios_vel(1) = linio(strcat(model,'/ref_angle'),1,'openinput');
ios_vel(2) = linio(strcat(model, '/velocity'),1,'openoutput');
% attach to model
setlinio(model,ios_vel);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys_vel = linearize(model,ios_vel,op);
% get transfer function
[num,den] = ss2tf(sys_vel.A, sys_vel.B, sys_vel.C, sys_vel.D);
G_vel = minreal(tf(num, den))

h = figure(920);
nyquist(G_vel)

h = figure(120);
bode(G_vel);
grid on
title('Transfer function from ref angle to velocity')


% Designing velocity controller

N_i2 = 2;
alpha2 = 0.1;

w_c2 = 7.98;  % new desired crossover frequency (read from old bode plot 120)

tau_i2 = N_i2 / w_c2;
tau_d2 = 1/((w_c2*(alpha2)^0.5));

K_P2 = 0.4;


% Ref velocity to velocity
load_system(model);
open_system(model);
% define points in model
ios_vel_c(1) = linio(strcat(model,'/ref_vel'),1,'openinput');
ios_vel_c(2) = linio(strcat(model, '/velocity'),1,'openoutput');
% attach to model
setlinio(model,ios_vel_c);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys_vel_c = linearize(model,ios_vel_c,op);
% get transfer function
[num,den] = ss2tf(sys_vel_c.A, sys_vel_c.B, sys_vel_c.C, sys_vel_c.D);
G_vel_c = minreal(tf(num, den))


h = figure(921);
nyquist(G_vel_c)

h = figure(121);
bode(G_vel_c);
grid on
title('Transfer function from ref vel to velocity')

h = figure(520);  % plot after closing loop
step(G_vel_c);


%% POSITION CONTROLLER


% Ref velocity to x position
load_system(model);
open_system(model);
% define points in model
ios_pos(1) = linio(strcat(model,'/ref_vel'),1,'openinput');
ios_pos(2) = linio(strcat(model, '/x_pos'),1,'openoutput');
% attach to model
setlinio(model,ios_pos);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys_pos = linearize(model,ios_pos,op);
% get transfer function
[num,den] = ss2tf(sys_pos.A, sys_pos.B, sys_pos.C, sys_pos.D);
G_pos = minreal(tf(num, den))

h = figure(930);
nyquist(G_pos)

h = figure(130);
bode(G_pos);
grid on
title('Transfer function from ref velocity to x position')


% Designing position controller

% N_i3 = 1;
alpha3 = 0.8;

w_c3 = 2.6;  % new desired crossover frequency (read from old bode plot 130)

% tau_i3 = N_i3 / w_c3;
tau_d3 = 1/((w_c3*(alpha3)^0.5));

K_P3 = 1.2;


% Ref position to x position
load_system(model);
open_system(model);
% define points in model
ios_pos_c(1) = linio(strcat(model,'/ref_pos'),1,'openinput');
ios_pos_c(2) = linio(strcat(model, '/x_pos'),1,'openoutput');
% attach to model
setlinio(model,ios_pos_c);
% Use the snapshot time(s) 0 seconds
op = [0];
% Linearize the model
sys_pos_c = linearize(model,ios_pos_c,op);
% get transfer function
[num,den] = ss2tf(sys_pos_c.A, sys_pos_c.B, sys_pos_c.C, sys_pos_c.D);
G_pos_c = minreal(tf(num, den))


h = figure(931);
nyquist(G_pos_c)

h = figure(131);
bode(G_pos_c);
grid on
title('Transfer function from ref pos to x pos')

h = figure(530);  % plot after closing loop
step(G_pos_c);
grid on


