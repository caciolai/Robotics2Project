%% CONFIGURATION FILE FOR 2R PLANAR UNDER GRAVITY
%% DYNAMIC PARAMETERS
m = 4*ones([n 1]); % masses
l = 0.4*ones([n 1]); % link lengths
d = 0.3*ones([n 1]); % distance (>0) of the center of mass from O_RF_i
I = zeros(3,3,n); % the n inertia matrices
for k=1:n
    I(:,:,k) = 0.4*eye(3);
end
rg = 50*ones([n 1]); % the reduction ratios for the gears of the motors
Im_zz = 0.015*ones([n 1]); %the inertia matrix of the motor, only zz component 
k_el = 20000*ones([n 1]); % the elastic constants

%% INITIAL STATE OF THE SYSTEM
x0 = [-pi/2 0   , 0 0 , -pi/2 0 , 0 0];%initial state
x0_hat = [x0(1) x0(2) , 0 0   , 0 0  ,   0 0];%initial state for the observers
%% CONTROLLER AND OBSERVER PARAMETERS
q_fin = [pi/2 0];%position to regulate for the rest to rest motion
%% CONTROLLER AND OBSERVER PARAMETERS
%TOMEI OBSERVER PARAMETERS
A_t  = 2000*eye(n);
Q_t  = 2000*eye(n);
K1_t = lyap(A_t,Q_t);
K2_t = 2000*eye(n);
Ka_t = -10000*eye(n);
%CICCARELLA PARAMETERS;
modes_c = -linspace(300,1000,4*n); %WORKING, LESS OSCILLATIONS
% modes_c = -linspace(250, 260, 4*n); % WORKING, SLOWER DYNAMICS BUT MORE OSCILLATIONS
%CONTROLLER 1 PARAMETERS
modes_ctr1 = 5*[-1 -2 -3 -4 -5 -6 -7 -8];
Q_ctr = 100*eye(4*n);
%INTEGRAL MANIFOLD CONTROLLER PARAMETERS
Kp = 7000*diag(ones(1,n));
Kd = 7000*diag(ones(1,n));
modes_ctr2 = 70*[-1 -1.2 -1.3 -1.4];

