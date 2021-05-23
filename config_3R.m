%% CONFIGURATION FILE FOR 3R SPATIAL ELBOW-TYPE
%% KINEMATIC AND DYNAMIC PARAMETERS
m = 4*ones([n 1]); % masses
l = 0.4*ones([n 1]); % link lengths
d = 0.3 * ones([n 1]); % distance of the center of mass from O_RF_i
I = zeros(3,3,n); % the n inertia matrices
for k=1:n
    I(:,:,k) = 0.4*eye(3);
end
rg = 50*ones([n 1]); % the reduction ratios for the gears of the motors
Im_zz = 0.035*ones([n 1]); %the inertia matrix of the motor, only zz component 
k_el = 20000*ones([n 1]); % the elastic constants

%% INITIAL STATES

% Initial state for the system
x0 = [0 pi/2 0 , 0 0 0 , 0 pi/2 0 , 0 0 0];
%x0 = [-pi pi -pi , -4 5 10 , 1-.5*pi pi/2 -pi , 6 10 -20];
% Initial state for the observers
x0_hat = [x0(1) x0(2) x0(3) , 0 0 0 , 0 0 0 , 0 0 0];
%x0_hat = [0 0 0 , 0 0 0 , 0 0 0 , 0 0 0];

%% FINAL STATE FOR REGULATION
q_fin = [pi/2 pi 0];

%% CONTROLLER AND OBSERVER PARAMETERS

% TOMEI OBSERVER PARAMETERS
A_t  = 2000*eye(n);
Q_t  = 2000*eye(n);
K1_t = lyap(A_t,Q_t);
K2_t = 2000*eye(n);
Ka_t = -10000*eye(n);

% CICCARELLA PARAMETERS;
modes_c = -linspace(250, 260, 4*n); 

% SPONG FEEDBACK LINEARIZING CONTROLLER PARAMETERS
modes_ctr1 = 5*[-1 -2 -3 -4 -5 -6 -7 -8 -9 -10 -11 -12];
Q_ctr = 100*eye(4*n);

% SPONG INTEGRAL MANIFOLD CONTROLLER PARAMETERS
Kp = 7000*diag(ones(1,n));
Kd = 7000*diag(ones(1,n));
modes_ctr2 = 70*[-1 -1.2 -1.3 -1.4 -1.5 -1.6];