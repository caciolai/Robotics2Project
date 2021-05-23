clear;
clc;
%% PARAMETERS FOR THE SIMULATION
%Set n = 2 to simulate a 2R planar under gravity
%Set n = 3 to simulate a 3R spatial
n = 3;
clear_funs = true;%set to true if want to recompute all the functions for running the simulation. SET TO TRUE IF MODEL HAS BEEN MODIFIED
simplify_all = false; %set to true if want to apply the simplify function to all symbolic terms computed, may require some time...
optimize_export = false;%set to false to disable code optimization (will spped up the exportfuns time...)
% REST-TO-REST trajectory
sim_time = 10;%len of simulation
traj_time = sim_time*0.5; % Time to reach final pose with
%% SCRIPT OF INIT FOR SETTING THE PATHS FOR SIMULATION
init;
%% LOAD THE MODEL WITH THE CHOOSEN SIMULATION PARAMETERS
switch n
    case 2
        config_2R;
        model_2R;
    case 3
        config_3R;
        model_3R;
    otherwise
        error("Not implemented!");
end
%% COMPUTATION OF THE DESIRED JOINT TRAJECTORY (REGULATION TASK)
% Defined in terms of links' position (yd1) ,velocity (yd2),
% acceleration (yd3) , jerk(yd4) and jounce (dyd4).
[yd1, yd2, yd3, yd4, dyd4] = my_spline(x0(1:n), q_fin, traj_time);
t = linspace(0,sim_time)';
%Computing the desired trajectory to use by controller
yd1 = cell2mat(arrayfun(yd1,t(:),'UniformOutput',false));
yd2 = cell2mat(arrayfun(yd2,t(:),'UniformOutput',false));
yd3 = cell2mat(arrayfun(yd3,t(:),'UniformOutput',false));
yd4 = cell2mat(arrayfun(yd4,t(:),'UniformOutput',false));
dyd4 = cell2mat(arrayfun(dyd4,t(:),'UniformOutput',false));
yd = [t yd1(:,1:n) yd2(:,1:n) yd3(:,1:n) yd4(:,1:n) dyd4(:,1:n)];
%% COMPUTE ALL THE STUFF FOR THE CONTROLLERS AND THE OBSERVERS
computegains;
%% EXPORT ALL THE REQUIRED FUNCTIONS FOR SIMULINK
if clear_funs
    exportfuns;
end
%% OPEN SIMULINK MODEL
disp("Opening the simulink model...");
tic;
open_system('simulink_model.slx');
toc;
%% EXECUTING CLOSE STEPS
close;