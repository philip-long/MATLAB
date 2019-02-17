clear all,clc,close all
%
% Initial with several different conditions. Check convergence wp versus
% wpstar. Check convergence rate with Multistart

addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/GeometricFunctions'))
addpath(genpath('/media/philip/39C2CB4B4DF25122/MatlabFunctions/MATLAB/UtilityFunctions'))
addpath(genpath('/home/philip/tbxmanager'))
%% 

solver_name='interior-point';
%solver_name='sqp';

TOTAL_SUCCESS_WP=[];
TOTAL_SUCCESS_WPSTAR=[];
WP_FLAG=[];
WP_GOOD_SOL=[];
WPSTAR_FLAG=[];
WPSTAR_GOOD_SOL=[];
%for object_case=1:2
try
parpool('local');
catch
end

for test_no=1:30
    
    object_case=randi(2);
    
    constrained_manipulability=false;
    initial_condition=[randRange(0.5,0.7,8)',randRange(-0.1,0.2,8)',randRange(-pi/8,pi/4,8)'];
    try
    [exit_flag_wp,good_solution_wp]=IKOptimization(constrained_manipulability,...
                                                            object_case,...
                                                            initial_condition,...
                                                            solver_name);
    constrained_manipulability=true;
    [exit_flag_wpstar,good_solution_wpstar]=IKOptimization(constrained_manipulability,...
                                                                    object_case,...
                                                                    initial_condition,...
                                                                    solver_name);
    
    WP_FLAG=    [WP_FLAG    ;exit_flag_wp];    
    WPSTAR_FLAG=[WPSTAR_FLAG;exit_flag_wpstar];
    
    WP_GOOD_SOL=    [WP_GOOD_SOL     ;good_solution_wp];
    WPSTAR_GOOD_SOL=[WPSTAR_GOOD_SOL;good_solution_wpstar];
    
    catch
        disp ' Something went wrong'
    end            
end

save('convergence_test4');
%%
system('systemctl poweroff');
