clc;clear all;close all
%% startup:
addpath('Env')
addpath('Algos')
addpath('Policies')

run('Env\globalParameters')

%% set env:
env = TargetAndTrackingSystem();

%% set policy:
policy = LinearGainPolicy('ObDim',2,'BufferSize',1);

%% set learning algorithm:

algo =  REINFORCE('env',env,...
                  'policy',policy,...    
                  'n_itr',5e3,...
                  'step_size',25,...
                  'sigma',10,...
                  'discount',1,...
                  'max_grad',0.02,...
                  'render',false,...
                  'n_of_workers',4);
        
theta0 = [0,0];              
algo.init_training(theta0);

%% Estimate J  (Brute Force estimeation in the 2D case, for illustration purpases only):
%algo.EstimateJ()

%% learn!      
algo.train()

%% cleanup:
rmpath('Env')
rmpath('Algos')
rmpath('Policies')