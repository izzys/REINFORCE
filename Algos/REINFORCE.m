classdef REINFORCE < handle
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% REINFORCE algorithm with optimal baseline estimation.
%   
% Theoretical background based on:
% J. Peters and S. Schaal, (2008). "Reinforcement learning of motor skills with policy gradients," Neural Networks, 21, 4, pp.682-97.
%
%
% Adopted from an implemention in:
% Implementation and Optimization of an Open Loop Gait Controller on a Mono Pedal Robot
% M.Sc. Thesis by Israel Schallheim
% https://www.researchgate.net/publication/329681688_Implementation_and_Optimization_of_an_Open_Loop_Gait_Controller_on_a_Mono_Pedal_Robot
%
% Written by: Israel Schalheim
% Last edited: 19.01.19
% Version: 1.00
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    %% Properties:
    properties
        env;
        policy;
        
        % some options and their default value:
        sigma     = 0.1;
        n_itr     = 1000;
        discount  = 0.99;
        step_size = 0.01;
        render    = false;
        max_grad  = 0.01;
        
        % handles to stats figure
        hStats;
        
        % Logger struct
        Logger;
        
        % Parallel computations setup
        n_of_workers = 4;
        
        % Version control;
        Version = 1.00;
        
        % Stop training:
        StopTraining = false;
    end
    %% end of properties
       
    %% Methods:
    methods
        
        %% REINFORCE:
        function obj = REINFORCE(varargin)
                                   
            % count arguments
            nArgs = length(varargin);
            if round(nArgs/2)~=nArgs/2
               error('REINFORCE needs propertyName/propertyValue pairs')
            end  
            
            if nargin<4
               error('REINFORCE needs at least 2 inputs (env and policy)') 
            end
            
            for pair = reshape(varargin,2,[]) % pair is {propName;propValue}
               inpName = lower(pair{1}); % make case insensitive

               switch inpName 
                    case 'env'
                        obj.env = pair{2};
                    case 'policy'
                         obj.policy = pair{2}; 
                    case 'n_itr'
                        obj.n_itr = pair{2};
                    case 'discount'
                         obj.discount = pair{2};                       
                    case 'step_size'
                         obj.step_size = pair{2};   
                    case 'max_grad'
                         obj.max_grad = pair{2};
                    case 'sigma'
                         obj.sigma = pair{2};                          
                    case 'render'
                         obj.render = pair{2};  
                    case 'n_of_workers'
                         obj.n_of_workers = pair{2};
                    otherwise
                         error('%s is not a recognized parameter name',inpName)
               end                  
            end          
        end
        %% end of method
        
        %% Run training session: 
        function [] = train(obj)
        
            % initialize:
            theta = obj.policy.theta;
            sig = ones(1,size(theta,2)).*obj.sigma;
            grad  = [];
            J     = [];
            
            % initial Jnom:
            parfor j = 1:obj.n_of_workers
                [~, r{j}, ~] = obj.run_traj(zeros(1,length(sig)));
                 if obj.discount==1
                    Jtmp(j) = sum(r{j})/length(r{j});
                 else
                    Jtmp(j) = sum(r{j});
                 end
            end
            Jnom = mean(Jtmp);
            
            set(obj.hStats.Lines.Jnom,'Xdata',0,'Ydata',Jnom)
       
            pause(0.1)
            disp(['#############################################################'])            
            disp(['Starting REINFORCE algorithm ...'])            
            disp(['initial theta = [' num2str(theta) ']']) 
            disp(['~~~~~~~~~~~~~~~~~~~~~~ start training. ~~~~~~~~~~~~~~~~~~~~~~~~'])         
            pause(0.1)
            
            % Main loop: collect experience in env and update/log each epoch
            for i = 1:obj.n_itr
                
                % estimate gradient:
                [grad(i,:),J(i),trials(i)] = obj.estimate_gradient(theta(i,:),sig);
                grad_clipped = max( -obj.max_grad ,min( obj.max_grad, grad(i,:) ));
                
                % update theta:
                theta(i+1,:) = theta(i,:) + obj.step_size * grad_clipped;
                obj.policy.theta = theta(i+1,:);
                disp(['done iteration #' num2str(i) ' with ' num2str(trials(i)) ' episodes. theta = [' num2str(theta(i+1,:)) ']']) 
                
                % plot the nominal J:
                parfor j = 1:obj.n_of_workers
                    [~, r{j}, ~] = obj.run_traj(zeros(1,length(sig)));  
                    if obj.discount==1
                        Jtmp(j) = sum(r{j})/length(r{j});
                    else
                        Jtmp(j) = sum(r{j});
                    end
                        
                end
                Jnom(i+1) = mean(Jtmp);
                
                set(obj.hStats.Lines.J,'Xdata',cumsum(trials),'Ydata',J)
                set(obj.hStats.Lines.Jnom,'Xdata',[0 cumsum(trials)],'Ydata',Jnom)

                for j = 1:size(theta,2)
                    set(obj.hStats.Lines.Theta(j),'Xdata',[0 cumsum(trials)],'Ydata',theta(:,j)) 
                end
                
                set(obj.hStats.Lines.Grad,'Xdata',cumsum(trials),'Ydata',sum(abs( max( -obj.max_grad ,min( obj.max_grad, grad )) ),2)/size(theta,2))
                
                %clear(obj.hStats.Axes.GradArrows)
                set(obj.hStats.Lines.GradArrows,...
                            'XData',theta(1:end-1,1) ,...
                            'YData',theta(1:end-1,2),... x,y
                            'UData',theta(2:end,1)-theta(1:end-1,1),...
                            'VData',theta(2:end,2)-theta(1:end-1,2) )  
                drawnow

                
                Logger(i).env = obj.env;
                Logger(i).policy = obj.policy;                
                Logger(i).trials = trials(end);
                Logger(i).J = J(end);
                Logger(i).Jnom = Jnom(end);
                obj.Logger(i).grad = grad(end,:);
                Logger(i).sigma = sig(end,:);
                Logger(i).theta = theta(end,:);
                Logger(i).step_size = obj.step_size;
                Logger(i).hFig = obj.hStats.Fig;
                
                
                warning('off','MATLAB:Figure:FigureSavedToMATFile')
                save(['REINFORCE_LearningSession_' obj.hStats.computer_name '_' obj.hStats.date_str '_'  num2str(obj.hStats.Hour) '_' num2str(obj.hStats.Minute) '_' num2str(obj.hStats.Seconds) ],'Logger')
                
                if obj.StopTraining
                    break
                end
            end
            
           
            disp(['~~~~~~~~~~~~~~~~~~~~~~ done trianing. ~~~~~~~~~~~~~~~~~~~~~~~~'])       
            disp(['final theta = [' num2str(theta(end,:)) ']']) 
            disp(['Last session saved as: REINFORCE_LearningSession_' obj.hStats.computer_name '_' obj.hStats.date_str '_'  num2str(obj.hStats.Hour) '_' num2str(obj.hStats.Minute) '_' num2str(obj.hStats.Seconds) '.mat'])
            disp(['To go back and observe this learning session, double click the saved MAT file.']) 
            disp(['###############################################################']) 
            
        end
        %% end of method
        
        %% Gradient estimation
        function [grad,Jout,trials] = estimate_gradient(obj,theta,sigma)
            
            N = 100;
            J_sum = 0;
            sumdJdTheta = 0;
            grad_beffer_size = 3;
            
            sumdLogPi = zeros(1,length(theta));
            sum_b_num = zeros(1,length(theta));
            sum_b_den = zeros(1,length(theta));
            
            trial = 1;
            grad_buffer = zeros(grad_beffer_size,length(theta));
            
            for k = 1:N

                  % get a trajectory with x0 and a policy theta:
                  parfor i = 1:obj.n_of_workers
                      [ob{i}, r{i}, dtheta{i}] = obj.run_traj(sigma);
                      if obj.discount==1
                         R{i} = sum(r{i})/length(r{i});
                      else
                         R{i} = sum(r{i});
                      end
                  end

                  for i = 1:obj.n_of_workers
                        
                        dLogPi = sum( dtheta{i} );%./repmat(sigma.^2,size(dtheta{i},1),1) ,1);
 
                        sumdLogPi = sumdLogPi+dLogPi;
                        sum_b_num = sum_b_num + (dLogPi).^2 * R{i};
                        sum_b_den = sum_b_den +  (dLogPi).^2;

                        b_num = sum_b_num;
                        b_den = sum_b_den;

                        b = b_num./b_den;

                        dJdTheta = dLogPi .* (R{i}*ones(1,length(theta)) - b);
                        sumdJdTheta = sumdJdTheta + dJdTheta;


                        grad = sumdJdTheta/trial;
                        J_sum = J_sum + R{i};
                        Jout = J_sum/trial;

                        trial = trial+1;

                 end

                 if k <3
                       % do nothing. 
                 else
                     
                       for i = 1:grad_beffer_size
                            angle_diff(i)  = obj.GetAngle( grad,grad_buffer(i,:));
                       end
                       angle_diff_avg = mean(angle_diff);
                       

                       if angle_diff_avg<15 || obj.StopTraining
                           trials = trial-1;
                           return
                       end
                 end

                 grad_buffer = [grad ; grad_buffer(1:end-1,:)];
                 
            end

            trials = trial -1;
            disp(['Gradient estimate did NOT converge! exiting after: ' num2str(trials) ' episodes'])
            
        end
        %% end of method
        
        %% Run trajectory:
        function [Ob, R, dTheta] = run_traj(obj,sigma)
            
            % init env and policy:
            Ob = [];
            R = [];
            dTheta = [];
            ob = obj.env.reset(); 
            obj.policy.reset(); 
            
            for i = 1:1e4
    
                [a,dtheta] = obj.policy.sampleAction(ob,sigma);
                [ob,r,done] = obj.env.step(a);

                Ob = [Ob,ob];
                R  = [R,obj.discount^(i-1)*r];
                dTheta = [dTheta;dtheta];
                
                if done
                    ob = obj.env.reset();
                    obj.policy.reset(); 
                    break
                end
                
                if obj.render
                    obj.env.render();
                end
    
            end
            
        end
        %% end of method
        
        %% Init training:
        function init_training(obj,theta0)
            
            obj.policy.theta = theta0;
            
            obj.hStats.date_str = datestr(now,'dd-mmm-yyyy');

            t = datetime('now','Format','HH:mm:ss');
            [h,m,s] = hms(t);
            
            obj.hStats.Hour = h;
            obj.hStats.Minute = m;
            obj.hStats.Seconds = round(s);
            obj.hStats.computer_name = getenv('COMPUTERNAME');
            

            obj.hStats.Fig = figure('Name','REINFORCE: Learning progression',...
                                    'NumberTitle' ,'off');

            
            obj.hStats.Axes.J = subplot(2,2,1);
            hold on;grid on
            obj.hStats.Lines.Jnom = plot(obj.hStats.Axes.J,NaN,NaN); 
            obj.hStats.Lines.J = plot(obj.hStats.Axes.J,NaN,NaN);
            legend('Deterministic policy','Stochastic policy')           
            xlabel episode#
            ylabel J
            title 'Objective Function (J(\theta))'

            obj.hStats.Axes.Theta = subplot(2,2,2);   
            hold on;grid on
            for i = 1:length(theta0)
                obj.hStats.Lines.Theta(i) = plot(obj.hStats.Axes.Theta,0,theta0(i),'DisplayName',['\theta_' num2str(i)]);
            end
            xlabel episode#
            ylabel('\theta')
            legend show
            title 'Policy parameters (\theta)'
            drawnow
            
            obj.hStats.Axes.Theta = subplot(2,2,3);   
            hold on;grid on
            %for i = 1:length(theta0)
            obj.hStats.Lines.Grad = plot(obj.hStats.Axes.Theta,NaN,NaN);%,'DisplayName',['dJ/d\theta_' num2str(i)]);
           % end
            xlabel episode#
            ylabel('|dJ/d\theta|')
            legend show
            title 'Policy gradient norm (|dJ/d\theta|)'
            
            obj.hStats.Axes.GradPath = subplot(2,2,4);   
            hold on
            xlabel \theta_1
            ylabel \theta_2           
            title 'Policy gradient projection on 2D plane (dJ/d\theta)'   
            obj.hStats.Lines.GradArrows = quiver(obj.hStats.Axes.GradPath,...
                            NaN,NaN,... x,y
                            NaN,NaN,...u,v
                            'Color','b',...
                            'LineWidth',1,...
                            'MaxHeadSize',2,...
                            'AutoScale','off',...
                            'AutoScaleFactor',1,...
                            'AlignVertexCenters','on');
            
            warning('off','MATLAB:contour:NonFiniteData')
            [~,obj.hStats.Lines.Jcontour] = contour(obj.hStats.Axes.GradPath,NaN(2),NaN(2),NaN(2));
            
            
             obj.hStats.PauseButton = uicontrol(...
                          'Parent',  obj.hStats.Fig,...
                          'Style',   'pushbutton',...
                          'Units',   'normalized', ...
                          'Position',[0.77 0.03 0.2 0.05],...
                          'String',  'Stop training', ...
                          'Callback', @obj.StopTrainingCallback); 
            
            pause(0.1)         
            drawnow
            

            
        end    
        %% end of method
        
         %% Stop training callback:
        function StopTrainingCallback(obj,handle,event)
            
            set(handle,'Enable','off')
            obj.StopTraining = true;
        
        end
        %% end of method    
        
        %% Estimate J:
        function [] = EstimateJ(obj)

            
             sig = ones(1,size(obj.policy.theta,2)).*obj.sigma;
             theta1  = -0.5:0.5:15;
             theta2  = -0.5:0.5:15;
             
             [Th1 ,Th2] = meshgrid(theta1,theta2);
             Jnom = zeros(size(Th1,1),size(Th1,2));
             set(obj.hStats.Lines.Jcontour,'XData',Th1,'YData',Th2,'ZData',Jnom)
             
             for k1 = 1:length(theta1)
                 for k2 = 1:length(theta2)

                    obj.policy.theta = [theta1(k1) , theta2(k2)];
                    parfor j = 1:obj.n_of_workers
                        [~, r{j}, ~] = obj.run_traj(zeros(1,length(sig)));
                        Jtmp(j) = sum(r{j})/length(r{j});
                    end
                    Jnom(k1,k2) = mean(Jtmp);

                    set(obj.hStats.Lines.Jcontour,'ZData',Jnom)
                    drawnow
                    disp(['J estimation: Done iteration ' num2str((k1-1)*length(theta2) + k2) ' out of ' num2str(length(Th1(:)))])
                 end
             end
             
             
            
        end
        %% end of method
        
    end
    
    %% Utility functions:
    methods(Static)
        function  angle  = GetAngle( u,v )

            if abs( u*v'/norm(u)/norm(v) ) < 1
                angle =  wrapTo360(  acos(u*v'/norm(u)/norm(v))*180/pi );
            else
                angle = nan;
            end
            
            if isnan(angle)
                angle = 999;
            end
        end
    end
           
end


