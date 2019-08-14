classdef TargetAndTrackingSystem < handle
    properties 
        t
        Ts
        target
        gimbalSystemObj
        tracker
        sceneHandle
        targetPlot
        trackerWindowPlot
        State
        episodeNumber
        ResetFlag = true;
    end
     
    methods              
        % Contructor method creates an instance of the environment
        function this = TargetAndTrackingSystem()    
            
            global  Wn zetta frameRate simulationTimeStep
            this.episodeNumber = 0; 
            this.t=0;
            this.Ts=simulationTimeStep; 
            this.target=targetToTrack; 
            this.gimbalSystemObj=gimbalSystem(Wn,zetta); 
            this.tracker=EOTrackerDescrerte(frameRate,this.target,this.gimbalSystemObj,1);
            this.tracker=output(this.tracker,this.gimbalSystemObj,this.target,this.t); 
            
            this.State = this.tracker.outputBuffer; 
            
        end
        
        function [NextObservation, Reward, IsDone] = step(this, Action)
            
            % Apply system dynamics and simulates the environment with the given action for one step.
            global simulationTimeStep Tf
            this.t=this.t+simulationTimeStep;
            this.target=this.target.propogateState(this.t); 
            this.gimbalSystemObj=this.gimbalSystemObj.propogateState(Action);
            this.tracker=output(this.tracker,this.gimbalSystemObj,this.target,this.t); 
            NextObservation=this.tracker.outputBuffer;
            Reward=-this.tracker.outputBuffer'*this.tracker.outputBuffer;%-10*this.tracker.isOutofFOV;
           % IsDone=or(this.tracker.isOutofFOV,this.t>=Tf);
           IsDone=this.t>=Tf;
        end
                
        function [InitialObservation] = reset(this)
            % Reset environment to initial state and output initial observation
            global zetta Wn frameRate
            this.episodeNumber = this.episodeNumber + 1; 
            this.t=0;
            this.target=targetToTrack; 
            this.gimbalSystemObj=gimbalSystem(Wn,zetta); 
            this.tracker=EOTrackerDescrerte(frameRate,this.target,this.gimbalSystemObj,1);
            this.tracker=output(this.tracker,this.gimbalSystemObj,this.target,this.t); 
            this.State = this.tracker.outputBuffer;
            InitialObservation=this.State; 
            IsDone=false;
            this.ResetFlag = true;

        end   
        
        function render(this)  
          
            if this.ResetFlag
                
                if isempty(findobj('Type','Figure','Name','Target Tracking Visualizer'))
                    
                    global FOV
                    
                    this.sceneHandle = gca(figure('Toolbar','none','HandleVisibility','on','NumberTitle','off','Name','Target Tracking Visualizer','MenuBar','none'));
                    this.sceneHandle.XLim=[-FOV 2*FOV]; 
                    this.sceneHandle.YLim=[-FOV 2*FOV]; 
                    axis(this.sceneHandle,'square');
                    grid(this.sceneHandle,'on');
                    hold(this.sceneHandle,'on');

                    this.targetPlot=plot(this.target.X+this.target.graphicHandleX,this.target.Y+this.target.graphicHandleY,'linewidth',2,'parent',this.sceneHandle);
                    this.trackerWindowPlot=plot(this.gimbalSystemObj.XAz(1)+this.gimbalSystemObj.graphicHandleX,this.gimbalSystemObj.XEl(1)+this.gimbalSystemObj.graphicHandleY,'linewidth',2,'parent',this.sceneHandle);
                else
                    
                    this.sceneHandle = gca(findobj('Type','Figure','Name','Target Tracking Visualizer'));
                    
                    this.targetPlot=plot(this.target.X+this.target.graphicHandleX,this.target.Y+this.target.graphicHandleY,'linewidth',2,'parent',this.sceneHandle);
                    this.trackerWindowPlot=plot(this.gimbalSystemObj.XAz(1)+this.gimbalSystemObj.graphicHandleX,this.gimbalSystemObj.XEl(1)+this.gimbalSystemObj.graphicHandleY,'linewidth',2,'parent',this.sceneHandle);     
                end 
                    
                    
                
                title(this.sceneHandle,['Episode: ',num2str(this.episodeNumber)]); 
                
                this.ResetFlag = false;
            end
            
            % Visualization 
            set(this.targetPlot,'XData', this.target.X+this.target.graphicHandleX,...
                'YData',this.target.Y+this.target.graphicHandleY);
            set(this.trackerWindowPlot,'XData',this.gimbalSystemObj.XAz(1)+this.gimbalSystemObj.graphicHandleX,...
                'YData', this.gimbalSystemObj.XEl(1)+this.gimbalSystemObj.graphicHandleY);
       
            drawnow
        end
                
    end
end


