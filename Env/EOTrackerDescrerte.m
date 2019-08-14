classdef EOTrackerDescrerte < handle
    properties
        dX
        dY
        dXprev
        dYprev
        outputBuffer
        frameInterval
        lastUpdate
    end
    methods
        function this = EOTrackerDescrerte(frameRate,target,system,bufferLengh)
            this.outputBuffer=999*ones(2,bufferLengh);
            this.outputBuffer(:,1)=[target.X - system.XAz(1);  target.Y - system.XEl(1)];
            this.frameInterval=1/frameRate;
            this.lastUpdate=0;
        end
        function this = output(this,system,target,t)
            if length(this.outputBuffer(1,:))>1
                this.outputBuffer(:,2:end)=this.outputBuffer(:,1:end-1);
            end
            this.outputBuffer(:,1)=[target.X - system.XAz(1);  target.Y - system.XEl(1)];
            this.lastUpdate=t;
        end
        function boolenIsOutOfFOV=isOutofFOV(this)
            global FOV
            boolenIsOutOfFOV=abs(this.outputBuffer(1,1))>1.1*FOV/2 || abs(this.outputBuffer(2,1))>1.1*FOV/2;
        end    
        function boolenIsNewData=isNewData(this,t)
            boolenIsNewData=t-this.lastUpdate>=this.frameInterval;
        end
        function boolenIsValid=isValid(this)
            boolenIsValid=this.outputBuffer(1,end)~=999;
        end        
    end
end

