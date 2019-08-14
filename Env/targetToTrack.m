classdef targetToTrack < handle
    properties
        X
        Y
        gamma
        maneuverDirection
        switchInterval
        lastSwitchTime
        maneuverDirectionSpace=[-1 0 1]; 
        graphicHandle 
        graphicHandleX=0.003*sin(0:.1:2*pi);
        graphicHandleY=0.003*cos(0:.1:2*pi);
    end
    methods
        function this=targetToTrack % Constructor
            global   FOV switchTimeConstant 
            L=rand*4*FOV;
            this.X=L*(L<=FOV)+FOV*(L>FOV && L<=2*FOV)+(L-2*FOV)*(L>2*FOV && L<=3*FOV)+0*(L>3*FOV);
            this.Y=0*(L<=FOV)+(L-FOV)*(L>FOV && L<=2*FOV)+FOV*(L>2*FOV && L<=3*FOV)+(L-3*FOV)*(L>3*FOV);
            this.gamma=atan2(FOV/2- this.Y,FOV/2-this.X);
            this.maneuverDirection=0;
            this.switchInterval=rand*switchTimeConstant+switchTimeConstant;
            this. lastSwitchTime=0;
            
        end
        function this=propogateState(this,t)
            if this.isSwitchTime(t) %(target,t,lastSwitchTime)
                this=this.switchManeuver; 
                this.lastSwitchTime=t;
            end
            xNew = this.RungeKutta;
            this.X=xNew(1); 
            this.Y=xNew(2); 
            this.gamma=xNew(3); 
        end
        function xNew = RungeKutta(this)
            global simulationTimeStep
            xOld=[this.X this.Y this.gamma]'; 
            k1=simulationTimeStep*this.x_dot(xOld); 
            k2=simulationTimeStep*this.x_dot(xOld+k1/2); 
            k3=simulationTimeStep*this.x_dot(xOld+k2/2); 
            k4=simulationTimeStep*this.x_dot(xOld+k3/2);
            xNew=xOld+(k1+2*k2+2*k3+k4)/6;
        end
        function r=x_dot(this,x)
            global  targetSpeed  H maxTurnRate
            r(1)=targetSpeed*cos(x(3))/H;
            r(2)=targetSpeed*sin(x(3))/H;
            r(3)=this.maneuverDirection*maxTurnRate; 
            r=r';
        end
        function booleanIsOutOfFOV=isOutofFOV(this)
            global FOV
            booleanIsOutOfFOV=this.X<0 || this.X>FOV || this.Y<0 || this.Y>FOV; 
        end
        function booleanIsSwitchTime=isSwitchTime(this,t)
            booleanIsSwitchTime=(t-this.lastSwitchTime>this.switchInterval); 
        end
        function this=switchManeuver(this)
            global switchTimeConstant
            q=this.maneuverDirectionSpace(~ismember(this.maneuverDirectionSpace,this.maneuverDirection)); 
            if rand<0.5
                this.maneuverDirection=q(1); 
            else 
                this.maneuverDirection=q(2);
            end
            this.switchInterval=rand*switchTimeConstant+switchTimeConstant; 
            if this.maneuverDirection~=0
                            this.switchInterval=this.switchInterval/3; 
            end
        end
    end
end