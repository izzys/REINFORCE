classdef controllerDescretePID < handle 
    properties
        type
        Kp
        Ki
        Kd
        A
        B
        C
        D
        state
        rateCommandLimit
    end
    methods
        function this = controllerDescretePID(plant,type,wc,rateCommandLimit,frameRate)
            global simulationTimeStep
            Cntrl=pidtune(plant,type,wc);
            this.Kp=Cntrl.Kp; this.Ki=Cntrl.Ki; this.Kd=Cntrl.Kd;
            Css=c2d(ss(tf(pid(Cntrl.Kp,Cntrl.Ki,Cntrl.Kd,simulationTimeStep))),1/frameRate);
            this.A=Css.A; this.B=Css.B; this.C=Css.C; this.D=Css.D;
            this.state=zeros(size(Css.B));
            this.rateCommandLimit=rateCommandLimit;
            this.type=type;
        end 
        function [output] = rateCommand(this,err)
            this.state=this.A*this.state+this.B*err;
            output=this.C*this.state+this.D*err;
            output = saturate(output,this.rateCommandLimit);
        end
    end
end

