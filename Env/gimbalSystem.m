classdef gimbalSystem < handle
    properties
        XAz % Az Az_dot Az_dot2 
        XEl %El El_dot El_dot2
        A
        B
        dynamicSystem
        graphicHandle 
        graphicHandleX
        graphicHandleY
    end
    methods
        function this=gimbalSystem(Wn,zetta) % Constructor
            global FOV
            this.XAz=[FOV/2 0 0]';
            this.XEl=[FOV/2 0 0]';
            this.A=[   0               1            0
                          0               0            1
                          0            -Wn^2  -2*zetta*Wn];
            this.B=[0  0 Wn^2]';
            this.dynamicSystem=ss(this.A,this.B,[1 0 0],0);
            this.graphicHandleX=[-FOV/2 FOV/2 FOV/2 -FOV/2 -FOV/2]; 
            this.graphicHandleY=[-FOV/2 -FOV/2 FOV/2 FOV/2 -FOV/2];
        end
        function this=propogateState(this,U)
            this.XAz = this.RungeKutta(this.XAz,U(1));
            this.XEl = this.RungeKutta(this.XEl,U(2)); 
        end
        function xNew = RungeKutta(this,xOld,u)
            global simulationTimeStep rateCommandLimit
            k1=simulationTimeStep*this.x_dot(xOld,u); k1(2) = saturate(k1(2),rateCommandLimit);
            k2=simulationTimeStep*this.x_dot(xOld+k1/2,u); k2(2) = saturate(k2(2),rateCommandLimit);
            k3=simulationTimeStep*this.x_dot(xOld+k2/2,u); k3(2) = saturate(k3(2),rateCommandLimit);
            k4=simulationTimeStep*this.x_dot(xOld+k3/2,u); k4(2) = saturate(k4(2),rateCommandLimit);
            xNew=xOld+(k1+2*k2+2*k3+k4)/6; xNew(2) = saturate(xNew(2),rateCommandLimit);
        end
        function r=x_dot(this,x,u)
                r=this.A*x+this.B*u;
        end
    end
end