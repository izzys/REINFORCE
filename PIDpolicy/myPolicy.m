classdef myPolicy < handle
    properties
        azController
        elController
    end
    
    methods
        function this = myPolicy(azController,elController)
            this.azController = azController; 
            this.elController = elController; 
        end
        function [action]=getAction(this,ob)
            action(1)=rateCommand(this.azController,ob(1));
            action(2)=rateCommand(this.elController,ob(2));
        end
        function reset(this)
            this.azController.state=zeros(size(this.azController.state)); 
            this.elController.state=zeros(size(this.elController.state)); 
        end

    end
end

