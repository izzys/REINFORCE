classdef LinearGainPolicy < handle

    properties
        ObDim;
        BufferSize = 1;
        ObBuffer   = [];
        theta      = [];
    end
    
    methods
        function obj = LinearGainPolicy(varargin)
            
            % count arguments
            nArgs = length(varargin);
            if round(nArgs/2)~=nArgs/2
               error('LinearGainPolicy needs propertyName/propertyValue pairs')
            end  
            
            if nargin<2
               error('LinearGainPolicy needs at least 1 input (Dim)') 
            end
            
            for pair = reshape(varargin,2,[]) % pair is {propName;propValue}
               inpName = lower(pair{1}); % make case insensitive

               switch inpName 
                    case 'obdim'
                        obj.ObDim = pair{2};
                    case 'buffersize'
                         obj.BufferSize = pair{2}; 
                    otherwise
                         error('%s is not a recognized parameter name',inpName)
               end                  
            end
            
        end
        
        function a = getAction(obj,ob)
                       
            K = obj.get_K(obj.theta);
            
            obj.ObBuffer = [ob(:)  , obj.ObBuffer(:,1:end-1)] ;
            a = K*obj.ObBuffer';

        end
      
        function [a,z] = sampleAction(obj,ob,sigma)
                       
            z = randn(1,length(sigma)).*sigma;
            theta_z = obj.theta + z;
            K = obj.get_K(theta_z);
            
            obj.ObBuffer = [ob(:)  , obj.ObBuffer(:,1:end-1)] ;
            
            for i = 1:obj.ObDim
                a(i) = K(i,:)*obj.ObBuffer(i,:)';
            end

        end
        
        function K = get_K(obj,theta)
            
               K = reshape( theta(:) , obj.BufferSize,obj.ObDim)';

        end
        

        
        function reset(obj)
            obj.ObBuffer = zeros(obj.ObDim,obj.BufferSize);
        end
    end
end

