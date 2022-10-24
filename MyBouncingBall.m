classdef MyBouncingBall < HybridSystem
    % Define bouncing ball system as a subclass of HybridSystem.
    properties
        % Define variable class properties.
        
    end
    properties(SetAccess = immutable)
        % Define constant (i.e., "immutable") class properties.
        gamma = 0.8;
        g = 9.81;
    end
    methods
        % Define functions, including definitions of f, g, C, and D.      
        function xdot = flowMap(this, x, t, j)
%         x = [ height, velocity ]
        
        xdot = [x(2); - this.g]; 
        end
        function xplus = jumpMap(this, x, t, j)
            
            xplus = [ x(1); - this.gamma * x(2) ];
        end
        function inC = flowSetIndicator(this, x, t, j)
            inC = (x(1) > 0 || x(2) >= 0);
        end
        function inD = jumpSetIndicator(this, x, t, j)
            inD = (x(1) <= 0 && x(2) < 0);
        end
    end
end