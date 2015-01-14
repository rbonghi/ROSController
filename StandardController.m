classdef StandardController < handle
    %STANDARDCONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        lastPose = [];
        lastVel = [];
        %Timer
        replay_timer;
        count = 0;
        frame_rate = 10.0;
    end
    
    methods
        function this = StandardController()
            % Start timer for controller
            this.replay_timer = timer('TimerFcn', {@this.on_timer}, 'ExecutionMode', 'fixedSpacing', ...
                'Period', 1/this.frame_rate, 'BusyMode', 'drop', 'TasksToExecute', inf);
            disp('Create timer');
        end
        
        function goal(this)
            start(this.replay_timer);
            disp('start');
        end
        
        function stop(this)
            stop(this.replay_timer);
        end
        
        function on_timer(this, varargin)
            this.count = this.count + 1;
            fprintf('[%d] on_timer()\n', this.count);
            %h.control(h.lastPositon);
        end
        
        function odoCallBack(this, q, v)
            this.lastPose = q;
            this.lastVel = v;
            % Plot console control
            %plotConsole(h)
        end
        
        function delete(this)
            % Destructor.
            delete(this.replay_timer);
        end
    end
    
end

