classdef TimerHandle < handle    
    properties
        replay_timer
        count = 0
    end
    methods
        function register_timer(obj)
            obj.replay_timer = timer('TimerFcn', {@obj.on_timer}, 'ExecutionMode', 'fixedSpacing', ...
                'Period', 1, 'BusyMode', 'drop', 'TasksToExecute', inf);
        end
        function on_timer(obj, varargin)
            obj.count = obj.count + 1;
            fprintf('[%d] on_timer()\n', obj.count);
        end
        function delete(obj)
            delete(obj.replay_timer);
            obj.delete@handle();
        end
    end
end
