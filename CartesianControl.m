classdef CartesianControl < handle
    %CARTESIACONTROL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        k = [0.5 1];
        precision = 10^-3;
        Vm = [0.1 pi/2];
        forward = 1;
        dimFilter = 5;
    end
    
    properties (Access = protected)
        Explorer;
        pos_rif = [0 0];
        enable = 0;
        
        %Image
        figConsole;
        dimData = 50;
        Error = [];
        Vel = [];
        VelC = [];
        qHistory = [];
        lastPositon;
        %filter
        velFilter = [];
        
        %record
        record = 0;
        
        %Timer
        replay_timer;
        count = 0;
        frame_rate = 10.0;
    end
    
    methods
        function h = CartesianControl(Explorer)
            h.Explorer = Explorer;
            h.Explorer.setOdometryCallback(@h.odoCallBack);
            h.Explorer.setAddToMap(@h.addToMap);
            
            % Start timer plot
            period = 1/h.frame_rate;
            h.replay_timer = timer('TimerFcn', {@h.on_timer}, 'ExecutionMode', 'fixedSpacing', ...
                'Period', period, 'BusyMode', 'drop', 'TasksToExecute', inf);
        end
        
        function on_timer(h, varargin)
            %h.count = h.count + 1;
            %fprintf('[%d] on_timer()\n', h.count);
            h.control(h.lastPositon);
        end
        
        function odoCallBack(h, q, ~)
            h.lastPositon = q;
            % Plot console control
            plotConsole(h)
        end
        
        function control(h, q)
            if h.enable == 1
                % Save position
                h.qHistory = [h.qHistory; q];
                % Control
                v = zeros(1,2);
                e = h.pos_rif-q(1:2);
                h.Error = [h.Error; e];
                th = q(3);
                v(1) = h.k(1)*(e(1)*cos(th) + e(2)*sin(th));
                if h.forward == 1
                    v(1) = abs(v(1));
                else
                    if e(1) <= 0
                        e = -e;
                    end
                end
                v(2) = h.k(2)*(atan2(e(2),e(1))-th);
                h.Vel = [h.Vel; v];
                % Correction control
                vc = zeros(1,2);
                vn = abs(v(1))/h.Vm(1);
                wn = abs(v(2))/h.Vm(2);
                s = max([vn wn 1]);
                if s == 1
                    vc = v;
                else
                    if s == vn
                        vc(1) = sign(v(1))*h.Vm(1);
                        vc(2) = v(2)/s;
                    else
                        vc(1) = v(1)/s;
                        vc(2) = sign(v(2))*h.Vm(2);
                    end
                end
                % Filter command
                h.velFilter = [h.velFilter(2:h.dimFilter,:); vc];
                mvc = mean(h.velFilter);
                vc(1) = mvc(1);
                % Save data
                h.VelC = [h.VelC; vc];
                % Send control
                v = vc;
                h.Explorer.setVelocity(v);
                if abs(e) <= ones(1,2)*h.precision
                    h.stop(q);
                end
            end
        end
        
        function goal(h, position, record)
            if h.enable == 1
                h.stop();
            end
            h.pos_rif = position;
            %h.Explorer.setCenter(h.pos_rif);
            h.Explorer.setCenter(h.lastPositon(1:2));
            Xtext = sprintf('START! go to: %3.f %3.f', h.pos_rif(1),h.pos_rif(2));
            disp(Xtext);

            
            h.Error = [];
            h.Vel = [];
            h.VelC = [];
            h.qHistory = [];
            h.velFilter = zeros(h.dimFilter, 2);
            
            h.figConsole = figure(2);
            set(2,'Name','Cartesian controller','Position', [100, 100, 1024, 768]);
            
            % record
            if exist('record','var')
                h.record = 1;
                h.Explorer.startRecord(record);
            end
            start(h.replay_timer);
            % Start controller
            h.enable = 1;
        end
        
        function stop(h, q)
            h.enable = 0;
            h.Explorer.setVelocity([0 0]);
            stop(h.replay_timer);
            if exist('q','var')
                Xtext = sprintf('Stop! Arrive at: [x: %3.f, y: %3.f]', q(1), q(2));
            else
                Xtext = 'Force stop';
            end
            disp(Xtext);
            if h.record == 1
                h.Explorer.stopRecord;
            end
        end
        
        function delete(h)
            % Destructor.
            delete(h.replay_timer);
            close(h.figConsole);
            h.Explorer.setAddToMap([]);
            h.Explorer.setOdometryCallback([]);
        end
    end
    
    methods (Access = protected)
        function addToMap(h, q, v)
            if ~isempty(h.qHistory)
                plot(h.qHistory(:,1),h.qHistory(:,2),'r-.','LineWidth',2);
                plot(h.pos_rif(1),h.pos_rif(2),'ro');
            end
        end
        
        function plotConsole(h)
            % Plot data
            h.figConsole;
            set(0,'CurrentFigure',h.figConsole);
            subplot(2,3,[1,2,4,5]);
            ts_error = timeseries(h.Error, 1:size(h.Error,1), 'name', 'Error');
            plot(ts_error);
            grid on;
            % Velocity
            subplot(2,3,3);
            ts_velocity = timeseries(h.Vel, 1:size(h.Vel,1), 'name', 'Velocity');
            plot(ts_velocity);
            grid on;
            % Velocity constrain
            subplot(2,3,6);
            ts_velocity_c = timeseries(h.VelC, 1:size(h.VelC,1), 'name', 'Velocity constrain');
            plot(ts_velocity_c);
            grid on;
        end
    end
end

