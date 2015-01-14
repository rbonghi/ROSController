classdef ExplorerComm < handle
    %EXPLORER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    properties (Access = protected)
        Node = [];                  % ROS node.
        
        VelPub = [];                % ROS publisher for velocity commands.
        %PosePub = [];               % ROS publisher for pose commands.
        OdomSub = [];               % ROS subscriber for odometry data.
        BaseVelocityMsg = [];       % ROS message for carrying velocity information.
        %PoseMsg = [];               % ROS message for carrying pose information.
        
        dimData = 50;
        poseData = [];
        velData = [];
        
        graph = [];
        figConsole;
        centerMap = [0 0];
        spaceDistMap = 0.5;
        %filter
        lastVel = [];
        dimFilt = 5;
        % hystory record
        videoObj;
        record = 0;
        qHystory = [];
        vHystory = [];
        % Functions
        funMap = {};                 % Function to add property in position map
        funFilter;              % Function to filtering data to odometry
        funOdometryCallback;    % Function callback from odometry
    end
    
    methods
        function h = ExplorerComm(rosMasterIp, localhostIp)
            h.Node = rosmatlab.node('matlab_controller', rosMasterIp, 11311, 'rosIP', localhostIp);
            
            % Add a publisher.
            h.VelPub = h.Node.addPublisher('/robot/command/velocity', 'geometry_msgs/Twist');
            % Add a publisher.
            %h.PosePub = h.Node.addPublisher('/robot/pose', 'serial_bridge/Pose');
            
            % Add subscribers.
            h.OdomSub = h.Node.addSubscriber('/robot/odometry', 'nav_msgs/Odometry', 25);
            h.OdomSub.setOnNewMessageListeners({@h.odometryCallback});
            
            % Create a message.
            h.BaseVelocityMsg = rosmatlab.message('geometry_msgs/Twist', h.Node);
            %h.PoseMsg = rosmatlab.message('serial_bridge/Pose', h.Node);

            % Filter
            h.lastVel = zeros(h.dimFilt,2);
            
            % Set figure
            h.poseData = zeros(h.dimData,3);
            h.velData = zeros(h.dimData,2);
            h.figConsole = figure(1);
            set(1,'Name','Console Robot','Position', [100, 100, 1024, 768]);%,'DefaultFigureCloseRequestFcn',@h.console_close);
        end
        
        function setVelocity(h, v)
            % SETVELOCITY(v) - Set the linear velocity target
            % v     Linear velocity.
            angVelocity = h.BaseVelocityMsg.getAngular();
            linVelocity = h.BaseVelocityMsg.getLinear();
            linVelocity.setX(v(1));
            angVelocity.setZ(v(2));
            linVelocity.setY(0.0); linVelocity.setZ(0.0);
            angVelocity.setX(0.0); angVelocity.setY(0.0);
            h.BaseVelocityMsg.setLinear(linVelocity);
            h.BaseVelocityMsg.setAngular(angVelocity);
            h.VelPub.publish(h.BaseVelocityMsg);
        end
        
%         function setPose(h, q)
%             % SETVELOCITY(v) - Set the linear velocity target
%             % v     Linear velocity.
%             h.PoseMsg.setX(q(1));
%             h.PoseMsg.setY(q(2));
%             h.PoseMsg.setTheta(q(3));
%             h.PoseMsg.setSpace(0.0);
%             h.VelPub.publish(h.PoseMsg);
%         end
        
        function startRecord(h, name)
            if h.record == 0
                h.videoObj = VideoWriter([name '.avi']);
                h.videoObj.FrameRate = 10;
                open(h.videoObj);
                h.qHystory = [];
                h.vHystory = [];
                h.record = 1;
            end
        end
        
        function [q, v] = stopRecord(h)
            if h.record == 1
                pause(2);
                close(h.videoObj);
                h.record = 0;
            end
            q = h.qHystory;
            v = h.vHystory;
        end
        
        function setAddToMap(h,map)
            if isempty(map)
                h.funMap = {};
            else
                h.funMap{size(h.funMap,2)+1} = map;
            end
        end
        
        function setFilter(h,filter)
            h.funFilter = filter;
        end
        
        function setOdometryCallback(h,odometry)
            h.funOdometryCallback = odometry;
        end
        
        function setCenter(h, center)
            h.centerMap = center;
        end
        
        function setSpaceDistance(h, dist)
           h.spaceDistMap = dist;
        end
        
        function delete(h)
            % Destructor.
            h.stopRecord();
            close(h.figConsole);
            delete(h.Node);
        end
    end
    
    methods (Access = protected)        
        function odometryCallback(h, message)
            % ODOMETRYCALLBACK - Execute tasks when new odometry data
            % (nav_msgs/Odometry) is received.
            
            pose = message.getPose();
            pos = pose.getPose().getPosition();
            vel = message.getTwist().getTwist();
            orientation = pose.getPose().getOrientation();
            den = orientation.getW()^2 - orientation.getZ()^2;
            num = 2 * (orientation.getW() * orientation.getZ());
            % Position
            x = pos.getX();
            y = pos.getY();
            th = atan2(num, den);
            % Velocity
            lin = vel.getLinear().getX();
            ang = vel.getAngular().getZ();
            % Update array
            if ~isempty(h.funFilter)
                [h.poseData, h.velData] = h.funFilter(h.poseData,h.velData, [x, y, th], [lin, ang]);
            else
                h.poseData = [h.poseData(2:h.dimData,:); [x, y, th]];
                h.lastVel = [h.lastVel(2:h.dimFilt,:); [lin, ang]];
                vm = mean(h.lastVel);
                h.velData = [h.velData(2:h.dimData,:); vm];
            end
            % Odometry Callback
            if ~isempty(h.funOdometryCallback)
                h.funOdometryCallback(h.poseData(end,:),h.velData(end,:));
            end
            % Plot console
            h.figureConsole(h.poseData, h.velData);
            
            if h.record == 1
                % Save frame
                F_frame = getframe(h.figConsole);
                h.qHystory = [h.qHystory; [x, y, th]];
                h.vHystory = [h.vHystory; [lin, ang]];
                writeVideo(h.videoObj,F_frame);
            end
        end
        
        function figureConsole(h, q, v)
            h.figConsole;
            set(0,'CurrentFigure',h.figConsole);
            % Position control
            subplot(3,3,[1,2,4,5]);
            h.plotRobot();
            % Plot Position
            ts_pos = timeseries(q(:,1:2), 1:h.dimData, 'name', 'Position');
            subplot(3,3,3);
            plot(ts_pos);
            grid on;
            % Plot angle
            subplot(3,3,6);
            ts_angle = timeseries(q(:,3), 1:h.dimData, 'name', 'Angle');
            plot(ts_angle);
            set(gca,'ytick',-pi:pi/4:pi);
            set(gca,'yticklabel',{'-pi','-3/4 pi','-pi/2','-pi/4','0','pi/4','pi/2','3/4 pi','pi'});
            grid on;
            % Plot velocity
            subplot(3,3,[7,8,9]);
            ts_velocity = timeseries(v, 1:h.dimData, 'name', 'Velocity');
            plot(ts_velocity);
            grid on;
        end
        
        function plotRobot(h)
            plot(h.poseData(:,1),h.poseData(:,2),':','LineWidth',2);
            hold on;
            plot(h.poseData(end,1),h.poseData(end,2),'.','Color','blue');
            [arrowX, arrowY] = h.triangle(h.poseData(end,1),...
                h.poseData(end,2),...
                h.poseData(end,3), 0.2, 0.2);
            plot(arrowX, arrowY,'Color','blue');
            if ~isempty(h.funMap)
                for i=1:size(h.funMap,1)
                    h.funMap{i}(h.poseData,h.velData);
                end
            end
            h.boxMap();
            axis(h.graph);
            axis square;
            grid on;
            hold off;
            xlabel('m');
            ylabel('m');
            title('Robot position');
        end
        
        function boxMap(h)
            r = abs(h.centerMap-h.poseData(end,1:2))/2;
            box_dist = max(r)+h.spaceDistMap;
            center_map = (h.centerMap+h.poseData(end,1:2))/2;
            h.graph = [center_map(1)-box_dist center_map(1)+box_dist center_map(2)-box_dist center_map(2)+box_dist];
            plot(h.centerMap(1),h.centerMap(2),'*','Color','red');
            plot(0,0,'.','Color','red');
            plot([center_map(1)-box_dist center_map(1)+box_dist],[0 0],'Color','red');
            plot([0 0],[center_map(2)-box_dist center_map(2)+box_dist],'Color','red');
        end
    end
    
    methods (Access = protected, Static)
        function [arrowX, arrowY] = triangle(x,y,th, lenght, width)
            
            vectorX = [-lenght/2 lenght/2  -lenght/2 -lenght/2];
            vectorY = [-width/2       0.0   width/2  -width/2 ];
            
            sinTh = sin(th);
            cosTh = cos(th);
            
            arrowX = [0 0 0 0];
            arrowY = [0 0 0 0];
            
            for i=1:4
                arrowX(i) = vectorX(i)*cosTh - vectorY(i)*sinTh;
                arrowY(i) = vectorX(i)*sinTh + vectorY(i)*cosTh;
            end
            
            arrowX = arrowX + x;
            arrowY = arrowY + y;
        end
    end
    
end

