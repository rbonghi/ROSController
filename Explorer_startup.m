%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%       Project: ROBOT ROS CONTROLLER                    %%%
%%%       Author: BONGHI RAFFAELLO                         %%%
%%%       File configuration                               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~exist(fullfile(matlabroot, 'toolbox', 'psp', 'rosmatlab'), 'dir')
    % Abort if ROS I/O Package is not installed.
    errordlg(['You must install ROS I/O Package before running this controller. ',...
        'Please visit www.mathworks.com/ros to download ROS I/O Package.'], ...
        'ROS I/O Package Not Found');
else
    %global Explorer;
    Explorer = ExplorerComm('explorer.local','Enterprise.local');
    %Explorer.setAddToMap(@addToMap);
end



