function collisionFlag = checkCollisionState(robot, config, obstacle)
    % Determine the data format of the robot
    dataFormat = robot.DataFormat;
    
    % Initialize the configuration structure based on the data format
    if strcmp(dataFormat, 'struct')
        configStruct = homeConfiguration(robot);
        
        % Check if the length of the config matches the number of joints in the robot
        if length(config) ~= length(configStruct)
            error('Configuration length does not match the number of joints in the robot.');
        end

        % Ensure that configStruct is a struct array
        if ~isstruct(configStruct)
            error('homeConfiguration did not return a structure array. Check the output of homeConfiguration.');
        end

        % Set the joint positions based on the current configuration
        for k = 1:length(config)
            if isfield(configStruct(k), 'JointPosition')
                configStruct(k).JointPosition = config(k);
            else
                error('Field JointPosition not found in the configuration structure. Verify the structure of homeConfiguration output.');
            end
        end
    elseif strcmp(dataFormat, 'row') || strcmp(dataFormat, 'column')
        configStruct = config;
    else
        error('Unsupported DataFormat: %s', dataFormat);
    end

    % Check for collisions with specified SkippedSelfCollisions argument
    collisionFlag = any(checkCollision(robot, configStruct, {obstacle}, 'SkippedSelfCollisions', 'parent'));
end
