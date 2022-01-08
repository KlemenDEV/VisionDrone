function type = localization_type(name)
    [~, name, ~] = fileparts(name);
    parts = regexp(name, '_', 'split', 'once');
    type = string(parts(2));
    
    if type == "dead_reckoning"
        type = "IMU";
    end
    if type == "flow_opencv"
        type = "KLT";
    end
    if type == "flow_px4"
        type = "PX4";
    end
    if type == "motion_simulation"
        type = "MOD";
    end
    if type == "orbslam3"
        type = "SLAM";
    end
end

