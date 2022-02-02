function color = localization_color(name)
name = localization_type(name);

color = [0 0 0];

if name == "IMU"
    color = sscanf("3d9db2", '%2x%2x%2x', [1 3])/255;
end
if name == "KLT"
    color = sscanf("dc8181", '%2x%2x%2x', [1 3])/255;
end
if name == "PX4"
    color = sscanf("43bb4b", '%2x%2x%2x', [1 3])/255;
end
if name == "MOD"
    color = sscanf("e9a06c", '%2x%2x%2x', [1 3])/255;
end
if name == "SLAM"
    color = sscanf("ebd96a", '%2x%2x%2x', [1 3])/255;
end

end

