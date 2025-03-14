% Helper function for angle adjustment
function angles = adjustEulerAngles(angles)
    for i = 1:length(angles)
        if angles(i) > pi
            angles(i) = angles(i) - 2*pi;
        elseif angles(i) < -pi
            angles(i) = angles(i) + 2*pi;
        end
    end
end