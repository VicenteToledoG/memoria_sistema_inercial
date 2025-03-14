function adjustedAngle = adjustAngle(angleDifference)
    if angleDifference > pi
        adjustedAngle = angleDifference - 2 * pi;
    elseif angleDifference < -pi
        adjustedAngle = angleDifference + 2 * pi;
    else
        adjustedAngle = angleDifference;
    end