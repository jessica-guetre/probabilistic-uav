function motion = updateFlight(motion, maxVelocity, xlimits, ylimits)
    % Move UAV along its velocity vector.
    if any(motion(1) < (xlimits(1)  + 10)) || any(motion(1) > (xlimits(2) - 10)) || any(motion(2) < (ylimits(1) + 10)) || any(motion(2) > (ylimits(2) + 10))
        motion(4) = rand * maxVelocity * randsample([-1,1],1); % Random new x velocity
        disp('motion(4): ');
        disp(motion(4));
        motion(5) = sqrt(maxVelocity^(2) - motion(4)^(2)) * randsample([-1,1],1); % Random new y velocity
        disp('motion(5): ');
        disp(motion(5));
    end
end
