function [dist,tng] = user_dist2Obs(curPos)
% this func gives the min distance between bug and obstacle
% and the tangent value of this direction.

    allAng = linspace(-pi, +pi, 360);
    
    dist = read_sensor(0, [curPos(1) curPos(2)]);
    tng = pi;
    for ang = 1:1:360
        if(dist>=read_sensor(allAng(ang), [curPos(1) curPos(2)]))
            dist = read_sensor(allAng(ang), [curPos(1) curPos(2)]);
            tng = allAng(ang);
        end
    end

end