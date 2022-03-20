function [shortestPoints, shortestDistance]  = user_shortestPoints(curPos,lastShortestPoints)
%this func. is created to find best point when bug travel around the
%obstacle. It also return the shortestDistance between bug and qgoal.

global qgoal;

% a : distance between current Pos and qgoal
% b : distance between latest Pos and qgoal distance
a = norm([curPos(1)-qgoal(1), curPos(2)-qgoal(2)]);
b = norm([lastShortestPoints(1)-qgoal(1), lastShortestPoints(2)-qgoal(2)]);

if( a < b )
    shortestPoints(1) = curPos(1);
    shortestPoints(2) = curPos(2);
    shortestDistance = a;
else
    shortestPoints = lastShortestPoints;
    shortestDistance = b;
end



end