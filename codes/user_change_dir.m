function [m_goal_out, n_distance] = user_change_dir(last_min, curPos)
% This function is used to change direction of the given angle values.
% Function finds the min. distance between bug and obstacle from right side
% of the bug. If last distance value is greater or lesser this one it returns 
% a new angle value because it means bug is move away from obstacle or get
% closer to the obstacle. It returns new distance 
%% -----------------

global infinity;
global last_angle;
global sensor_range;

global LogFile;

[n_distance,tng] = user_dist2Obs(curPos);

%yeni aci bulunamadý!!
if((n_distance == infinity) && (tng == pi))
    tng = last_angle + pi/2;
    n_distance = last_min;
end

if((n_distance <= 0.5*sensor_range) )
    tng = tng - pi/2;
    fprintf(LogFile, '\nExtremely Get Closed!!\n');
end


m_goal_out = tng - pi/2;
last_angle = tng;
