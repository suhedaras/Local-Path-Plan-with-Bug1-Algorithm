% Write your Bug Motion Planning algorithm inside this function.
% This function is expected to return a collision free path
% starting from qstart and ending in qgoal, both supplied as row
% vectors. The returned values should be row vectors with the x and
% y coordinates of the robot along this path, as it would have
% followed with bug1 and bug2 algorithms in effect, respectively.
%
% Try a number of different scenarios and report the videos of results
% while the robot is following the planned motion path.

function [x,y] = bug_planner( qstart, qgoal )

global sensor_range infinity;

global LogFile;

%% 1. A bug which blindly goes towards the goal passing over obstacles:

% stepSize = 0.5* sensor_range;
% dist2goal = norm(qgoal-qstart);
% nSamples = dist2goal / stepSize;
% t = linspace(0,1,nSamples);
% x = qstart(1) + t*(qgoal(1) - qstart(1));
% y = qstart(2) + t*(qgoal(2) - qstart(2));

%% 2. A bug which goes towards the goal and stops when it hits an obstacle:
%
% stepSize = 0.5* sensor_range;
% curPos = qstart;
% x(1) = curPos(1);
% y(1) = curPos(2);
%
% m_goal = atan2( (qgoal(2)-qstart(2)) , (qgoal(1)-qstart(1)) );    % Calculate tangent angle from start to goal
%
% i = 1;
% while read_sensor( m_goal , [x(i) y(i)] ) > sensor_range
%     read_sensor( m_goal , [x(i) y(i)] )
%     i=i+1;
%     curPos(1) = curPos(1) + stepSize*cos(m_goal);
%     curPos(2) = curPos(2) + stepSize*sin(m_goal);
%     x(i) = curPos(1);
%     y(i) = curPos(2);
% end


%% 3. User defined bug1 algorithm: 

%we have 3 mode

stepSize = 0.2* sensor_range;
curPos = qstart;
x(1) = curPos(1);
y(1) = curPos(2);
i = 1;

m_goal = atan2( (qgoal(2)-qstart(2)) , (qgoal(1)-qstart(1)) );    % Calculate tangent angle from start to goal

dist_mgoal_dir2Obs = read_sensor( m_goal , [x(i) y(i)] );
[shortestPoints, shortestDistance]  = user_shortestPoints(curPos,curPos);

mode = user_ModeTypes.GoDirectly;
stop = false;
finish_type = 0;

while(~ stop)
    
    %disp(mode);
    switch(mode)
        case user_ModeTypes.GoDirectly
            fprintf(LogFile,  'GoDirectly Mode!\n');
            %stop = true;
            %mode = user_ModeTypes.TurntoShortestPoint;
            
            while dist_mgoal_dir2Obs > sensor_range              
                
                i=i+1;
                
                curPos(1) = curPos(1) + stepSize*cos(m_goal);
                curPos(2) = curPos(2) + stepSize*sin(m_goal);
                x(i) = curPos(1);
                y(i) = curPos(2);
                [shortestPoints, shortestDistance]  = user_shortestPoints(curPos, shortestPoints);
                dist_mgoal_dir2Obs = read_sensor(m_goal , [x(i) y(i)]);
                fprintf(LogFile,  'i : %d \t x : %0.4f \t y : %0.4f \t dist2Obs : %0.4f\n', i, x(i), y(i), dist_mgoal_dir2Obs );
                
                if( shortestDistance <= stepSize)
                    mode = user_ModeTypes.Finish;
                    finish_type = 2;
                    break;
                end
                
            end
            
            if(dist_mgoal_dir2Obs <= sensor_range)
                mode = user_ModeTypes.TurnAround;
                %finish_type = 1;
            elseif(finish_type == 0)
                mode = 1; 
            else
                %fprintf(LogFile,  'dist_mgoal_dir2Obs is Needed to look!\n');
                fprintf(LogFile,  'Wait for Result!\n');
            end
            
            
        case user_ModeTypes.TurntoShortestPoint
            fprintf(LogFile, 'TurntoShortestPoint Mode!\n');
            %stop = true;
            %mode = user_ModeTypes.TurnAround;
            L = linspace(0, 2*pi, 360);
            shPointX = shortestPoints(1)+cos(L)*stepSize;
            shPointY = shortestPoints(2)+sin(L)*stepSize;
            
            while(true)
                
                [m_goal_n, lastmin] = user_change_dir(lastmin,curPos);
                i=i+1;                
                curPos(1) = curPos(1) + stepSize*cos(m_goal_n);
                curPos(2) = curPos(2) + stepSize*sin(m_goal_n);
                x(i) = curPos(1);
                y(i) = curPos(2);
                
                dist_mgoal_dir2Obs = read_sensor(m_goal , [x(i) y(i)]);
                fprintf(LogFile, '______________________________________________________________________________________________\n');
                fprintf(LogFile, 'i : %d \t x : %0.4f \t y : %0.4f \t m_goal_n : %0.4f \t lastmin : %0.4f\n', i, x(i), y(i), rad2deg(m_goal_n), lastmin );
                
                if(inpolygon(curPos(1), curPos(2), shPointX, shPointY))
                    mode = user_ModeTypes.GoDirectly;
                    m_goal = atan2( (qgoal(2)-curPos(2)) , (qgoal(1)-curPos(1)) );
                    dist_mgoal_dir2Obs = read_sensor(m_goal , [curPos(1) curPos(2)]);
                    fprintf(LogFile, 'Found the shortest Point!\n');                    
                    break;
                end
            end
            
            
            
        case user_ModeTypes.TurnAround
            fprintf(LogFile, 'TurnAround Mode!\n');
            %stop = true;
            %mode = user_ModeTypes.Finish;
            %Bulnma aralýðýný hesaplayalým
            L = linspace(0, 2*pi, 360);
            startX = curPos(1)+cos(L)*stepSize;
            startY = curPos(2)+sin(L)*stepSize;
            
            m_goal_n = m_goal;
            lastmin = infinity;
            %baþlangýç pozisyonuna gelene kadar alttaki döngü devam etsin
            while(true)
                [m_goal_n, lastmin] = user_change_dir(lastmin,curPos);
                i=i+1;                
                curPos(1) = curPos(1) + stepSize*cos(m_goal_n);
                curPos(2) = curPos(2) + stepSize*sin(m_goal_n);
                x(i) = curPos(1);
                y(i) = curPos(2);
                
                %dist_mgoal_dir2Obs = read_sensor(m_goal , [x(i) y(i)]);
                [shortestPoints, shortestDistance]  = user_shortestPoints(curPos, shortestPoints);
                fprintf(LogFile, '______________________________________________________________________________________________\n');
                fprintf(LogFile, 'i : %d \t x : %0.4f \t y : %0.4f \t m_goal_n : %0.4f \n', i, x(i), y(i), rad2deg(m_goal_n) );
                fprintf(LogFile, 'shPos_x : %0.4f \t shPos_y : %0.4f \t shdis : %0.4f \t lastmin : %0.4f\n', shortestPoints(1), shortestPoints(2), shortestDistance, lastmin );
                
                if( shortestDistance <= stepSize)
                    mode = user_ModeTypes.Finish;
                    finish_type = 2;
                    break;
                end
                
                if(inpolygon(curPos(1), curPos(2), startX, startY))
                    mode = user_ModeTypes.TurntoShortestPoint;
                    fprintf(LogFile, 'Found the first point!\n')
                    break;
                end 

            end
            
            
        case user_ModeTypes.Finish
            fprintf(LogFile, 'Finish Mode!\n');
            if(finish_type == 1)
                fprintf(LogFile, 'In m_goal direction, distance between obstacle and bug is smaller than sensor_range !\n');
            elseif(finish_type == 2)
                fprintf(LogFile, 'qgoal and bug is near. Stepsize is greater than their distance!\n');
            else
                fprintf(LogFile, 'Need to look!\n');
            end
            stop = true;
        otherwise
            fprintf(LogFile, 'A Fail Happened!\n');
            stop = true;
    end
    
end

end



