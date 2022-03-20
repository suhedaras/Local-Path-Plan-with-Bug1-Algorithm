%
% ---- EE596 Mini Project ------------------------------------------
%
% distance = read_sensor( angle, position )
%  Returns the distance to the closest obstacle at the given angle for
%  a robot located at the specified position. If there are no
%  obstacles in sensor range, a large value (1e5) is returned as a
%  substitute for infinity.
%
%  Inputs:  angle   : Angle for the sensor reading, relative to the x
%                     axis
%           position: Robot position (row vector [x y])
%
%  Outputs: distance : Distance to the closest obstacle in the
%                      given direction
%
% --------------------------------------------------------------

function distance = read_sensor( angle, position )

global sensor_range arena_map arena_limits infinity;
distance = 0;

position = [position(1) position(2)];

% First, determine if the position is outside the boundaries
xmin = arena_limits(1);
xmax = arena_limits(2);
ymin = arena_limits(3);
ymax = arena_limits(4);

if ( position(1) < xmin || position(1) > xmax ...
     || position(2) < ymin || position(2) > ymax )
  return;
end

% Determine if the robot is inside any of the obstacles
for i = 1:length(arena_map)
  obstacle = arena_map{i};
  if (inpolygon(position(1), position(2), obstacle(:,1), obstacle(: ,2)))
    return;
  end
end

% Now that we know the robot is outside all of the obstacles and
% inside the arena, determine the closest obstacle in the given
% direction

% Infinity is modeled as a very large value
distance = infinity;

% Initialize the ray in the direction of the given angle
sensor_distance = min(norm([xmax - xmin ymax - ymin]), sensor_range);
sensor_ray_end = position + sensor_distance * [cos(angle) sin(angle)];
sensor_ray = [position; sensor_ray_end];

% Start from the boundary
obstacle = [xmin xmin xmax xmax xmin; ymin ymax ymax ymin ymin]';
[xi,yi] = curveintersect(obstacle(:,1), obstacle(:,2), ...
                         sensor_ray(:,1), sensor_ray(:,2));
dist = distance;
for j=1:length(xi)
  dist(j+1) = norm([xi(j)-position(1) yi(j)-position(2)]);
end
distance = min( dist );

% Loop through the obstacles
for i=1:length(arena_map)
  obstacle = arena_map{i};
  obstacle = [obstacle; obstacle(1,:)];
  [xi,yi] = curveintersect(obstacle(:,1), obstacle(:,2), ...
                           sensor_ray(:,1), sensor_ray(:,2));
  dist = distance;
  for j=1:length(xi)
    dist(j+1) = norm([xi(j)-position(1) yi(j)-position(2)]);
  end
  distance = min( dist );
  
end