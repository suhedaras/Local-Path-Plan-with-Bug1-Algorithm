%
% ---- EE596 Mini Project ------------------------------------------
%
% draw_range_map( position, steps )
%  Draws an illustration of the range sensor map assuming the robot
%  is located at the specified position. Sensor readings are
%  plotted for the range [0,2*pi] with the given number of discrete
%  steps.
%
%  Inputs:  position: Robot position (row vector [x y])
%
%  Outputs: None
%
% --------------------------------------------------------------

function draw_range_map( position, steps )

global sensor_range arena_limits infinity;

position = [position(1) position(2)];
            
clf;
subplot(1,2,1);
draw_arena();
hold on;

distances = [];
angles = linspace( 0, 2*pi, steps );
for angle = linspace( 0, 2*pi, steps )
  distance = read_sensor( angle, position );
  distances = [distances distance];
  if ( distance < infinity /2 ) 
    sensor_ray_end = position + distance * [cos(angle) sin(angle)];
    sensor_ray = [position; sensor_ray_end];
    line(sensor_ray(:,1), sensor_ray(:,2));
  end
end
circle(position, sensor_range, steps, '-');
hold off;

subplot(1,2,2);
plot( rad2deg(angles), distances );

axis( [0 rad2deg(2*pi) 0 2*sensor_range] );
grid on;
title('Sensor Measurement');
xlabel('Measurement Angle (degrees)');
ylabel('Measured Distance (m)');
