%
% EE596 Robot Motion Planning
%
% Mini Project 1 - Implementing the Bug1 algorithm
%
%  This file is a template against which your solution will be
% tested. It provides wrapper functions as well as various
% utilities that may be useful for your implementation
%
%  Your solution is required to implement the following function
%
%   [x,y] = bug_planner( qstart, qgoal )
%
% This function is expected to return a collision free path
% starting from qstart and ending in qgoal, both supplied as row
% vectors. The returned values should be row vectors with the x and
% y coordinates of the robot along this path, as it would have
% followed with bug1 and bug2 algorithms in effect, respectively.
%
% In implementing these functions, you are only allowed to use the
% following functions supplied with the Mini Project-1 Package (as well as any
% others you may choose to implement yourself):
%
%  draw_arena, draw_range_map, read_sensor
%
%  Furthermore, you are only allowed to access the following global
% variables:
%
%  sensor_range, infinity
%
%  You may of course write your own functions as you see fit as long
% as you do not cheat by accessing the global arena map and follow
% submission guidelines for this Mini Project. You may also use various
% builtin Matlab functions as you see fit.
%
%  Please follow good coding standards and document your code with
% useful and clear comments in English.
%
% All quantities are in MKS units unless otherwise specified.
%  distances    : m
%  angles       : rad
%  speed        : m/s
%  acceleration : m/s^2
%

function EE596_MP1

clear all;
clc;

vidObj = VideoWriter('mustafa_yilmaz.avi');
open(vidObj);

global LogFile;
File_index = 'logFile.log';

LogFile = fopen(File_index,'w');


% Global Parameters Declarations -----------------
global sensor_range;  % Determines limited sensor range
global arena_limits;  % Boundaries of the arena: [xmin xmax ymin ymax]
global arena_map;     % Description of obstacles in the environment
global infinity;      % Large value to be used as 'infinity'
global qstart qgoal;  % Start and goal configurations

% Parameter values to be used for the Mini Project ---
sensor_range = 0.2; % For Bug 1-2 algorithms it should be < 0.2
infinity = 1e5;
arena_limits = [0 10 0 10];
arena_map = [];

% Invoking your solutions for the example arena ------------------------
init_arena();

[x_m1_b1, y_m1_b1] = bug_planner( qstart, qgoal );

% Draws the planned motion path ------------------------------------
figure(1);
clf(1);
draw_arena;
hold on; plot( x_m1_b1, y_m1_b1 );

savefig('mustafa_yilmaz2.fig');

% return;
% Animates the robot motion and sensor measurements ------------------
figure(2);
clf(2);
for i = 1:length(x_m1_b1)
  draw_range_map( [x_m1_b1(i) y_m1_b1(i)], 30 );
  drawnow;
  
  % Write each frame to the file.
  currFrame = getframe(gcf);
  writeVideo(vidObj,currFrame);
  
end

fclose(LogFile);

close(vidObj);

% -----------------------------------------------------------------
% init_arena
% Definition of the example arena map for Mini Project 1
% -----------------------------------------------------------------
function init_arena
global arena_map qstart qgoal;

arena_map = [];
i=0;
%(x,y) coordinates of vertices of each object: e.g. [x1 y1; x2 y2; x3 y3; .... ]
i=i+1; arena_map{i} = ...
[ 2.0392  3.5234; 1.8318  5.0731; 2.0161  6.7982; 2.6152  8.1433; ...
  3.5369  8.9035; 5.0576  9.0205; 6.3249  8.8158; 7.4078  7.8509; ...
  8.0300  6.8275; 8.1452  4.8977; 8.0760  3.4357; 7.6613  2.1784; ...
  6.8548  1.1842; 5.3571  0.8041; 4.3433  1.0965; 3.6751  2.5000; ...
  3.5369  3.7281; 3.4447  4.8099; 3.9977  6.2135; 4.5968  6.7982; ...
  5.0115  6.5936; 4.5968  5.6287; 4.1820  4.6930; 4.0668  3.7865; ...
  4.1129  2.5877; 4.5276  1.7398; 5.4954  1.3596; 6.6705  1.6228; ...
  7.2465  2.4415; 7.6843  3.6696; 7.7765  5.1608; 7.6843  6.3304; ...
  6.9700  7.3246; 6.1866  8.1140; 5.0346  8.3480; 3.7673  8.2018; ...
  3.1682  7.7924; 2.4539  6.7105; 2.3848  5.1023];
i=i+1; arena_map{i} = ...
[ 5.2889  5.1131; 4.7111  4.2839; 4.8869  3.5302; ...
  6.1683  3.9070; 6.1432  5.0377 ];
i=i+1; arena_map{i} = ...
[ 5.1382    7.2487; 5.3392  6.8719; 5.3392  6.3693; ...
    5.2889  6.0427; 5.5402  6.0930; 5.7412  6.4447; ...
    5.7412  6.9724; 5.4899  7.2990; 5.2638  7.4749 ];
i=i+1; arena_map{i} = ... % A square object between coordinates (1, 1) and (1.5, 1.5)
[ 1 1; 1.5 1; 1.5 1.5; 1 1.5  ];

qstart = [2 9];
qgoal = [6 3];
