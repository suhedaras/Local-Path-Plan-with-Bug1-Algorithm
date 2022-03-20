%
%  ---- EE596 Mini Project ------------------------------------------
%
% draw_arena()
%  Draws the current layout of the arena.
%
%  Inputs:  None
%  Outputs: None
%
% --------------------------------------------------------------

function draw_arena();

global arena_map arena_limits qstart qgoal;

xmin = arena_limits(1);
xmax = arena_limits(2);
ymin = arena_limits(3);
ymax = arena_limits(4);

line([xmin xmin xmax xmax xmin], [ymin ymax ymax ymin ymin]);

for i = 1:length(arena_map);
  obstacle = arena_map{i};
  patch(obstacle(:,1), obstacle(:,2),'black');
end
hold on;
plot(qstart(1), qstart(2), 'o');
plot(qgoal(1), qgoal(2), 'x');
hold off;
axis tight;
axis square;
grid on;

