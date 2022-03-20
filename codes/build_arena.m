%
% ---- EE596 Mini Project ------------------------------------------
%
% build_arena( n )
%  Interactively builds an arena with n obstacles based on user
%  input. The resulting arena data is placed in the global variable
%  arena_map.
%
%  Inputs:  n: number of obstacles to input
%  Outputs: None
%
% --------------------------------------------------------------

function build_arena(n)

global arena_map;

arena_map = [];

for i=1:n
    disp(sprintf('Please select all the vertices of obstacle %i and press Enter',i));
    xlim([0 10]);
    ylim([0 10]);
    grid on;
    [x,y] = ginput;
    
    arena_map{i} = [x y];
    draw_arena();
    arena_map{i}
end
