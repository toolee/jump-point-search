function jumppointsearch(input_map)
clc; clear all; close all;

global map; global ROW; global COL; global S; global G; global C; global O;

% validate input map
S = 7;
G = 8;
C = 1;
O = 0;

if( nargin == 0 )

small_map = ...
      [ S, C, C, C;
        C, C, O, C;
        C, O, O, C;
        C, C, C, G;
        ];
small_map2 = ...
      [ S, C, C;
        O, O, C;
        C, G, C;
        ];

small_map3 = ...
      [ S, C, C;
        O, C, C;
        C, C, G;
        ];

large_map = ...
      [ C, C, C, C, C, C, C, C, C, C;
        C, O, O, C, O, C, C, C, C, C;
        C, S, O, C, O, C, C, C, C, C;
        C, O, O, C, O, C, O, C, C, C;
        C, C, C, C, C, C, O, C, O, C;
        O, O, O, O, O, O, C, C, O, C;
        C, C, C, C, C, C, C, C, O, C;
        C, O, O, C, O, O, O, C, O, C;
        C, C, C, C, C, C, O, C, C, C;
        G, C, C, C, C, C, O, C, C, C;
        ];
      
no_path_large_map = ...
      [ C, C, C, C, O, C, C, C, C, C;
        C, O, O, C, O, C, C, C, C, C;
        C, S, O, C, O, C, C, C, C, C;
        C, O, O, C, O, C, C, C, C, C;
        C, C, C, C, C, C, C, C, O, C;
        C, C, C, C, C, C, C, C, O, C;
        C, C, C, C, C, C, C, C, O, C;
        O, O, O, O, O, O, O, O, O, O;
        C, C, C, C, C, C, C, C, C, C;
        G, C, C, C, C, C, C, C, C, C;
        ];
map = small_map;
%map = large_map;
%map = small_map2;
elseif ( nargin == 1)
  map = input_map;
end

global ROW;
global COL;
[ROW,COL] = size(map);
display(sprintf('INFO: Map size = %d x %d',ROW,COL));
display(sprintf('INFO: 0 - obstacle | 1 - clear path | 7 - start | 8 - goal'));

%------------------------------------
% 2) Validate map, and capture start, goal position
% Draw the map if valid
%------------------------------------
num_start = 0;
num_goal = 0;
for r = 1:ROW
  for c = 1:COL
    if( map(r,c) == S )
      num_start = num_start + 1;
      start_r = r; start_c = c;
    end
    if( map(r,c) == G )
      num_goal = num_goal + 1;
      goal_r = r; goal_c = c;
    end
  end
end
if( num_start > 1 || num_goal > 1 )
  display('ERROR: Validate map: Too many start or goal');
  return;
end

% draw a map
axis([1 COL+1 1 ROW+1]);
grid on;
hold on;
set(gca,'XTick',[1:1:COL]);
set(gca,'YTick',[1:1:ROW]);
set(gca,'xaxislocation','top','ydir','reverse');

% plot start, goal, obstacles
plot(start_c+0.5,start_r+0.5,'ro');
plot(goal_c+0.5,goal_r+0.5,'go');
for ri = 1:ROW
  for ci = 1:COL
    if(map(ri,ci)==O) % if it is a obstacle draw it
      plot(ci+0.5,ri+0.5,'kx');
    end
  end
end

% TESTING
%neighbor_rc(1,1)
%neighbor_rc(2,2)
%neighbor_rc(3,3)
%neighbor_rc(4,4)
%neighbors(3,3)


% identify successor
cur_node_r = start_r;
cur_node_c = start_c;

% traverse the map

%--------------------------------------------------------------------------
% function: neighbor_rc
% param   : r current row
% param   : c current col
% return  : r start/end, c start/end
%--------------------------------------------------------------------------
function [rs,re,cs,ce] = neighbor_rc(r,c)
global map; global ROW; global COL; global S; global G; global C; global O;
%    r-1
% c-1   c+1
%    r+1
if( r-1 > 0   ) rs = r-1; else rs = 1; end
if( r+1 < ROW ) re = r+1; else re = ROW; end
if( c-1 > 0   ) cs = c-1; else cs = 1; end
if( c+1 < COL ) ce = c+1; else ce = COL; end

%display(sprintf('DEBUG: neighbors(): %d,%d - %d,%d,%d,%d',r,c,rs,re,cs,ce));

%--------------------------------------------------------------------------
% function: neighbors
% param   : r current row
% param   : c current col
% return  : all neighbor including obstacles
%--------------------------------------------------------------------------
function n = neighbors(r,c)
global map; global ROW; global COL; global S; global G; global C; global O;

[rs,re,cs,ce] = neighbor_rc(r,c);

i = 1;

for ri = rs:re
  for ci = cs:ce
      n(i).r = ri;
      n(i).c = ci;
      i = i+1;
  end
end

%--------------------------------------------------------------------------
% function: prune
% param   : r current row
% param   : c current col
% param   : n all neighbors
% return  : pruned neightbors
%--------------------------------------------------------------------------
function prune(r,c,n)
global map; global ROW; global COL; global S; global G; global C; global O;

function jump