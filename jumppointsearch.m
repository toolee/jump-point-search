function jumppointsearch(input_map)
clc; clear all; close all;

global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;

% setup direction constants
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;
NW   = 1;   NORTH  = 2;  NE    = 3;
WEST = 4;   CENTER = 0;  EAST  = 6;
SW   = 7;   SOUTH  = 8;  SE    = 9;

create_map_symbols();

if( nargin == 0 )
  map = use_canned_map();
elseif ( nargin == 1)
  map = input_map;
end

global ROW;
global COL;
[ROW,COL] = size(map);
display(sprintf('INFO: Map size = %d x %d',ROW,COL));
display(sprintf('INFO: 0 - obstacle | 1 - clear path | 7 - start | 8 - goal'));

validate_map();

% draw an initial map
draw_map();


% TESTING
%neighbor_rc(1,1)
%neighbor_rc(2,2)
%neighbor_rc(3,3)
%neighbor_rc(4,4)
prune(neighbors(3,3));

%a.r = 3; a.c = 3;
%b.r = 4; b.c = 4;
%display(['INFO: ' dir_string(direction(a,b))]);



% identify successor
cur_node_r = start_r;
cur_node_c = start_c;

% traverse the map

%--------------------------------------------------------------------------
% function: neighbor_se
% param   : r current row
% param   : c current col
% return  : r (s)tart/(e)nd, c (s)tart/(e)nd
%--------------------------------------------------------------------------
function [rs,re,cs,ce] = neighbor_se(r,c)
global map; global ROW; global COL; global S; global G; global C; global O;
%    r-1
% c-1   c+1
%    r+1
if( r-1 > 0   ) rs = r-1; else rs = 1;   end
if( r+1 < ROW ) re = r+1; else re = ROW; end
if( c-1 > 0   ) cs = c-1; else cs = 1;   end
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

[rs,re,cs,ce] = neighbor_se(r,c);

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
% param   : n all neighbors
% return  : pruned neighbors
%--------------------------------------------------------------------------
function prune(n)
global map; global ROW; global COL; global S; global G; global C; global O;

ii = 1;
for i = 1:size(n,2)
  if ( map( n(i).r, n(i).c ) ~= O )
    nn(ii).r = n(i).r;
    nn(ii).c = n(i).c;
    nn(ii)
    ii = ii+1;
  end
end

%--------------------------------------------------------------------------
% function: jump
% param   : x initial node
% param   : dir direction
% param   : s start node
% param   : g goal node
% return  : n jump point
%--------------------------------------------------------------------------
function n = jump(x,dir,s,g)


%--------------------------------------------------------------------------
% function: create_map_symbols
% param   :
% return  : set global constants
%--------------------------------------------------------------------------
function create_map_symbols()
global S; global G; global C; global O;
S = 7;
G = 8;
C = 1;
O = 0;

%--------------------------------------------------------------------------
% function: use_canned_map
% param   :
% return  : canned map
%--------------------------------------------------------------------------
function ret = use_canned_map()
global S; global G; global C; global O;
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
ret = small_map;
 
%--------------------------------------------------------------------------
% function: validate_map
%   check for existence of start, goal, and number of occurances
% param   :
% return  : captures START, GOAL
%--------------------------------------------------------------------------
function validate_map()
global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;
num_start = 0;
num_goal = 0;
for r = 1:ROW
  for c = 1:COL
    if( map(r,c) == S )
      num_start = num_start + 1;
      START.r = r; START.c = c;
    end
    if( map(r,c) == G )
      num_goal = num_goal + 1;
      GOAL.r = r; GOAL.c = c;
    end
  end
end
if( num_start > 1 || num_goal > 1 )
  display('ERROR: Validate map: Too many start or goal');
  return;
end

%--------------------------------------------------------------------------
% function: draw_map
%   Draws the initial base map with start, goal, and obstacles
% param   :
% return  :
%--------------------------------------------------------------------------
function draw_map()
global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;
axis([1 COL+1 1 ROW+1]);
grid on;
hold on;
set(gca,'XTick',[1:1:COL]);
set(gca,'YTick',[1:1:ROW]);
set(gca,'xaxislocation','top','ydir','reverse');

% plot start, goal, obstacles
plot(START.c+0.5, START.r+0.5, 'ro');
plot(GOAL.c+0.5, GOAL.r+0.5, 'go');
for ri = 1:ROW
  for ci = 1:COL
    if( map(ri,ci) == O ) % if it is a obstacle draw it
      plot(ci+0.5, ri+0.5, 'kx');
    end
  end
end

%--------------------------------------------------------d------------------
% function: dir_north_south
%   figures out the direction for NORTH or SOUTH
% param   : x
% param   : n
% return  : North, South, 0 for no change
%--------------------------------------------------------------------------
function dir = dir_north_south(x,n)
global NORTH; global EAST; global SOUTH; global WEST;
global NW; global NE; global SW; global SE;
% Test direction UP
d = n.r - x.r;
if     ( d == 0 ) % on the same row
  dir = 0; return;
elseif ( d  > 0 ) % n is DOWN
  dir = SOUTH; return;
else              % n is UP
  dir = NORTH; return;
end

%--------------------------------------------------------------------------
% function: dir_east_west
%   figures out the direction for EAST or WEST
% param   : x
% param   : n
% return  : EAST, WEST, 0 for no change
%--------------------------------------------------------------------------
function dir = dir_east_west(x,n)
global NORTH; global EAST; global SOUTH; global WEST;
global NW; global NE; global SW; global SE;
% Test direction UP
d = n.c - x.c;
if     ( d == 0 ) % on the same col
  dir = 0; return;
elseif ( d  > 0 ) % n is EAST
  dir = EAST; return;
else              % n is WEST
  dir = WEST; return;
end

%--------------------------------------------------------------------------
% function: direction
%   figures out the direction of a node to another
% param   : x
% param   : n
% return  : actual direction
%--------------------------------------------------------------------------
function dir = direction(x,n)
global NORTH; global EAST; global SOUTH; global WEST;
global NW; global NE; global SW; global SE;

d1 = dir_north_south(x,n);
d2 = dir_east_west(x,n);

if ( d1 == 0 && d2 == 0 )
  display('ERROR: direction () same node');
end

if ( d1 == NORTH && d2 == WEST )
  dir = NW;
elseif ( d1 == NORTH && d2 == EAST )
  dir = NE;
elseif ( d1 == SOUTH && d2 == WEST )
  dir = SW;
elseif ( d1 == SOUTH && d2 == EAST )
  dir = SE;
elseif ( d1 == 0 )
  dir = d2;
elseif ( d2 == 0 )
  dir = d1;
end

%--------------------------------------------------------------------------
% function: dir_string
%   print direction
% param   : dir
% return  : none
%--------------------------------------------------------------------------
function str = dir_string(dir)
global NORTH; global EAST; global SOUTH; global WEST;
global NW; global NE; global SW; global SE;

switch dir
  case 0
    str = sprintf('CENTER');
  case NORTH
    str = sprintf('NORTH');
  case EAST
    str = sprintf('EAST');
  case SOUTH
    str = sprintf('SOUTH');
  case WEST
    str = sprintf('WEST');
  case NW
    str = sprintf('NW');
  case NE
    str = sprintf('NE');
  case SW
    str = sprintf('SW');
  case SE
    str = sprintf('SE');
  otherwise
    str = sprintf('ERROR: unknown direction');
end

%--------------------------------------------------------------------------
% function: step
%   one step over the direction specified
% param   : x initial node
% param   : dir direction
% return  : n new node
%--------------------------------------------------------------------------
function step(x,dir)


