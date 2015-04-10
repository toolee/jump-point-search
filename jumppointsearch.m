function jumppointsearch(input_map)
clc; clear all; close all;
% This will only run unit test then exit
unit_test = true;


global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;

% setup direction constants
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;
NW   = 1;   NORTH  = 2;  NE    = 3;
WEST = 4;   CENTER = 0;  EAST  = 6;
SW   = 7;   SOUTH  = 8;  SE    = 9;

global INIT_G_VALUE; global INIT_F_VALUE;

% create map symbols
S = 7;
G = 8;
C = 1;
O = 0;

if( nargin == 0 )
  map = use_canned_map(unit_test);
elseif ( nargin == 1)
  map = input_map;
end

global ROW;
global COL;
[ROW,COL] = size(map);
display(sprintf('INFO: Map size = %d x %d',ROW,COL));
display(sprintf('INFO: 0 - obstacle | 1 - clear path | 7 - start | 8 - goal'));

validate_map();

map_set_start_pos;
map_set_goal_pos;

% draw an initial map
draw_map();

if ( unit_test )
  TEST_dir_east_west;
  TEST_dir_north_south
  TEST_step;
  TEST_is_forced_neighbor_exist
  TEST_identify_successor;
  TEST_prune2
  %return;
end

% compute h values for all nodes, create nodes for the first time
nodes = astar_compute_h;

nodes = draw_fgh_value(nodes);

keep_running = false;
start_index = rc2indx(START.r,START.c);
cur_n = nodes(start_index);
dir = CENTER;
oi = 1;

while not( is_same_node(cur_n,GOAL) )
  cur_n_hdlr = plot(cur_n.c + 0.2,cur_n.r + 0.2,'r*');
  [scr,cnt] = identify_successor(cur_n,dir);
  if ( cnt > 0 )
    for i = 1:size(scr,2)
      %scr(i)
      scr(i).hldr = plot(scr(i).c + 0.1, scr(i).r + 0.1, 'b*');
      [nodes,indx] = update_f_g_value(scr(i),cur_n, nodes);
      % add to open_list
      open_list(oi) = indx;
      oi = oi + 1;
    end
  end
  nodes = draw_fgh_value(nodes);
  
  delete (cur_n_hdlr);
  for i = 1:size(scr,2)
    delete(scr(i).hldr);
  end
  
  % pop open_list
  oi = oi - 1;
  %pop_index = open_list(oi);
  
  % find smallest f value
  minv = 999999;
  for i = 1:size(open_list,2)
    open_list(i);
    nodes(open_list(i));
    if( nodes(open_list(i)).f < minv )
      minv = nodes(open_list(i)).f;
      pop_index = open_list(i);
      sm_i = i;
    end
  end
  
  
  open_list(sm_i) = [];
  parent_n = make_node_struct(nodes(pop_index).parent_r,...
    nodes(pop_index).parent_c);
  dir = direction(parent_n, nodes(pop_index));
  cur_n = nodes( pop_index );
end

% traverse the map
% traverse back
ti = rc2indx(GOAL.r,GOAL.c);
parent_n = make_node_struct(nodes(ti).parent_r,nodes(ti).parent_c);
if( is_outside(parent_n) )
    display('no path');
    return;
end

while (nodes(ti).r ~= START.r || nodes(ti).c ~= START.c)
    if( nodes(ti).c > nodes(ti).parent_c )
        xx = [nodes(ti).parent_c+0.5 nodes(ti).c+0.5];
    else
        xx = [nodes(ti).c+0.5 nodes(ti).parent_c+0.5];
    end
    if( nodes(ti).r > nodes(ti).parent_r )
        yy = [nodes(ti).parent_r+0.5 nodes(ti).r+0.5];
    else
        yy = [nodes(ti).r+0.5 nodes(ti).parent_r+0.5];
    end
    plot(xx,yy);
    ti = rc2indx(nodes(ti).parent_r,nodes(ti).parent_c);
    pause(0.5);
end

%--------------------------------------------------------------------------
% function: identify successor
% param   : x - current node
% return  : successors
%--------------------------------------------------------------------------
function [successors,si] = identify_successor(x,dir)
global CENTER
si = 0;
successors = [];
if ( dir == CENTER )
  the_nb = prune(neighbors(x.r,x.c));
else
  the_nb = prune2(x,dir);
end
for i = 1:size(the_nb,2)
  the_dir = direction(x,the_nb(i));
  n = jump(x,the_dir);
  if( ~isempty(n) )
    if ( isempty(successors) )
      clear successors;
    end
    si = si + 1;
    successors(si) = n;
  end
end



%--------------------------------------------------------------------------
% function: jump
% param   : x initial node
% param   : dir direction
% param   : s start node
% param   : g goal node
% return  : ret_n jump point - null - no jump point
%                            - non-null - actual jump point
%--------------------------------------------------------------------------
function ret_n = jump(x,dir)
global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;

% stepping toward 'dir'
n = step(x,dir);

ret_n = [];

if ( is_outside(n) )
  return;
end

if ( is_obstacle(n) )
  return;
end

if ( is_same_node(n,GOAL) )
  ret_n = n;
  return; 
end

if ( is_forced_neighbor_exist(n,dir) )
  ret_n = n;
  return;
end

if ( is_dir_diagonal(dir) )
  
  % set the opposite of the diagonal dir
  % d1 and d2 represent the striaght moves
  d = [];
  if ( dir == NW )
    d(1) = NORTH;
    d(2) = WEST;
  elseif ( dir == NE )
    d(1) = NORTH;
    d(2) = EAST;
  elseif ( dir == SW )
    d(1) = SOUTH;
    d(2) = WEST;
  elseif ( dir == SE )
    d(1) = SOUTH;
    d(2) = EAST;
  end
  
  % run thru d1 and d2 straight moves
  if ( size( jump(n,d(1)) ) > 0 )
    ret_n = n;
    return;
  end
  if ( size( jump(n,d(2)) ) > 0 )
    ret_n = n;
    return;
  end
end

% recurse with node n
ret_n = jump(n,dir);

%--------------------------------------------------------------------------
% function: is_forced_neighbor_exist
% param   : n - current node
% param   : dir - dir for parent of x to x
% return  : true or false
%--------------------------------------------------------------------------
function ret = is_forced_neighbor_exist(n,dir)
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;

ret = false;

if ( dir == CENTER )
  display('ERROR: is_forced_neighbor_exist(): CENTER'); 
  ret = false; 
  return; 
end

%--------------------------------------
% diagonal move
if ( dir == NW )
  n1 = make_node_struct( n.r, n.c + 1 );
  n2 = make_node_struct( n.r + 1, n.c );
  if ( is_inside(n1) && is_obstacle(n1) || is_inside(n2) && is_obstacle(n2) )
    ret = true;
    return;
  end
end

if ( dir == NE )
  n1 = make_node_struct( n.r, n.c - 1 );
  n2 = make_node_struct( n.r + 1, n.c );
  if ( is_inside(n1) && is_obstacle(n1) || is_inside(n2) && is_obstacle(n2) )
    ret = true;
    return;
  end
end

if ( dir == SW )
  n1 = make_node_struct( n.r, n.c + 1 );
  n2 = make_node_struct( n.r - 1, n.c );
  if ( is_inside(n1) && is_obstacle(n1) || is_inside(n2) && is_obstacle(n2) )
    ret = true;
    return;
  end
end

if ( dir == SE )
  n1 = make_node_struct( n.r, n.c - 1 );
  n2 = make_node_struct( n.r - 1, n.c );
  if ( is_inside(n1) && is_obstacle(n1) || is_inside(n2) && is_obstacle(n2) )
    ret = true;
    return;
  end
end

%----------------------------------------
% striaght move
if ( dir == EAST || dir == WEST )
  n1 = make_node_struct( n.r-1, n.c );
  n2 = make_node_struct( n.r+1, n.c );
  if ( is_inside(n1) && is_obstacle(n1) || is_inside(n2) && is_obstacle(n2) )
    ret = true;
    return;
  end
elseif ( dir == NORTH || dir == SOUTH )
  n1 = make_node_struct( n.r, n.c-1 );
  n2 = make_node_struct( n.r, n.c+1 );
  if ( is_inside(n1) && is_obstacle(n1) || is_inside(n2) && is_obstacle(n2) )
    ret = true;
    return;
  end
end

%--------------------------------------------------------------------------
% function: update_f_g_value
% param   : scr
% param   : cur_n
% return  :
%--------------------------------------------------------------------------
function [nodes,index] = update_f_g_value(scr,cur_n,nodes)
global INIT_G_VALUE;
index = rc2indx(scr.r,scr.c);
if ( cur_n.g == INIT_G_VALUE )
  nodes(index).g = distance(cur_n, scr);
else
  nodes(index).g = cur_n.g + distance(cur_n, scr);
end
nodes(index).f = nodes(index).g + nodes(index).h;
nodes(index).parent_r = cur_n.r;
nodes(index).parent_c = cur_n.c;
nodes(index)

%--------------------------------------------------------------------------
% function: distance
%   Only for straight distance computation
% param   : x - a node
% param   : y - another node
% return  : distance in integer
%--------------------------------------------------------------------------  
function d = distance(x,y)
% if ( is_dir_diagonal( direction(x,y) ) )
%   display('ERROR: distance(): should not be computing diagonal distance');
% end

d = sqrt(abs(x.r-y.r)^2 + abs(x.c-y.c)^2 );


%--------------------------------------------------------------------------
% function: rc2indx
% param   : r current row
% param   : c current col
% return  : index to nodes vector
%--------------------------------------------------------------------------
function index = rc2indx(r,c)
global COL;
index = (r-1)*COL+c;

%--------------------------------------------------------------------------
% function: indx2rc
% param   : i - index to nodes vector
% return  : r,c
%--------------------------------------------------------------------------
function [r,c] = indx2rc(i)
global COL;
r = ceil(i/COL);
c = mod(i,COL);
if( c == 0 )
  c = COL;
end

%--------------------------------------------------------------------------
% function: neighbor_start_end_index
% param   : r current row
% param   : c current col
% return  : r (s)tart/(e)nd, c (s)tart/(e)nd
%--------------------------------------------------------------------------
function [rs,re,cs,ce] = neighbor_start_end_index(r,c)
global map; global ROW; global COL; global S; global G; global C; global O;
%    r-1
% c-1   c+1
%    r+1
if( r-1 > 0   ) rs = r-1; else rs = 1;   end
if( r+1 < ROW ) re = r+1; else re = ROW; end
if( c-1 > 0   ) cs = c-1; else cs = 1;   end
if( c+1 < COL ) ce = c+1; else ce = COL; end

%--------------------------------------------------------------------------
% function: neighbors
% param   : r current row
% param   : c current col
% return  : all neighbor including obstacles
%--------------------------------------------------------------------------
function n = neighbors(r,c)
global map; global ROW; global COL; global S; global G; global C; global O;

[rs,re,cs,ce] = neighbor_start_end_index(r,c);

i = 1;

for ri = rs:re
  for ci = cs:ce
    if( ri == r && ci == c )
    else
      n(i).r = ri;
      n(i).c = ci;
      i = i+1;
    end
  end
end

%--------------------------------------------------------------------------
% function: prune
% param   : n all neighbors
% return  : pruned neighbors (no obstacle)
%--------------------------------------------------------------------------
function ret_n = prune(n)
global map; global ROW; global COL; global S; global G; global C; global O;

ii = 1;
for i = 1:size(n,2)
  if ( map( n(i).r, n(i).c ) ~= O )
    nn(ii).r = n(i).r;
    nn(ii).c = n(i).c;
    ii = ii+1;
  end
end

ret_n = nn;

function ret_n = prune2(n,dir)
global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;

% setup direction constants
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;
NW   = 1;   NORTH  = 2;  NE    = 3;
WEST = 4;   CENTER = 0;  EAST  = 6;
SW   = 7;   SOUTH  = 8;  SE    = 9;

n1 = make_node_struct( n.r-1, n.c-1 );
n2 = make_node_struct( n.r-1, n.c );
n3 = make_node_struct( n.r-1, n.c+1 );

n4 = make_node_struct( n.r, n.c-1 );
n6 = make_node_struct( n.r, n.c+1 );

n7 = make_node_struct( n.r+1, n.c-1 );
n8 = make_node_struct( n.r+1, n.c );
n9 = make_node_struct( n.r+1, n.c+1 );

ret_n = struct('r',[],'c',[]);
i = 1;
%----------------------------
if ( dir == EAST )
  % hit obstacle
  if ( is_outside(n6) || is_obstacle(n6) )
    % no negihbors
    ret_n = [];
    return;
  end
  
  ret_n(i) = n6; i = i+1;
  
  % if n2 is an obstacle, then n3 is a forced neighbor
  if ( is_inside(n2) && is_obstacle(n2) )
    ret_n(i) = n3;
    i = i+1;
  end
  
  if ( is_inside(n8) && is_obstacle(n8) )
    ret_n(i) = n9;
    i = i+1;
  end
  return;
end
%----------------------------
if ( dir == WEST )
  % hit obstacle
  if ( is_outside(n4) || is_obstacle(n4) )
    % no negihbors
    ret_n = [];
    return;
  end
  
  ret_n(i) = n4; i = i+1;
  
  % if n2 is an obstacle, then n1 is a forced neighbor
  if ( is_inside(n2) && is_obstacle(n2) )
    ret_n(i) = n1;
    i = i+1;
  end
  
  % if n8 is an obstacle, then n7 is a forced neighbor
  if ( is_inside(n8) && is_obstacle(n8) )
    ret_n(i) = n7;
    i = i+1;
  end
  return;
end
%----------------------------
if ( dir == NORTH )
  % hit obstacle
  if ( is_outside(n2) || is_obstacle(n2) )
    % no negihbors
    ret_n = [];
    return;
  end
  
  ret_n(i) = n2; i = i+1;

  if ( is_inside(n4) && is_obstacle(n4) )
    ret_n(i) = n1;
    i = i+1;
  end

  if ( is_inside(n6) && is_obstacle(n6) )
    ret_n(i) = n3;
    i = i+1;
  end
  return;
end
%----------------------------
if ( dir == SOUTH )
  % hit obstacle
  if ( is_outside(n8) || is_obstacle(n8) )
    % no negihbors
    ret_n = [];
    return;
  end
  
  ret_n(i) = n8; i = i+1;

  if ( is_inside(n4) && is_obstacle(n4) )
    ret_n(i) = n7;
    i = i+1;
  end

  if ( is_inside(n6) && is_obstacle(n6) )
    ret_n(i) = n9;
    i = i+1;
  end
  return;
end
%----------------------------
if ( dir == NE )
  if ( is_inside(n2) && is_not_obstacle(n2) )
    ret_n(i) = n2; i = i + 1;
  end
  if ( is_inside(n3) && is_not_obstacle(n3) )
    ret_n(i) = n3; i = i + 1;
  end
  if ( is_inside(n6) && is_not_obstacle(n6) )
    ret_n(i) = n6; i = i + 1;
  end
  
  if ( is_inside(n4) && is_obstacle(n4) )
    ret_n(i) = n1; i = i + 1;
  end
  
  if ( is_inside(n8) && is_obstacle(n8) )
    ret_n(i) = n9; i = i + 1;
  end
  
  return;
end
%----------------------------
if ( dir == NW )
  if ( is_inside(n1) && is_not_obstacle(n1) )
    ret_n(i) = n1; i = i + 1;
  end
  if ( is_inside(n2) && is_not_obstacle(n2) )
    ret_n(i) = n2; i = i + 1;
  end
  if ( is_inside(n4) && is_not_obstacle(n4) )
    ret_n(i) = n4; i = i + 1;
  end
  
  if ( is_inside(n6) && is_obstacle(n6) )
    ret_n(i) = n3; i = i + 1;
  end
  
  if ( is_inside(n8) && is_obstacle(n8) )
    ret_n(i) = n7; i = i + 1;
  end
  
  return;
end

%----------------------------
if ( dir == SW )
  if ( is_inside(n4) && is_not_obstacle(n4) )
    ret_n(i) = n4; i = i + 1;
  end
  if ( is_inside(n7) && is_not_obstacle(n7) )
    ret_n(i) = n7; i = i + 1;
  end
  if ( is_inside(n8) && is_not_obstacle(n8) )
    ret_n(i) = n8; i = i + 1;
  end
  
  if ( is_inside(n2) && is_obstacle(n2) )
    ret_n(i) = n1; i = i + 1;
  end
  
  if ( is_inside(n6) && is_obstacle(n6) )
    ret_n(i) = n9; i = i + 1;
  end
  
  return;
end

%----------------------------
if ( dir == SE )
  if ( is_inside(n6) && is_not_obstacle(n6) )
    ret_n(i) = n6; i = i + 1;
  end
  if ( is_inside(n8) && is_not_obstacle(n8) )
    ret_n(i) = n8; i = i + 1;
  end
  if ( is_inside(n9) && is_not_obstacle(n9) )
    ret_n(i) = n9; i = i + 1;
  end
  
  if ( is_inside(n2) && is_obstacle(n2) )
    ret_n(i) = n3; i = i + 1;
  end
  
  if ( is_inside(n4) && is_obstacle(n4) )
    ret_n(i) = n7; i = i + 1;
  end
  
  return;
end

%--------------------------------------------------------------------------
% function: is_same_node
%   compare r,c value only
% param   : x,y
% return  : true or false
%--------------------------------------------------------------------------
function ret = is_same_node(x,y)
ret = false;
if ( x.r == y.r && x.c == y.c )
  ret = true;
end

function ret = is_same_node2(n,r,c)
ret = false;
if ( n.r == r && n.c == c )
  ret = true;
end


%--------------------------------------------------------------------------
% function: use_canned_map
% param   :
% return  : canned map
%--------------------------------------------------------------------------
function ret = use_canned_map(unit_test)
global S; global G; global C; global O;
unit_test_map = ...
      [ S, C, C, C;
        C, O, C, C;
        C, C, C, C;
        C, C, C, G;
        ];

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

      small_map4 = ...
        [ S, C, C, C;
          C, C, O, C;
          O, O, O, C;
          C, C, C, C;
          G, C, O, C;
          C, O, O, C;
          C, C, C, C;
        ];
            small_map5 = ...
        [ S, C, C, C;
          C, C, C, C;
          C, C, C, C;
          C, C, C, C;
          C, C, C, C;
          C, C, C, C;
          C, C, G, C;
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
ret = small_map5;

% override to use unit_test_map
if ( unit_test )
  ret = unit_test_map;
end
 
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
% function: is_obstacle
% param   :
% return  : true or false
%--------------------------------------------------------------------------
function ret = is_obstacle(n)
global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;
ret = false;
if ( map(n.r, n.c) == O ) ret = true; return; end

%--------------------------------------------------------------------------
% function: is_not_obstacle
% param   :
% return  : true or false
%--------------------------------------------------------------------------
function ret = is_not_obstacle(n)
  ret = not(is_obstacle(n));
  
%--------------------------------------------------------------------------
% function: is_outside
% param   :
% return  : true or false
%--------------------------------------------------------------------------
function ret = is_outside(n)
global map; global ROW; global COL;
ret = false;
if ( n.r < 1 || ROW < n.r ) ret = true; return; end
if ( n.c < 1 || COL < n.c ) ret = true; return; end

%--------------------------------------------------------------------------
% function: is_inside
% param   : n - a node
% return  : true or false
%--------------------------------------------------------------------------
function ret = is_inside(n)
  ret = not(is_outside(n));

%--------------------------------------------------------------------------
% function: make_node_struct
% param   : r,c
% return  : node structure with r,c populated
%--------------------------------------------------------------------------
function n = make_node_struct(r,c)
  n.r = r; n.c = c;
  
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

%--------------------------------------------------------------------------
% function: is_dir_diagonal
% param   : dir
% return  : true or false
%--------------------------------------------------------------------------
function ret = is_dir_diagonal(dir)
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;
ret = false;
if ( dir == NW || dir == NE || dir == SW || dir == SE )
  ret = true;
  return;
end

%--------------------------------------------------------d------------------
% function: dir_north_south
%   figures out the direction for NORTH or SOUTH
% param   : x a node
% param   : n another node
% return  : North, South, 0 for no change
%--------------------------------------------------------------------------
function dir = dir_north_south(x,n)
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
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
% param   : x a node
% param   : n another node
% return  : EAST, WEST, 0 for no change
%--------------------------------------------------------------------------
function dir = dir_east_west(x,n)
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
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
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
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
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;

switch dir
  case 0
    str = sprintf('C');
  case NORTH
    str = sprintf('N');
  case EAST
    str = sprintf('E');
  case SOUTH
    str = sprintf('S');
  case WEST
    str = sprintf('W');
  case NW
    str = sprintf('NW');
  case NE
    str = sprintf('NE');
  case SW
    str = sprintf('SW');
  case SE
    str = sprintf('SE');
  otherwise
    str = sprintf('ERROR: dir_string(): unknown direction'); % this should not happen
end

%--------------------------------------------------------------------------
% function: step
%   one step over the direction specified
% param   : x initial node
% param   : dir direction
% return  : n new node, expected behavior is it could be outside the grid, 
%           or at an obstacle node
%--------------------------------------------------------------------------
function n = step(x,dir)
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;

switch dir
  case NORTH
    n.r = x.r - 1;
    n.c = x.c;
  case SOUTH
    n.r = x.r + 1;
    n.c = x.c;
  case EAST
    n.r = x.r;
    n.c = x.c + 1;
  case WEST
    n.r = x.r;
    n.c = x.c - 1;
  case NW
    n.r = x.r - 1;
    n.c = x.c - 1;
  case NE
    n.r = x.r - 1;
    n.c = x.c + 1;
  case SW
    n.r = x.r + 1;
    n.c = x.c - 1;
  case SE
    n.r = x.r + 1;
    n.c = x.c + 1;
  otherwise
    display('ERROR: step(): CENTER'); % this should not happen
    n.r = -1;
    n.c = -1;
end

%--------------------------------------------------------------------------
% function: draw_fgh_value
%   draw f, g, and h value on map
% param   : nodes - all the node structure
% return  : nodes - with some handler added or updated
%--------------------------------------------------------------------------
function nodes = draw_fgh_value(nodes)
global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;

% setup direction constants
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;
global INIT_G_VALUE;
global INIT_F_VALUE;
% draw h values
for ni = 1:size(nodes,2)
    % if it is not an obstacle draw debug info
    if( map(nodes(ni).r,nodes(ni).c) ~= O )
        %------------------------
        % draw f = g + h
        s = sprintf('%0.2f',nodes(ni).h);
        if( nodes(ni).h_hldr == 0 )
            nodes(ni).h_hldr = text(nodes(ni).c+0.8,nodes(ni).r+0.1,s);
        else
            set(nodes(ni).h_hldr,'String',s);
        end
        if( nodes(ni).g ~= INIT_G_VALUE )
            s = sprintf('%0.2f',nodes(ni).g);
            if( nodes(ni).g_hldr == 0 )
                nodes(ni).g_hldr = text(nodes(ni).c+0.45,nodes(ni).r+0.1,s);
            else
                set(nodes(ni).g_hldr,'String',s);
            end
        end
        if( nodes(ni).f ~= INIT_F_VALUE )
            s = sprintf('%0.2f',nodes(ni).f);
            if( nodes(ni).f_hldr == 0 )
                nodes(ni).f_hldr = text(nodes(ni).c+0.1,nodes(ni).r+0.1,s);
            else
                set(nodes(ni).f_hldr,'String',s);
            end
        end
        %----------------------------
        % draw arrow
        update_arrow = false;
        parent_n = make_node_struct(nodes(ni).parent_r,nodes(ni).parent_c);
        if ( is_inside(parent_n) )
          dir = direction(nodes(ni),parent_n);
          s = dir_string(dir);
          update_arrow = true;
          
%           if( 0 < nodes(ni).r+1 && nodes(ni).r+1 == nodes(ni).parent_r && nodes(ni).r+1 < ROW )
%             s = 'v'; update_arrow = true;
%           elseif( 0 < nodes(ni).r-1 && nodes(ni).r-1 == nodes(ni).parent_r && nodes(ni).r-1 < ROW )
%             s = '\^'; update_arrow = true;
%           elseif( 0 < nodes(ni).c+1 && nodes(ni).c+1 == nodes(ni).parent_c && nodes(ni).c+1 < COL )
%             s = '>'; update_arrow = true;
%           elseif( 0 < nodes(ni).c-1 && nodes(ni).c-1 == nodes(ni).parent_c && nodes(ni).c-1 < COL )
%             s = '<'; update_arrow = true;
%           end
          if( update_arrow )
            if( nodes(ni).arrow_hldr == 0 )
              nodes(ni).arrow_hldr = text(nodes(ni).c+0.05,nodes(ni).r+0.1,s);
            else
              set(nodes(ni).arrow_hldr,'String',s);
            end
          end
        end
    end
end

%--------------------------------------------------------------------------
% function: astar_compute_h
%   Compute the initial h value
% return  : nodes - with h value populated, f and g values are initialized
%--------------------------------------------------------------------------
function nodes = astar_compute_h
global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;

% setup direction constants
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;

nodes(ROW*COL) = struct('r',[],'c',[],'h',[],'g',[],'f',[], ...
    'parent_r',[],'parent_c',[],'h_hldr',[],'g_hldr',[],'f_hldr',[],...
    'arrow_hldr',[]);

[goal_r,goal_c] = find(map==G);

global INIT_G_VALUE;
INIT_G_VALUE = 99999;
global INIT_F_VALUE;
INIT_F_VALUE = 99999;

n = 1;
for r = 1:ROW
  for c = 1:COL
          nodes(n).r = r;
          nodes(n).c = c;
          nodes(n).h = sqrt( abs(goal_r-r)^2 + abs(goal_c-c)^2 );
          nodes(n).g = INIT_G_VALUE;
          nodes(n).f = INIT_F_VALUE;
          nodes(n).parent_r = 0;
          nodes(n).parent_c = 0;
          nodes(n).h_hldr = 0;
          nodes(n).g_hldr = 0;
          nodes(n).f_hldr = 0;
          nodes(n).arrow_hldr = 0;
          n = n+1;
  end
end

%--------------------------------------------------------------------------
% function: map_set_goal_pos
%   Set GOAL node
%--------------------------------------------------------------------------
function map_set_goal_pos
global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;
[r,c] = find(map == G);
GOAL.r = r;
GOAL.c = c;

%--------------------------------------------------------------------------
% function: map_set_start_pos
%   Set START node
%--------------------------------------------------------------------------
function map_set_start_pos
global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;
[r,c] = find(map == S);
START.r = r;
START.c = c;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TESTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function TEST_step
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;

status = 1;

x.r = 3; x.c = 3;
n = step(x,NORTH);
if ( ~(n.r == 2 && n.c == 3) ) status = 0; end

n = step(x,SOUTH);
if ( ~(n.r == 4 && n.c == 3) ) status = 0; end

n = step(x,EAST);
if ( ~(n.r == 3 && n.c == 4) ) status = 0; end

n = step(x,WEST);
if ( ~(n.r == 3 && n.c == 2) ) status = 0; end

n = step(x,NW);
if ( ~(n.r == 2 && n.c == 2) ) status = 0; end

n = step(x,NE);
if ( ~(n.r == 2 && n.c == 4) ) status = 0; end

n = step(x,SW);
if ( ~(n.r == 4 && n.c == 2) ) status = 0; end

n = step(x,SE);
if ( ~(n.r == 4 && n.c == 4) ) status = 0; end

if (status == 0) display('TEST: FAILED: step()'); else
  display('TEST: PASSED: step()');
end

function TEST_dir_east_west
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;

status = 1;

n1.r = 3; n1.c = 3;
n2.r = 5; n2.c = 6;
dir = dir_east_west(n1,n2);
if( dir ~= EAST ) status = 0; display('TEST: FAILED: dir_east_west: EAST'); end

n2.r = 5; n2.c = 1;
dir = dir_east_west(n1,n2);
if( dir ~= WEST ) status = 0; display('TEST: FAILED: dir_east_west: WEST'); end

if (status == 0) display('TEST: FAILED: dir_east_west'); else
  display('TEST: PASSED: dir_east_west()');
end

function TEST_dir_north_south
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;

status = 1;

n1.r = 3; n1.c = 3;
n2.r = 1; n2.c = 6;
dir = dir_north_south(n1,n2);
if( dir ~= NORTH ) status = 0; display('TEST: FAILED: dir_north_south: NORTH'); end

n2.r = 5; n2.c = 1;
dir = dir_north_south(n1,n2);
if( dir ~= SOUTH ) status = 0; display('TEST: FAILED: dir_north_south: SOUTH'); end

if (status == 0) display('TEST: FAILED: dir_north_south'); else
  display('TEST: PASSED: dir_north_south()');
end

function TEST_is_forced_neighbor_exist
global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;

global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;

status = 1;

n.r = 3; n.c = 2;
if ( is_forced_neighbor_exist(n,EAST) == false )
  display('TEST: FAILED: is_forced_neighbor_exist(): EAST');
end
if ( is_forced_neighbor_exist(n,WEST) == false )
  display('TEST: FAILED: is_forced_neighbor_exist(): WEST');
end
% the other side
n.r = 1; n.c = 2;
if ( is_forced_neighbor_exist(n,EAST) == false )
  display('TEST: FAILED: is_forced_neighbor_exist(): EAST 2');
end
if ( is_forced_neighbor_exist(n,WEST) == false )
  display('TEST: FAILED: is_forced_neighbor_exist(): WEST 2');
end

n.r = 2; n.c = 3;
if ( is_forced_neighbor_exist(n,NORTH) == false )
  display('TEST: FAILED: is_forced_neighbor_exist(): NORTH');
end
if ( is_forced_neighbor_exist(n,SOUTH) == false )
  display('TEST: FAILED: is_forced_neighbor_exist(): SOUTH');
end
% the other side
n.r = 2; n.c = 1;
if ( is_forced_neighbor_exist(n,NORTH) == false )
  display('TEST: FAILED: is_forced_neighbor_exist(): NORTH 2');
end
if ( is_forced_neighbor_exist(n,SOUTH) == false )
  display('TEST: FAILED: is_forced_neighbor_exist(): SOUTH 2');
end

% n.r = 2; n.c = 1;
% if ( is_forced_neighbor_exist(n,NE) == false )
%   display('TEST: FAILED: is_forced_neighbor_exist(): NE');
% end
% if ( is_forced_neighbor_exist(n,SE) == false )
%   display('TEST: FAILED: is_forced_neighbor_exist(): SE');
% end
% 
% n.r = 2; n.c = 3;
% if ( is_forced_neighbor_exist(n,NW) == false )
%   display('TEST: FAILED: is_forced_neighbor_exist(): NW');
% end
% if ( is_forced_neighbor_exist(n,SW) == false )
%   display('TEST: FAILED: is_forced_neighbor_exist(): SW');
% end

% f - forced neighbor O - obstacle p - parent x - current node
%   1  2  3  4
% 1    f  
% 2    O  x
% 3    p  O  f
% 4
n.r = 2; n.c = 3;
if ( is_forced_neighbor_exist(n,NE) == false )
  display('TEST: FAILED: is_forced_neighbor_exist(): NE');
end

% f - forced neighbor O - obstacle p - parent x - current node
%   1  2  3  4
% 1    p  O  f
% 2    O  x
% 3    f
% 4
n.r = 2; n.c = 3;
if ( is_forced_neighbor_exist(n,SE) == false )
  display('TEST: FAILED: is_forced_neighbor_exist(): SE');
end

% f - forced neighbor O - obstacle p - parent x - current node
%   1  2  3  4
% 1    f
% 2 x  O
% 3 O  p
% 4
n.r = 2; n.c = 1;
if ( is_forced_neighbor_exist(n,NW) == false )
  display('TEST: FAILED: is_forced_neighbor_exist(): NW');
end

% f - forced neighbor O - obstacle p - parent x - current node
%   1  2  3  4
% 1 O  p
% 2 x  O
% 3    f
% 4
if ( is_forced_neighbor_exist(n,SW) == false )
  display('TEST: FAILED: is_forced_neighbor_exist(): SW');
end


if ( status == 0 )
  display('TEST: FAILED: is_forced_neighbor_exist()');
else
  display('TEST: PASSED: is_forced_neighbor_exist()');
end

function TEST_jump
global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;

function TEST_identify_successor
global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;

% setup direction constants
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;

x.r = 4; x.c = 1;
successors =  identify_successor(x,CENTER);
if ( not( is_same_node( successors(1), struct('r',2,'c',1) ) ) )
  display('TEST: FAILED: identify_successor 2');
else
  display('TEST: PASSED: identify_successor 2');
end
if ( not( is_same_node( successors(2), struct('r',3,'c',2) ) ) )
  display('TEST: FAILED: identify_successor 2');
else
  display('TEST: PASSED: identify_successor 2');
end
if ( not( is_same_node( successors(3), struct('r',4,'c',4) ) ) )
  display('TEST: FAILED: identify_successor 2');
else
  display('TEST: PASSED: identify_successor 2');
end

x.r = 1; x.c = 1;
successors = identify_successor(x,CENTER);
if ( not( is_same_node( successors(1), struct('r',1,'c',2) ) ) )
  display('TEST: FAILED: identify_successor 1');
else
  display('TEST: PASSED: identify_successor 1');
end

x.r = 3; x.c = 3;
successors =  identify_successor(x,CENTER);
if ( not( is_same_node( successors(1), struct('r',2,'c',3) ) ) )
  display('TEST: FAILED: identify_successor 3.1');
else
  display('TEST: PASSED: identify_successor 3');
end
if ( not( is_same_node( successors(2), struct('r',3,'c',2) ) ) )
  display('TEST: FAILED: identify_successor 3.2');
else
  display('TEST: PASSED: identify_successor 3');
end
if ( not( is_same_node( successors(3), struct('r',4,'c',4) ) ) )
  display('TEST: FAILED: identify_successor 3.3');
else
  display('TEST: PASSED: identify_successor 3');
end

function TEST_prune2
global map; global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;

% setup direction constants
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;

%---------------------------------------------
n.r = 2; n.c = 1;
ret_n = prune2(n,EAST);
if ( isempty(ret_n) )
  display('TEST: PASSED: prune2(): E1');
else
  display('TEST: FAILED: prune2(): E1');
end

n.r = 3; n.c = 1;
ret_n = prune2(n,EAST);
if ( ret_n.r == 3 && ret_n.c == 2 )
  display('TEST: PASSED: prune2(): E2');
else
  display('TEST: FAILED: prune2(): E2');
end

n.r = 1; n.c = 2;
ret_n = prune2(n,EAST);
if ( ret_n(1).r == 1 && ret_n(1).c == 3 )
  display('TEST: PASSED: prune2(): E3');
else
  display('TEST: FAILED: prune2(): E3');
end
if ( ret_n(2).r == 2 && ret_n(2).c == 3 )
  display('TEST: PASSED: prune2(): E3');
else
  display('TEST: FAILED: prune2(): E3');
end

n.r = 4; n.c = 2;
ret_n = prune2(n,EAST);
if ( ret_n(1).r == 4 && ret_n(1).c == 3 )
  display('TEST: PASSED: prune2(): E4');
else
  display('TEST: FAILED: prune2(): E4');
end
%---------------------------------------------
n.r = 2; n.c = 3;
ret_n = prune2(n,WEST);
if ( isempty(ret_n) )
  display('TEST: PASSED: prune2(): W1');
else
  display('TEST: FAILED: prune2(): W1');
end

n.r = 3; n.c = 3;
ret_n = prune2(n,WEST);
if ( ret_n.r == 3 && ret_n.c == 2 )
  display('TEST: PASSED: prune2(): W2');
else
  display('TEST: FAILED: prune2(): W2');
end

n.r = 1; n.c = 2;
ret_n = prune2(n,WEST);
if ( ret_n(1).r == 1 && ret_n(1).c == 1 )
  display('TEST: PASSED: prune2(): W3');
else
  display('TEST: FAILED: prune2(): W3');
end
if ( ret_n(2).r == 2 && ret_n(2).c == 1 )
  display('TEST: PASSED: prune2(): W3');
else
  display('TEST: FAILED: prune2(): W3');
end

n.r = 4; n.c = 2;
ret_n = prune2(n,WEST);
if ( ret_n(1).r == 4 && ret_n(1).c == 1 )
  display('TEST: PASSED: prune2(): W4');
else
  display('TEST: FAILED: prune2(): W4');
end
%---------------------------------------------
n.r = 3; n.c = 2;
ret_n = prune2(n,NORTH);
if ( isempty(ret_n) )
  display('TEST: PASSED: prune2(): N1');
else
  display('TEST: FAILED: prune2(): N1');
end

n.r = 3; n.c = 3;
ret_n = prune2(n,NORTH);
if ( ret_n.r == 2 && ret_n.c == 3 )
  display('TEST: PASSED: prune2(): N2');
else
  display('TEST: FAILED: prune2(): N2');
end

n.r = 2; n.c = 3;
ret_n = prune2(n,NORTH);
if ( ret_n(1).r == 1 && ret_n(1).c == 3 )
  display('TEST: PASSED: prune2(): N3');
else
  display('TEST: FAILED: prune2(): N3');
end
if ( ret_n(2).r == 1 && ret_n(2).c == 2 )
  display('TEST: PASSED: prune2(): N3');
else
  display('TEST: FAILED: prune2(): N3');
end

n.r = 2; n.c = 4;
ret_n = prune2(n,NORTH);
if ( ret_n(1).r == 1 && ret_n(1).c == 4 )
  display('TEST: PASSED: prune2(): N4');
else
  display('TEST: FAILED: prune2(): N4');
end
%---------------------------------------------
n.r = 1; n.c = 2;
ret_n = prune2(n,SOUTH);
if ( isempty(ret_n) )
  display('TEST: PASSED: prune2(): S1');
else
  display('TEST: FAILED: prune2(): S1');
end

n.r = 1; n.c = 3;
ret_n = prune2(n,SOUTH);
if ( ret_n.r == 2 && ret_n.c == 3 )
  display('TEST: PASSED: prune2(): S2');
else
  display('TEST: FAILED: prune2(): S2');
end

n.r = 2; n.c = 1;
ret_n = prune2(n,SOUTH);
if ( ret_n(1).r == 3 && ret_n(1).c == 1 )
  display('TEST: PASSED: prune2(): S3');
else
  display('TEST: FAILED: prune2(): S3');
end
if ( ret_n(2).r == 3 && ret_n(2).c == 2 )
  display('TEST: PASSED: prune2(): S3');
else
  display('TEST: FAILED: prune2(): S3');
end

n.r = 2; n.c = 4;
ret_n = prune2(n,SOUTH);
if ( ret_n(1).r == 3 && ret_n(1).c == 4 )
  display('TEST: PASSED: prune2(): S4');
else
  display('TEST: FAILED: prune2(): S4');
end

%---------------------------------------------
n.r = 2; n.c = 3;
ret_n = prune2(n,NE);
if ( is_same_node2(ret_n(1), 1, 3) && ...
     is_same_node2(ret_n(2), 1, 4) && ...
     is_same_node2(ret_n(3), 2, 4) && ...
     is_same_node2(ret_n(4), 1, 2))
  display('TEST: PASSED: prune2(): NE1');
else
  display('TEST: FAILED: prune2(): NE1');
end

n.r = 1; n.c = 2;
ret_n = prune2(n,NE);
if ( is_same_node2(ret_n(1), 1, 3) && ...
     is_same_node2(ret_n(2), 2, 3))
  display('TEST: PASSED: prune2(): NE2');
else
  display('TEST: FAILED: prune2(): NE2');
end
