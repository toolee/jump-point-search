function map = random_gen_map(r,c,percent_obst)

% r = 10;
% c = 10;
% percent_obst = 0.9;

global ROW; global COL; global S; global G; global C; global O;
global START; global GOAL;

% setup direction constants
global NORTH; global EAST; global SOUTH; global WEST; global CENTER;
global NW; global NE; global SW; global SE;
NW   = 1;   NORTH  = 2;  NE    = 3;
WEST = 4;   CENTER = 0;  EAST  = 6;
SW   = 7;   SOUTH  = 8;  SE    = 9;

% create map symbols
S = 7;
G = 8;
C = 1;
O = 0;

map = ones(r,c);

s.r = rand() * r;
s.r = ceil(s.r);

s.c = rand() * c;
s.c = ceil(s.c);

map(s.r, s.c) = S;

g.r = s.r;
g.c = s.c;

while( s.r == g.r && s.c == g.c )
  
  g.r = rand() * r;
  g.r = ceil(g.r);
  
  g.c = rand() * c;
  g.c = ceil(g.c);
end

map(g.r, g.c) = G;

for i = 1:r
  for j = 1:c
    if ( (i == s.r && j == s.c) || (i == g.r && j == g.c) )
      continue;
    else
      k = rand();
      if ( k <= percent_obst )
        map(i,j) = O;
      end
    end
  end
end

map;


