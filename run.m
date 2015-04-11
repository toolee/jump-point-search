function run
for i = 1:10
clear map; close all;
r = 50;
c = 50;
po = 0.0;
map = random_gen_map(r,c,po);
tic
jumppointsearch(map);
toc
%saveas(gcf,sprintf('r%dc%dpo%0.2fi%d.jpg',r,c,po,i),'jpg');
%print(gcf,'my-plot','-dpng');
%close all;
tic
%astar(map);
toc
pause(0.1);
end