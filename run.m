function run

map_size(1).r = 10;
map_size(1).c = 10;

map_size(2).r = 15;
map_size(2).c = 15;

map_size(3).r = 20;
map_size(3).c = 20;

map_size(4).r = 25;
map_size(4).c = 25;

map_size(5).r = 30;
map_size(5).c = 30;

obstacle_percentage = [0.1 0.2 0.3 0.4 0.5 0.6 0.7];


for i = 1:size(map_size,2)
  r = map_size(i).r;
  c = map_size(i).c;
  for j = 1:size(obstacle_percentage,2)
    op = obstacle_percentage(j);

    data(j).po = op;
    
    for it = 1:100
      clear map; close all;
      
      map = random_gen_map(r,c,op);
      
      tic
      jumppointsearch(map);
      data(j).etime_jps(it) = toc;
      
      %saveas(gcf,sprintf('r%dc%dpo%0.2fi%d.jpg',r,c,po,i),'jpg');
      %save(sprintf('map-r%dc%dpo%0.2fi%d',r,c,po,i),'map');
      
      %print(gcf,'my-plot','-dpng');
      %close all;
      tic
      astar(map);
      data(j).etime_as(it) = toc;
      pause(0.1);
    end
  end
  
  % plot them
  
  
end
