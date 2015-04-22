function run
clear all; close all;

ITERATIONS = 15;

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
  H = figure; hold on;
  r = map_size(i).r;
  c = map_size(i).c;
  for j = 1:size(obstacle_percentage,2)
    op = obstacle_percentage(j);
    
    data(i,j).op = op;
    
    for it = 1:ITERATIONS
      clear map; %close all;
      
      map = random_gen_map(r,c,op);
      
      tic
      jumppointsearch(map);
      data(i,j).etime_jps(it) = toc;
      close;
      
      %saveas(gcf,sprintf('r%dc%dpo%0.2fi%d.jpg',r,c,op,i),'jpg');
      %save(sprintf('map-r%dc%dpo%0.2fi%d',r,c,op,i),'map');
      
      %print(gcf,'my-plot','-dpng');
      %close all;
      tic
      astar(map);
      data(i,j).etime_as(it) = toc;
      close;
      %       pause(0.1);
    end
    
    % populate the mean and std
    data(i,j).jps_mean = mean(data(i,j).etime_jps);
    data(i,j).jps_std = std(data(i,j).etime_jps);
    
    data(i,j).as_mean = mean(data(i,j).etime_as);
    data(i,j).as_std = std(data(i,j).etime_as);
    
    errorbar(op-0.01,data(i,j).jps_mean,data(i,j).jps_std,'rx');
    errorbar(op+0.01,data(i,j).as_mean,data(i,j).as_std,'b.');
  end
  %set(gca, 'XTick', 1:3, 'XTickLabel', irisSpecies) % Set ticks and tick labels
  xlabel('Percetage of obstacle in map');
  ylabel('Time in seconds')
  title_str = sprintf('Map%dx%d',r,c);
  title(title_str);
  legend({'jps', 'astar'}, 'Location', 'Northeast')
  hold off
  saveas(H,[title_str '.jpg'],'jpg');
  close
end
