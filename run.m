function run
clear all; close all;
run_str='-2';
ITERATIONS = 2;

map_size(1).r = 10;
map_size(1).c = 10;

map_size(2).r = 15;
map_size(2).c = 15;

map_size(3).r = 7;
map_size(3).c = 7;

map_size(4).r = 6;
map_size(4).c = 6;

map_size(5).r = 5;
map_size(5).c = 5;

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
      
      [map,data(i,j).dist(it)] = random_gen_map(r,c,op);
      
      tic
      data(i,j).jps_nne(it) = jumppointsearch(map);
      data(i,j).etime_jps(it) = toc;
      close;
      
      %saveas(gcf,sprintf('r%dc%dpo%0.2fi%d.jpg',r,c,op,i),'jpg');
      %save(sprintf('map-r%dc%dpo%0.2fi%d',r,c,op,i),'map');
      
      %print(gcf,'my-plot','-dpng');
      %close all;
      tic
      data(i,j).as_nne(it) = astar(map);
      data(i,j).etime_as(it) = toc;
      close;
      %       pause(0.1);
    end
    
    % populate the mean and std
    data(i,j).jps_mean = mean(data(i,j).etime_jps);
    data(i,j).jps_std = std(data(i,j).etime_jps);
    
    data(i,j).as_mean = mean(data(i,j).etime_as);
    data(i,j).as_std = std(data(i,j).etime_as);
    
    data(i,j).jps_mean_nne = mean(data(i,j).jps_nne);
    data(i,j).as_mean_nne = mean(data(i,j).as_nne);
    data(i,j).jps_std_nne = std(data(i,j).jps_nne);
    data(i,j).as_std_nne = std(data(i,j).as_nne);
    
    data(i,j).mean_dist = mean(data(i,j).dist);
    data(i,j).std_dist = std(data(i,j).dist);
    
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
  %saveas(H,[title_str run_str '.jpg'],'jpg');
  close
end

for i = 1:size(map_size,2)
  for j = 1:size(obstacle_percentage,2)
    jps_mean(i,j) = data(i,j).jps_mean;
    jps_std(i,j) = data(i,j).jps_std;
    as_mean(i,j) = data(i,j).as_mean;
    as_std(i,j) = data(i,j).as_std;
    jps_mean_nne(i,j) = data(i,j).jps_mean_nne;
    jps_std_nne(i,j) = data(i,j).jps_std_nne;
    as_mean_nne(i,j) = data(i,j).as_mean_nne;
    as_std_nne(i,j) = data(i,j).as_std_nne;
    mean_dist(i,j) = data(i,j).mean_dist;
    std_dist(i,j) = data(i,j).std_dist;
  end
end

x = obstacle_percentage;
H = figure; hold on;
for i = 1:size(map_size,2)
  for j = 1:size(obstacle_percentage,2)
    errorbar(x,mean_dist(i,j),std_dist(i,j),'rx');
  end
end
xlabel('Percetage of obstacle in map');
ylabel('Euclidean Distance')
title_str = sprintf('Dist%dx%d',r,c);
title(title_str);
hold off
saveas(H,[title_str run_str '.jpg'],'jpg');
close

H = figure; hold on;
for i = 1:size(map_size,2)
  for j = 1:size(obstacle_percentage,2)
    errorbar(x-0.01,jps_mean_nne(i,j),jps_std_nne(i,j),'rx');
    errorbar(x+0.01,as_mean_nne(i,j),as_std_nne(i,j),'b.');
  end
end
xlabel('Percetage of obstacle in map');
ylabel('Num of Nodes Evaluated')
title_str = sprintf('Nne%dx%d',r,c);
title(title_str);
hold off
saveas(H,[title_str run_str '.jpg'],'jpg');
close

save all;
