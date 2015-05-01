function run
clear all; close all;
run_str='-2';
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


size_of_map = size(map_size,2);
size_of_op = size(obstacle_percentage,2);

for i = 1:size_of_map
  H = figure; hold on;
  r = map_size(i).r;
  c = map_size(i).c;
  for j = 1:size_of_op
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

for i = 1:size_of_map
  for j = 1:size_of_op
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

save all;

%%
% map size 1,ob(1,2,3)    map size 2,ob(1,2,3)
%color_op = ['bx'; 'gx'; 'rx'; 'cx'; 'mx'; 'kx'; 'ko'];
color_op = ['k.'; 'ko'; 'kx'; 'k+'; 'ks'; 'kv'; 'kh'];
x = obstacle_percentage;
H = figure; hold on;
for i = 1:size_of_map
  for j = 1:size_of_op
    errorbar( i + x(j), mean_dist(i,j),std_dist(i,j),color_op(j,:));
  end
end
xlabel('Percetage of obstacle in map');
labels = {'10x10' '15x15' '20x20' '25x25' '30x30'};
set(gca, 'XTick', 1.35:1:size_of_map+0.35, 'XTickLabel', labels);
ylabel('Euclidean Distance')
title_str = sprintf('Dist%dx%d',r,c);
title(title_str);
legend('a','b');
hold off
saveas(H,[title_str run_str '.jpg'],'jpg');
close


%%
x = 1:size_of_op;
H = figure; hold on;
bar(x,mean(jps_mean_nne),0.2,'r');
bar(x+0.25,mean(as_mean_nne),0.2,'b');
xlabel('Percetage of obstacle in all maps');
labels = {'0.1' '0.2' '0.3' '0.4' '0.5' '0.6' '0.7'};
set(gca, 'XTick', 1.25:1:size_of_op+0.25, 'XTickLabel', labels);
ylabel('Num of Nodes Evaluated')
title_str = sprintf('Average Number of Nodes Expanded');
title(title_str);
legend('jump point search','A*');
hold off
saveas(H,[title_str run_str '.jpg'],'jpg');
close

%%
x = 1:size_of_op;
H = figure; hold on;
bar(x,min(jps_mean_nne),0.2,'r');
bar(x+0.25,min(as_mean_nne),0.2,'b');
xlabel('Percetage of obstacle in all maps');
labels = {'0.1' '0.2' '0.3' '0.4' '0.5' '0.6' '0.7'};
set(gca, 'XTick', 1.25:1:size_of_op+0.25, 'XTickLabel', labels);
ylabel('Num of Nodes Evaluated')
title_str = sprintf('Best Case Number of Nodes Expanded');
title(title_str);
legend('jump point search','A*');
hold off
saveas(H,[title_str run_str '.jpg'],'jpg');
close

%%
x = 1:size_of_op;
H = figure; hold on;
bar(x,max(jps_mean_nne),0.2,'r');
bar(x+0.25,max(as_mean_nne),0.2,'b');
xlabel('Percetage of obstacle in all maps');
labels = {'0.1' '0.2' '0.3' '0.4' '0.5' '0.6' '0.7'};
set(gca, 'XTick', 1.25:1:size_of_op+0.25, 'XTickLabel', labels);
ylabel('Num of Nodes Evaluated')
title_str = sprintf('Worst Case Number of Nodes Expanded');
title(title_str);
legend('jump point search','A*');
hold off
saveas(H,[title_str run_str '.jpg'],'jpg');
close
