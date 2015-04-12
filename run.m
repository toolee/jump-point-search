function run
for i = 1:100
  clear map; close all;
  r = 5;
  c = 5;
  po = 0.40;
  map = random_gen_map(r,c,po);
  tic
  jumppointsearch(map);
  toc
  saveas(gcf,sprintf('r%dc%dpo%0.2fi%d.jpg',r,c,po,i),'jpg');
  save(sprintf('map-r%dc%dpo%0.2fi%d',r,c,po,i),'map');
  %print(gcf,'my-plot','-dpng');
  %close all;
  %tic
  %astar(map);
  %toc
  %pause(0.1);
end