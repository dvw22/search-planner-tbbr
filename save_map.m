% Transforms a png image of a map (black is unoccupied, any other colour is
% occupied) into an occupancy map and saves it.

image = imread('indoor_easy.png');  % Change to name of map image

matrix = rgb2gray(image);
matrix = matrix>0;
map = occupancyMap(matrix,2);

save('indoorEasyMap.mat','map') % Change to desired map file name
show(map)