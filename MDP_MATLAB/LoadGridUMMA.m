function [gridMap, rawMap] = LoadGridUMMA(walkable, gridSize)

% Parse arguments
if nargin < 2
    gridSize = 10;
    if nargin < 1
        walkable = 250;
    end
end

% Load raw map and hardcode map rotation angle.
rawMap = imread('umma.pgm');
p0 = [165 2];
p1 = [322 40];
dp = p0 - p1;
angle = asin(dp(2) / dp(1)) * 180 / pi;

% Rotate to align image
rawMap = imrotate(rawMap,angle, 'bilinear', 'loose');

% Crop image
xmin = find(max(rawMap, [], 2) > walkable, 1, 'first');
xmax = find(max(rawMap, [], 2) > walkable, 1, 'last');
ymin = find(max(rawMap, [], 1) > walkable, 1, 'first');
ymax = find(max(rawMap, [], 1) > walkable, 1, 'last');

dx = xmax - xmin;
dy = ymax - ymin;

m = ceil(dx / gridSize);
n = ceil(dy / gridSize);

% Enlarge boundaries by a gridsize.
xmin_ = max(0, xmin - gridSize);
xmax_ = min(size(rawMap, 1), xmin_ + (m+2) * gridSize);
ymin_ = max(0, ymin - gridSize);
ymax_ = min(size(rawMap, 2), ymin_ + (n+2) * gridSize);

rawMap = rawMap(xmin_:xmax_, ymin_:ymax_);

% Discretize image to gird cells
mn_ = floor(size(rawMap) / gridSize);
gridMap = zeros(mn_);
for i = 1:mn_(1)
    for j = 1:mn_(2)
        xl = (i-1) * gridSize + 1;
        xu = i * gridSize;
        yl = (j-1) * gridSize + 1;
        yu = j * gridSize;
        temp = rawMap(xl:xu, yl:yu) > walkable;
        if sum(sum(temp)) == gridSize * gridSize
            gridMap(i, j) = 1;
        end
    end
end

end