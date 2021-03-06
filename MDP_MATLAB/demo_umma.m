%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo of running mdp on umma map. First, discretize the umma map into
% grid. Then random number of obstacles will be placed in the walkable
% area. The goal position is then selected. Fianlly will run mdp on this
% map.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all

% Width and height of displayed cell.
displayCellWidth = 70;
displayCellHeight = 70;

%% Init
% Load grid map
walkable = 250;
gridStepSize = 50;
[gridMap, rawMap] = LoadGridUMMA(walkable, gridStepSize);

% Select goal position from grid map.
[goalX, goalY] = PlotAndSelect(gridMap);
nLoss = input('How many loss cells?\n');

% Construct grid world environment and randomly place obstacles in walkable
% area.
gridSize = size(gridMap);
walkableIndexes = find(gridMap > 0);
obstacleIndexes = find(gridMap <= 0);
lossIndexes = datasample(walkableIndexes, nLoss, 'Replace', false);

GoalCell = struct('x', goalX, 'y', goalY, 'reward', 1);
LossCells = ConstructCells(lossIndexes, gridSize, -1);
ObstacleCells = ConstructCells(obstacleIndexes, gridSize, nan);
world = GridWorldSimple(gridSize(1), gridSize(2), GoalCell, LossCells, ...
    ObstacleCells, displayCellWidth, displayCellHeight);

% Construct mdp solver.
algorithm = 'value iteration';
gamma = 0.9;  % discount factor
tolerance = 0.01;  % tolerance for value iteration convergence
mdp = MDP(world, algorithm, gamma, tolerance);

%% Run MDP solver
% Display initial Grid World.
fig = figure(1);
mdp.world.Display(fig, 'Initial world')
pause(0.5);

% Run MDP algorithm.
MAX_ITERATIONS = 100;
while ~mdp.IsConverged() && mdp.iterations < MAX_ITERATIONS
    mdp = mdp.Update();
    titleStr = sprintf('current %s: %d', algorithm, mdp.iterations);
    mdp.world.Display(fig, titleStr);
    pause(0.1);
end
% Rotate plot so that its coordinate is aligned with raw UMMA map.
camroll(-90);

% Display raw map and discretized grid map
figure(2)
imshow(rawMap)
xlabel('y')
ylabel('x')
figure(3)
heatmap(gridMap);
xlabel('y')
ylabel('x')

%% Helpers
function cells = ConstructCells(indexes, gridSize, reward)
    % Construct cells.
    %
    % Args:
    %   indexes: linear indexes of cells
    %   gridSize: size of grid world
    %   reward: reward of these cells
    
    for i = length(indexes):-1:1
        [x, y] = ind2sub(gridSize, indexes(i));
        cells(i) = struct('x', x, 'y', y, 'reward', reward);
    end
end

function [x, y] = PlotAndSelect(map)
    % Plot heatmap of input map and promopt user to select point in map.
    fig = figure();
    heatmap(map);
    xlabel('y')
    ylabel('x')
    x = input('Choose x position from map:\n');
    y = input('Choose y position from map:\n');
    close(fig)
end