%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo of running mdp on grid map. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create figure for display.
fig = figure();

% Ask user to choose between default or custom grid world.
x = input('Choose default or custome grid world? 0: default or 1: custom \n');

% Construct MDP grid world solver.
algorithm = 'value iteration';
gamma = 0.9;  % discount factor
tolerance = 0.01;  % tolerance for value iteration convergence
if x == 0
    world = GridWorldComplicated();
%     world = GridWorldSimple();
    mdp = MDP(world, algorithm, gamma, tolerance);
elseif x == 1
    % Construct custome grid world. Follow this example to create your own
    % grid world.
    width = 8;
    length = 6;
    GoalCell = struct('x', 8, 'y', 6, 'reward', 1);
    LossCells(1) = struct('x', 4, 'y', 3, 'reward', -1);
    LossCells(2) = struct('x', 6, 'y', 4, 'reward', -1);
    ObstacleCells(1) = struct('x', 2, 'y', 2, 'reward', nan);
    ObstacleCells(2) = struct('x', 4, 'y', 2, 'reward', nan);
    world = GridWorldComplicated(width, length, GoalCell, LossCells, ObstacleCells);
%     world = GridWorldSimple(width, length, GoalCell, LossCells, ObstacleCells);
    mdp = MDP(world, algorithm, gamma, tolerance);
else
    disp('Choice of grid world does not exist');
end

% Display initial Grid World.
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
