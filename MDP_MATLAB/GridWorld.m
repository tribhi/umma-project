classdef GridWorld
    % GridWorld
    %   Representation of a grid world. The grid world is represented as a 
    %   2D grid. A scratch is shown below,
    %
    %        ----------------------
    %        |  |  |  |  |  |  |  |    j (y)
    % height |--------------------|    ^
    %        |  |  |  |  |  |  |  |    |
    %        ----------------------    |-->   i (x)
    %                width
    %   Inside the grid, there could be goal cell, obstacle cells or loss 
    %   cells. And each cell contains an instant reward, Q value and
    %   policy. The states of the world a set of all cells in this world
    %   and it is represented as a struct of (i, j), where i, j are
    %   coordinates of each cell.
    
    properties(Access = public)
        % rewards - A 2D array of instant rewards of each state.
        rewards
        % qValues - A 2D array of qValues (Utilities) of each state.
        qValues
        % horizon - Horizon for calculating Qvalue.
        horizon
        % policies - Policies of each state.
        policies
    end
    
    properties(Access = protected)
        % width - Width of the world.
        width
        % height - Height of the world.
        height
        % goalCell - Stored goal cell.
        goalCell
        % lossCells - Stored loss cells.
        lossCells
        % obstacleCells - Stored obstacle cells.
        obstacleCells
        % cellWidth - Display width of cell
        cellWidth
        % cellHeight - Display height of cell
        cellHeight
    end
    
    properties(Abstract, Constant)
        % Possible actions taken by robot.
        actions
    end

    methods
        function obj = GridWorld(width, height, goalCell, lossCells, ...
                obstacleCells, cellWidth, cellHeight)
            % Constructor.
            %
            % Args:
            %   width: max width of grid world
            %   height: max height of grid world
            %   goalCell: struct of goal cell (x, y, reward)
            %   lossCells: struct array of loss cells (x, y, reward)
            %   obstacleCells: struct array of obstacle cells
            %       (x, y, reward)
            %   cellWidth: display width of cell
            %   cellHeight: display height of cell
            
            % Convert (x, y) indexes of special cells to linear indices.
            shape = [width, height];
            goalCellLinearIndices = ...
                sub2ind(shape, [goalCell.x], [goalCell.y]);
            lossCellsLinearIndices = ...
                sub2ind(shape, [lossCells.x], [lossCells.y]);
            obstacleCellsLinearIndices = ...
                sub2ind(shape, [obstacleCells.x], [obstacleCells.y]);
            
            % Init instant rewards of all cells, default to zero.
            obj.rewards = zeros(shape);
            obj.rewards(goalCellLinearIndices) = [goalCell.reward];
            obj.rewards(lossCellsLinearIndices) = [lossCells.reward];
            obj.rewards(obstacleCellsLinearIndices) = [obstacleCells.reward];

            % Init qValues to instant rewards at each state and set Qvalue
            % horizon to 1.
            obj.qValues = obj.rewards;
            obj.horizon = 1;

            % Generate ramdom policy.
            obj.policies = obj.actions(randi(numel(obj.actions), shape));
            obj.policies(goalCellLinearIndices) = '-';
            obj.policies(lossCellsLinearIndices) = '-';
            obj.policies(obstacleCellsLinearIndices) = '-';

            % Store goal cell, loss cells and obstacle cells.
            obj.width = width;
            obj.height = height;
            obj.goalCell = goalCell;
            obj.lossCells = lossCells;
            obj.obstacleCells = obstacleCells;        
            obj.cellWidth = cellWidth;
            obj.cellHeight = cellHeight;
        end
        
        function Display(obj, fig, titleStr)
            % Show the grid world.
            %
            % Args:
            %   fig: Figure Object to display
            %   titleStr: String to be displayed as figure title

            % Set figure properties.
            fig.Position = [100, 100, obj.cellWidth * obj.width, obj.cellHeight * obj.height];
            fig.Name = 'Grid World';
            fig.Color = 'black';
            % Set grid properties.
            set(gca, 'Layer', 'top') % set grid to be on top of others.
            set(gca, 'GridColor', 'black')
            % Set ticks properties.
            set(gca, 'XTick', 1:obj.width, 'XColor', 'w')
            set(gca, 'YTick', 1:obj.height, 'YColor', 'w')
            % Set other properties.
            axis([0, obj.width, 0, obj.height])
            grid on;
            hold on;            
        
            % Render all cells.
            maxQvalue = max(max(obj.qValues));
            minQvalue = min(min(obj.qValues));
        
            for i = 1:obj.width
                for j = 1:obj.height
                    qValue = obj.qValues(i, j);
                    action = obj.policies(i, j);
                    s = struct('x', i, 'y', j);
                    if isnan(qValue) % Obstacle cells.
                        obj.RenderCell(i, j, [0.5, 0.5, 0.5], '', ...
                            action);
                    elseif qValue > 0 % Cell with positive Qvalue
                        if obj.IsGoalCell(s) % Goal cell.
                            annotation = sprintf('G: %.2f', qValue);
                        else
                            annotation = sprintf('%.2f', qValue);
                        end
                        obj.RenderCell(i, j, [0, qValue / maxQvalue, 0], ...
                            annotation, action);
                    else % Cell with positive Qvalue
                        if obj.IsLossCell(s) % Loss cells.
                            annotation = sprintf('L: %.2f', qValue);
                        else
                            annotation = sprintf('%.2f', qValue);
                        end
                        obj.RenderCell(i, j, [qValue / minQvalue, 0, 0], ...
                            annotation, action);
                    end
                end
            end
            
            % Add title and leave some space between axis.
            t = title({titleStr, ' ', ' '});
            t.Color = 'white';
            
            % x, y labels
            xlabel('x', 'Color', 'w')
            ylabel('y', 'Color', 'w')
            
            % Retain background color.
            set(gcf, 'InvertHardCopy', 'off');
        end

        function [width, height] = Shape(obj)
            % Return shape of the world.
            width = obj.width;
            height = obj.height;
        end
        
        function isGoalCell = IsGoalCell(obj, s)
            % Whether state (x, y) is the goal cell.
            %
            % Args:
            %   s: state
            
            isGoalCell = obj.IsInCells(size(obj.qValues), ...
                obj.goalCell, s.x, s.y);
        end

        function isLossCell = IsLossCell(obj, s)
            % Whether state (x, y) is any of the loss cells.
            %
            % Args:
            %   s: state

            isLossCell = obj.IsInCells(size(obj.qValues), ...
                obj.lossCells, s.x, s.y);
        end

        function isObstacleCell = IsObstacleCell(obj, s)
            % Whether (x, y) is any of the obstacle cells.
            % Args:
            %   s: state

            isObstacleCell = obj.IsInCells(size(obj.qValues), ...
                obj.obstacleCells, s.x, s.y);
        end
             
        function isIn = IsInWorld(obj, s)
            % Whether state is inside world boundary.
            isIn = 1 <= s.x && s.x <= obj.width ...
                && 1 <= s.y && s.y <= obj.height;
        end        
    end
    
    methods(Abstract)
        reward = Reward(obj, s, a, s_)
        % Return reward after robot take action a from current state
        % s and arrives at state s_. Return zero if s is out of the
        % world or in obstacle.
        %
        % Args:
        %   s: current robot state (can be out of the world or in obstacle)
        %   a: current robot action (not used)
        %   s_: next robot state (not used)

        nextSPs = MotionModel(obj, s, a)
        % Return struct array of possible next states and corresponding
        % trainsition probabilities.
        %
        % Args:
        %   s: robot state
        %   a: robot action
    end

	methods(Access=protected)
        function s_ = RotateState(~, s, dx, dy, angle)
            % Rotate (s.x+dx, s.y+dy) by angle and return as struct.
            %
            % Args:
            %   s: state (x, y)
            %   dx: desired step in x direction
            %   dy: desired step in y direction
            %   angle: ratation angle in radius
            
            ct = cos(angle);
            st = sin(angle);
            x_ = s.x + ct*dx - st*dy;
            y_ = s.y + st*dx + ct*dy;
            s_ = struct('x', int32(x_), 'y', int32(y_));
        end
        
        function isInCells = IsInCells(~, gridSize, targetCells, x, y)
            % Description:
            %   Whether (x, y) is any of the target cells.
            % Args:
            %   gridSize: size of the grid world.
            %   targetCells: target cells.
            %   x: X coordiante.
            %   y: Y coordiante.

            % Convert indexes to linear indices.
            targetCellIndexes = ...
                sub2ind(gridSize, [targetCells.x], [targetCells.y]);
            myCellIndex = sub2ind(gridSize, x, y);

            isInCells = ~isempty(find(targetCellIndexes == myCellIndex, 1));
        end
        
        function RenderCell(~, x, y, faceColor, annotation)
            % Description:
            %   Paint, annotate and draw corresponding action of a cell.
            % Args:
            %   x: X Coordinate of this cell
            %   y: Y Coordinate of this cell
            %   faceColor: color of this cell
            %   annotation: text annotation of this cell

            % Render a rectangle for current cell and add annotation.
            r = rectangle();
            r.Position = [x - 1, y - 1, 1, 1];
            r.FaceColor = faceColor;
            r.EdgeColor = 'w';
            t = text(x-0.5, y-.5, annotation);
            t.HorizontalAlignment = 'center';
            t.Color = 'white';
        end
    end
end