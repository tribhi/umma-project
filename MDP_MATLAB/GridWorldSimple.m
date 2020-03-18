classdef GridWorldSimple < GridWorld
    % GridWorldSimple
    %   Represent a grid world with "NESW" actions.
    
    properties(Constant)
        % Possible actions taken by robot.
        actions = ["N", "E", "S", "W"];
    end

    methods(Access = public)
        function obj = GridWorldSimple(width, height, goalCell, lossCells, ...
                obstacleCells, cellWidth, cellHeight)
            % Parse input arguments.
            % Create default grid world if input arguments are not enough.
            if nargin < 7
                cellWidth = 140;
                cellHeight = 140;
            end

            if nargin < 5
                width = 4;
                height = 3;
                goalCell = struct('x', 4, 'y', 3, 'reward', 1);
                lossCells = struct('x', 4, 'y', 2, 'reward', -1);
                obstacleCells = struct('x', 2, 'y', 2, 'reward', nan);
            end
            
            % Constructor.
            obj@GridWorld(width, height, goalCell, lossCells, ...
                obstacleCells, cellWidth, cellHeight);      
        end
        
        function reward = Reward(obj, s, a, s_)
            % Return reward after robot take action a from current state
            % s and arrives at state s_. Return zero if s is out of the
            % world or in obstacle.
            %
            % Args:
            %   s: current robot state (can be out of the world or in obstacle)
            %   a: current robot action (not used)
            %   s_: next robot state (not used)

            if obj.IsInWorld(s) && ~obj.IsObstacleCell(s)
                reward = obj.rewards(s.x, s.y);
            else
                reward = 0;
            end
        end
        
        function nextSPs = MotionModel(obj, s, a)
            % Return struct array of possible next states and corresponding
            % trainsition probabilities.
            %
            % Args:
            %   s: robot state
            %   a: robot action
            
            % Possible next states and trainsition probabilities in a world 
            % free of boundary and obstacles.
            switch a
                case "N"
                    nextSPs = FreeForwardAction(obj, s, 0);
                case "W"
                    nextSPs = FreeForwardAction(obj, s, pi/2);
                case "S"
                    nextSPs = FreeForwardAction(obj, s, pi);
                case "E"
                    nextSPs = FreeForwardAction(obj, s, -pi/2);
                otherwise
                    disp("invalid action: "+a);
            end

            % Stay current states if robot hits obstacle or goes out of 
            % world boundary.
            for i = length(nextSPs):-1:1
                s_ = nextSPs(i).s;
                if ~obj.IsInWorld(s_) || obj.IsObstacleCell(s_)
                    nextSPs(i).s = s;
                end
            end
        end
    end
    
    methods(Access = private)
        function freeForward = FreeForwardAction(obj, s, angle)
            % Motion model for taking forward action. Return struct array
            % of possible states and transition probabilities.
            % 
            % Args:
            %   s: current robot state
            %   angle: relative angle to "N" in radius, positive CCW

            freeForward(1) = struct(...
                's', obj.RotateState(s, 0, 1, angle), 'p', 0.8);
            freeForward(2) = struct(...
                's', obj.RotateState(s, -1, 0, angle), 'p', 0.1);
            freeForward(3) = struct(...
                's', obj.RotateState(s, 1, 0, angle), 'p', 0.1);            
        end     
    end
    
	methods(Access=protected)
        function RenderCell(obj, x, y, faceColor, annotation, policy)
            % Description:
            %   Paint, annotate and draw corresponding action of a cell.
            % Args:
            %   policy: policy at this cell

            % Call base method to render and annotate cell.
            RenderCell@GridWorld(obj, x, y, faceColor, annotation);

            % Render action at current cell.
            ArrowLength = 0.2;
            switch policy
                case "N"
                    q = quiver(x-0.5, y-ArrowLength, 0, ArrowLength);
                case "E"
                    q = quiver(x-ArrowLength, y-0.5, ArrowLength, 0);
                case "S"
                    q = quiver(x-0.5, y-1+ArrowLength, 0, -ArrowLength);
                case "W"
                    q = quiver(x-1+ArrowLength, y-0.5, -ArrowLength, 0);
                otherwise
                    return
            end
            q.MaxHeadSize = 5;
            q.Color = [0.3010, 0.7450, 0.9330];
        end
    end
end