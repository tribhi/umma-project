classdef GridWorldComplicated < GridWorld
    % GridWorldComplicated
    %   Represent a grid world with "N,E,S,W,NE,NW,SE,SW" actions.
    
    properties(Constant)
        % Possible actions taken by robot.
        actions = ["N", "E", "S", "W", "NE", "SE", "SW", "NW"];
    end

    methods(Access = public)
        function obj = GridWorldComplicated(width, height, goalCell, lossCells, ...
                obstacleCells)
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
            
            % TODO: define a resonable motion model that does not jump over
            % obstacles.
            
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
                case "NE"
                    nextSPs = FreeDiagnolAction(obj, s, 0);
                case "NW"
                    nextSPs = FreeDiagnolAction(obj, s, pi/2);
                case "SW"
                    nextSPs = FreeDiagnolAction(obj, s, pi);
                case "SE"
                    nextSPs = FreeDiagnolAction(obj, s, -pi/2);
                otherwise
                    disp("invalid action: "+a);
            end

            % Remove invalid states and normalize probabilities.
            sumProbability = 1;
            for i = length(nextSPs):-1:1
                s_ = nextSPs(i).s;
                if ~obj.IsInWorld(s_) || obj.IsObstacleCell(s_)
                    % Remove states if robot hits obstacle or goes out of 
                    % world boundary.
                    sumProbability = sumProbability - nextSPs(i).p;
                    nextSPs(i) = [];
                end
            end
            
            for i = length(nextSPs):-1:1
                nextSPs(i).p = nextSPs(i).p / sumProbability;
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
                's', obj.RotateState(s, 0, 1, angle), 'p', 0.6);
            freeForward(2) = struct(...
                's', obj.RotateState(s, 0, 2, angle), 'p', 0.05);
            freeForward(3) = struct(...
                's', obj.RotateState(s, 1, 1, angle), 'p', 0.1);
            freeForward(4) = struct(...
                's', obj.RotateState(s, -1, 1, angle), 'p', 0.1);
            freeForward(5) = struct(...
                's', obj.RotateState(s, 1, 0, angle), 'p', 0.05);
            freeForward(6) = struct(...
                's', obj.RotateState(s, -1, 0, angle), 'p', 0.05);
            freeForward(7) = struct(...
                's', obj.RotateState(s, 2, 0, angle), 'p', 0.025);
            freeForward(8) = struct(...
                's', obj.RotateState(s, -2, 0, angle), 'p', 0.025);            
        end
        
        function freeDiagnol = FreeDiagnolAction(obj, s, angle)
            % Motion model for taking diagnol action. Return struct array
            % of possible states and transition probabilities.
            % 
            % Args:
            %   s: current robot state
            %   theta: relative angle to "N" in radius, positive CCW
            
            freeDiagnol(1) = struct(...
                's', obj.RotateState(s, 1, 1, angle), 'p', 0.6);
            freeDiagnol(2) = struct(...
                's', obj.RotateState(s, 0, 1, angle), 'p', 0.15);
            freeDiagnol(3) = struct(...
                's', obj.RotateState(s, 1, 0, angle), 'p', 0.15);
            freeDiagnol(4) = struct(...
                's', obj.RotateState(s, 0, 2, angle), 'p', 0.05);
            freeDiagnol(5) = struct(...
                's', obj.RotateState(s, 2, 0, angle), 'p', 0.05);
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
                case "NW"
                    q = quiver(x-1+ArrowLength, y-ArrowLength, -ArrowLength, ArrowLength);
                case "NE"
                    q = quiver(x-ArrowLength, y-ArrowLength, ArrowLength, ArrowLength);
                case "SW"
                    q = quiver(x-1+ArrowLength, y-1+ArrowLength, -ArrowLength, -ArrowLength);
                case "SE"
                    q = quiver(x-ArrowLength, y-1+ArrowLength, ArrowLength, -ArrowLength);
                otherwise
                    return
            end
            q.MaxHeadSize = 5;
            q.Color = [0.3010, 0.7450, 0.9330];
        end
    end
end