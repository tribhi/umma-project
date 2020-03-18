classdef MDP
    % MDP
    %   Markov Decision Process solver. 
    %   'value iteration' and 'policy iteration' algorithms are implemented
    %   in this class. Default algorithm is 'value iteration'.

    properties
        % world - World that defines a planning problem to solve with
        world
        % iterations - Current iteration
        iterations = 0
    end
    
    properties(Access = private)
        % algorithm - Can only be 'value iteration' or 'policy iteration'
        algorithm = 'value iteration'
        % gamma - Discounting factor for calculating qValues.
        gamma = 0.9
        % tolerance - Convergance tolerance.
        tolerance = 0.01
        % maxQvalueDiff - Max qValue difference between iterations.
        maxQvalueDiff = inf
        % policiesDiff - Policies difference between iterations.
        policiesDiff = inf
    end
    
	methods(Access = public)
        function obj = MDP(world, algorithm, gamma, tolerance)
            % Constructor.
            %
            % Args:
            %   world: world with planning algorithm to solve with
            %   algorithm: algorithm to use
            %   gamma: discounting factor
            %   tolerance: tolerance for value iteration
            
            obj.world = world;
            obj.algorithm = algorithm;
            if nargin >= 3
                obj.gamma = gamma;
                if nargin >= 4
                    obj.tolerance = tolerance;
                end
            end
        end
        
        function obj = Update(obj)
            % Perform one iteration update.
            %
            % One iteration update in either value iteration or policy
            % iteration algorithm. Update the world property.

            obj.iterations = obj.iterations + 1;
            if strcmp(obj.algorithm, 'value iteration')
                obj = obj.UpdateValueIteration();
            elseif strcmp(obj.algorithm, 'policy iteration')
                obj = obj.UpdatePolicyIteration();
            end
        end
        
        function converged = IsConverged(obj)
            % Whether algorithm has converged.
            %
            % Return true if MDP algorithm has converged and false
            % otherwise.

            if strcmp(obj.algorithm, 'value iteration')
                if obj.maxQvalueDiff < obj.tolerance
                    converged = true;
                else
                    converged = false;
                end
            elseif strcmp(obj.algorithm, 'policy iteration')
                if obj.policiesDiff == 0
                    converged = true;
                else
                    converged = false;
                end
            end
        end
    end
    
    methods(Access = private)
        function skip = ShouldSkip(obj, s)
            % Description:
            %   Whether to skip state s.
            % Args:
            %   s: robot state.

            % Skip goal cell, loss cells, obstacle cells.
            skip = obj.world.IsLossCell(s) | obj.world.IsGoalCell(s) | ...
                obj.world.IsObstacleCell(s);
        end

        function qvalue = Qvalue(obj, s, a)
            % Description:
            %   Return Qvalue at state s after taking action a using
            %   Bellman equation.
            % Args:
            %   s: robot state
            %   a: robot action

            % Possible next states, along with transition probability.
            sp_s = obj.world.MotionModel(s, a);
            % Accumulate for each possible next state.
            qvalue = 0;
            for sp_ = sp_s
                s_ = sp_.s;
                p_ = sp_.p;
                qvalue = qvalue + p_ * (obj.world.Reward(s, a, s_) + ...
                    obj.gamma * obj.world.qValues(s_.x, s_.y));
            end
        end

        function obj = UpdateValueIteration(obj)
            % Description:
            %   One iteration update of value iteration algorithm.

            [width, length] = obj.world.Shape();
            % Create temp qValues for update.
            qValues_ = obj.world.rewards;
            for i = 1:width
                for j = 1:length
                    s = struct('x', i, 'y', j);
                    if obj.ShouldSkip(s)
                        continue
                    end
                    % Keep track of action that results in max Qvalue.
                    bestAction = obj.world.policies(i, j);
                    for a = obj.world.actions
                        qValue_ = obj.Qvalue(s, a);
                        if qValue_ > qValues_(i, j)
                            qValues_(i, j) = qValue_;
                            bestAction = a;
                        end
                    end
                    obj.world.policies(i, j) = bestAction;
                end
            end
            % Update max qValue difference to check for convergence.
            obj.maxQvalueDiff = max(max(abs(obj.world.qValues-qValues_)));
            % Update qValues.
            obj.world.horizon = obj.world.horizon + 1;
            obj.world.qValues = qValues_;
        end

        function obj = UpdatePolicyIteration(obj)
            % Description:
            %   One iteration update of policy iteration algorithm.

            % Policy evaluation.
            [width, length] = obj.world.Shape();
            % Reset qValues and horizon at the start of policy evaluation.
            obj.world.horizon = 0;
            obj.world.qValues = obj.world.rewards;
            % Iterate until Qvalues converge.
            obj.maxQvalueDiff = inf;
            while obj.maxQvalueDiff > obj.tolerance
                Qvalues_ = obj.world.rewards;
                for i = 1:width
                    for j = 1:length
                        if obj.ShouldSkip(struct('x', i, 'y', j))
                            continue
                        end
                        action = obj.world.policies(i, j);
                        Qvalues_(i, j) = obj.Qvalue( ...
                            struct('x', i, 'y', j), action);
                    end
                end
                % Update qValues and max qValue difference between
                % iterations to check convergence.
                obj.maxQvalueDiff = max(max(abs(obj.world.qValues-Qvalues_)));
                obj.world.qValues = Qvalues_;
                obj.world.horizon = obj.world.horizon + 1;
            end

            % Policy improvement.
            OldPolicies = obj.world.policies;
            for i = 1:width
                for j = 1:length
                    if obj.ShouldSkip(struct('x', i, 'y', j))
                        continue
                    end
                    % Set policy to action that gives max qValue with
                    % one-step look aheand.
                    qValue = 0;
                    for a = obj.world.actions
                        qValue_ = obj.Qvalue(struct('x', i, 'y', j), a);
                        if qValue_ > qValue
                            obj.world.policies(i, j) = char(a);
                            qValue = qValue_;
                        end
                    end
                end
            end
            obj.policiesDiff = sum(sum(abs(char(obj.world.policies) - char(OldPolicies))));
        end        
    end
    
    methods(Access = private, Static = true)
        function idx = IndexOfPlannedAction(action)
            % Description:
            %   Map planned actions to an integer. Return [0, 1, 2, 3] for
            %   ['N', 'E', 'S', 'W] and -1 for unknown actions.
            % Args:
            %   action: robot action

            if action == 'N'
                idx = 0;
            elseif action == 'E'
                idx = 1;
            elseif action == 'S'
                idx = 2;
            elseif action == 'W'
                idx = 3;
            else
                idx = -1;
            end
        end

        function idx = IndexOfActuatedAction(s, s_)
            % Description:
            %   Same as above except that using actuated action instaed of
            %   planned action.
            % Args:
            %   s: current robot state
            %   s_: next robot state

            dx = s_.x - s.x;
            dy = s_.y - s.y;
            if dx == 0 && dy == 1 % 'N'
                idx = 0;
            elseif dx == 1 && dy == 0 % 'E'
                idx = 1;
            elseif dx == 0 && dy == -1 % 'S'
                idx = 2;
            elseif dx == -1 && dy == 0 % 'W'
                idx = 3;
            else
                idx = -1;
            end
        end
    end
end