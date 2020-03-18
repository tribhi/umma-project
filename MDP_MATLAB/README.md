# MDP-Grid

MDP-Grid solves grid world planning problems using Markov Decision Process with 'value iteration' or 'policy iteration' algorithm. 


![Basic working](https://bitbucket.org/guanksu/mdp-grid/raw/b68efa5def86470c95e10fd96930bc87dde7ec61/example%20results/default%20grid%20world/policy%20iteration.png)

---

## Description of files
- demo.m
Demo of creating costumed grid map and sovle it with MDP.

- demo_umma.m
Demo of load and discretize UMMA map into grid map and solve it with MDP.

- LoadGridUMMA.m
Function to load umma.pgm map and discretize it to grid map.

- GridWorld.m
Base class to represent a grid world environment.

- GridWorldSimple.m
Subclass of GridWorld with a motion model composed of ["N", "E", "W", "S"] actions.

- GridWorldComplicated.m
Subclass of GridWorld with a motion model composed of ["N", "E", "W", "S", "NE", "NW", "SE", "SW"] actions.

- MDP.m
Class of MDP solver with both 'value iteration' and 'policy iteration' algorithm. 

- umma.pgm
UMMA map.

## How to run

1. Run demo.m in Matlab. You will be prompted to select default or custom grid world. Then plots will be generated.

2. Run demo_umma.m in Matlab. You will be prompted to goal position and number of loss cells (something you don't want the robot to touch) in the discretized grid map of UMMA. Then plots will be generated.

Example results of default grid world and custom grid world are stored under `$REPO_ROOT_DIRECTORY/example results/`:

---

## References

Berkeley AI CS 188 Spring 2014: Lec 8 MDPs and Lec 9 MDPs II [video](https://www.youtube.com/playlist?list=PLIZQvCoJVokgKBNx210mkXEk4FeSOGfWu) and [slides](http://ai.berkeley.edu/lecture_slides.html)
