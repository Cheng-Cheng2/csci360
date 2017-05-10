# PuzzleGenerator
A Puzzle Generator that uses a local-solver algorithm to 'find' the most challenging and fun puzzles within 60 seconds. This project was completed as a part of CSCI 360 "Introduction to Artificial Intelligence" at the University of Southern California instructed by Sven Koenig. All project information (below) and starting code (indicated within the source files) was provided by and is property of Sven Koenig.

## Objective:
Use local search methods (such as hill-climbing and simulated annealing) to generate puzzles that are optomized to be challenging and entertaining. Initially provided code has functionality for generating and evaluating puzzles (with respect to a predefined value function detailed below). The local-solver will be evaluated based on the value of the best puzzle it can generate within 60 seconds and will be tested using a variety of different puzzle constraints (i.e. size/complexity).

## The Puzzle:
Each puzzle that is generated consists of a grid with r rows and c columns of cells. Each cell contains exactly one integer in the range from i to j (inclusive), where i and j are positive integers. The player of the puzzle has to start in the upper-left cell (start cell) and move with the smallest number of actions tothe lower-right cell (goal cell). If the player is in a cell that contains integer x, then they can perform one of at most four actions, namely either move x cells to the left (L), x cells to the right (R), x cells up (U), or x cells down (D), provided that they do not leave the grid. An example puzzle of size 5x5 is given below:

---------------------
| 3 | 2 | 1 | 4 | 1 |
---------------------
| 3 | 2 | 1 | 3 | 3 |
---------------------
| 3 | 3 | 2 | 1 | 4 |
---------------------
| 3 | 1 | 2 | 3 | 3 |
---------------------
| 1 | 4 | 4 | 3 | G |
---------------------

The shortest solution for the instance above is 19 moves: R D L R U D R L R L U D L D R R U D D.


##The Value Function for the Puzzle:
Below are some features that are expected from a 'good' puzzle that is both challenging and entertaining.
- The puzzle has a solution.
- The puzzle has a unique shortest solution.
- The puzzle contains as few "black holes" (dead ends) as possible. A "black hole" is a cell that can be reached but once there, there is no possible way to ever reach the goal.
- The puzzle contains as few "white holes" as possible. A "white hole" is a cell that is not possible to get to.
- The puzzle contains as few forced forward moves as possible. A forced forward move occurs when there is only one action that leaves a reachable cell.
- The puzzle contains as few forced backward moves as possible. A forced backward move occurs when there is only one action that reaches a reaching cell.

Using these features, we are able to quantitatively evaluate how "good" a puzzle is (i.e. how challenging and entertaining):
- Multiply the length of a puzzle by 5.
- Add (rows x columns) points if there is a unique shortest solution.
- Subtract 2 points for each white or black hole.
- Subtract 2 points for each forced forward or backward move.
- Subtract (rows x columns x 100) points if the puzzle does not have a solution.

