#include "PuzzleGenerator.h"
#include <iostream>
using namespace std;

//Constructor
PuzzleGenerator::PuzzleGenerator(int _nRows, int _nColumns, int _minVal, int _maxVal)
	:nRows(_nRows), nColumns(_nColumns), minVal(_minVal), maxVal(_maxVal)
{
	maxtime = 57.0; //set the max time value
}
/*Sample code, not used*/
/*Puzzle PuzzleGenerator::GeneratePuzzle()
{
	timer.StartTimer();
	maxtime = 59.9;	// To make sure we don't exceed a minute
	
	return RandomWalk(5);	// Do a random walk for 5 seconds and return the solution.

	// We could also do as many random walks as we can within the given time limit.
	while (timer.GetElapsedTime() + 5 < maxtime)
	{
		Puzzle p = RandomWalk(5);
		// Check if p is better than the best puzzle we have found so far.
	}
}*/

Puzzle PuzzleGenerator::GeneratePuzzle()
{
	timer.StartTimer();
	// Declare variables for Simulated Annealing Algorithm
	double initialTemp = 100;
	double finalTemp = 0.001;
	double changePerStep = 0.9999;
	

	//Try to start at a puzzle that has a solution
	Puzzle initialPuzzle = Puzzle(nRows, nColumns, minVal, maxVal);
	while(!initialPuzzle.HasSolution() && timer.GetElapsedTime() < maxtime){
		initialPuzzle = Puzzle(nRows, nColumns, minVal, maxVal);
	}

	//perform the simulated annealing algorithm
	Puzzle bestPuzzle = simulatedAnnealing(initialPuzzle, initialTemp, changePerStep, finalTemp);
	while (timer.GetElapsedTime() < maxtime){	
		initialPuzzle = Puzzle(nRows, nColumns, minVal, maxVal);
		//Try to perform the algorithm on a puzzle with solution
		while(!initialPuzzle.HasSolution() && timer.GetElapsedTime() < maxtime){
			initialPuzzle = Puzzle(nRows, nColumns, minVal, maxVal);
		}
		Puzzle p = simulatedAnnealing(initialPuzzle, initialTemp, changePerStep, finalTemp);
		if (p.GetValue() > bestPuzzle.GetValue()){
			bestPuzzle = p;
		}
	}

	return bestPuzzle;
}
//uses the exponential simulated Annealing
Puzzle PuzzleGenerator::simulatedAnnealing(Puzzle p, double initialTemp, double changePerStep, double finalTemp)
{
	//cerr << "get in SA time: " << timer.GetElapsedTime() << endl;
	Puzzle curr = p;
	int count = 0; // iteration counter (for debugging)
	double currTemp = initialTemp;

	while (timer.GetElapsedTime() < maxtime && currTemp > finalTemp)
	{
		// get random successor
		Puzzle next = curr.GetRandomSuccessor();		
		if(next.GetValue() <= curr.GetValue()){//if the next value is not better than the current value
			double expo_thres = exp((next.GetValue() - curr.GetValue())/currTemp);
			double doubleRand = (rand() % 1000) / 1000.0; // generate random value between (0,1)
			if (doubleRand < expo_thres){//jump out of the local optimum by accepting a worse solution
				curr = next;
			}
		}else{//if next value is better than the current colution
			curr = next;
		}
		// at each iteration the temperature is cooled
		currTemp = changePerStep * currTemp;//using the exponential simulated annealing
		count++;
	}
	//cerr << "get out SA time: " << timer.GetElapsedTime() << endl;
	return curr;
}


/*Sample code, not used*/
/*Puzzle PuzzleGenerator::RandomWalk(double timelimit)
{
	// A very simple function that start at a random configuration and keeps randomly modifying it
	// until it hits the time limit. Returns the best solution found so far.

	Puzzle p(nRows, nColumns, minVal, maxVal);	// Generate a random puzzle with the specified values.
	
	// Keep track of the best puzzle found so far (and its value).
	Puzzle bestPuzzle = p;			
	int bestValue = p.GetValue();
	
	// Keep track of the time so we don't exceed it.
	Timer t;
	t.StartTimer();
	
	// Loop until we hit the time limit.
	while (t.GetElapsedTime() < timelimit-0.1)	// To make sure that we don't exceed the time limit, we stop just before we hit the time limit.
	{
		// Generate a successor of p by randomly changing the value of a random cell 
		// (since we are doing a random walk, we just replace p with its successor).
		p = p.GetRandomSuccessor();	
		
		// Update the current best solution.
		if (p.GetValue() > bestValue)	// Calling GetValue() for the first time is costly
										// since the puzzle has to be evaluated first.
		{
			bestValue = p.GetValue();	// Calling it a second time simply returns the value that was computed before.
			bestPuzzle = p;
		}
	}
	
	return bestPuzzle;
	
	// The following code is not executed in this function. It exists just as an example for getting all the successors of a puzzle.
	vector<Puzzle> successors;
	bestPuzzle.GetAllSuccessors(successors);
}*/







