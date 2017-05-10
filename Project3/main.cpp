#include <iostream>

#include "PartiallyKnownGrid.h"
#include "GridPathPlanner.h"
#include <vector>
using namespace std;

void Simulate(PartiallyKnownGrid* grid)
{
	// Use "GridPathPlanner planner(grid, true)" to test your Adaptive A* implementation.
	//original - GridPathPlanner planner(grid);
    GridPathPlanner planner(grid, true);
	// Start simulation
	int steps = 0;
    int waitCounter = 100; // amount to wait between steps (milliseconds)
	grid->Reset();
    grid->DrawGrid();

    int count = 0;//debug
    vector<int> expansions;
    while (!grid->GoalReached()) {  // loop until your robot find the target or dies
        count++; //debug;
        //if(count < 2){//debug
        xyLoc move_to = planner.GetNextMove(grid);
		//debug
        expansions.push_back(planner.GetNumExpansions());

        // Call the simulator to move your robot and count the steps
        bool valid_move = grid->MoveTo(move_to);
        if (!valid_move) {
        	cout<<"Stopping simulation due to invalid move..."<<endl;
        	return;
        }
		steps++;
        grid->DrawGrid();

        #if defined(_WIN32) || defined(_WIN64)
        Sleep(waitCounter);
        #else
        //*
        struct timespec req, rem;
        req.tv_nsec = 1000000*waitCounter;
        req.tv_sec = 0;
        nanosleep(&req, &rem);
        /*/
		usleep(1000*waitCounter);
		//*/
        #endif
    //}else{
    //    break; //debug
    //}//debug
    }
	cout<<"Target found in "<<steps<<" steps !!!"<<endl;
    //debug
    int d = 0;
    for(int i=0; i<expansions.size(); i++){
        d+= expansions[i];
    }
    int average = d/double(steps);
    cout << "total number of expansions: " <<d<< endl;
    cout << "average number of expansions: " <<average << endl;
    cout << "first search: " <<expansions[0]<< endl;
    cout << "2nd search: " <<expansions[1]<< endl;
}

int main() {
	PartiallyKnownGrid grid("map");
	Simulate(&grid);

	return 0;
}
