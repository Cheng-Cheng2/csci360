#include "GridPathPlanner.h"
#include <stdlib.h> 
#include <stdexcept>
#include <iostream>

/* 
CSCI 360 project 3
-This file implemeted the repeated forward A* algorithm and Adaptive A* Algorithm
utilized by a GridPathPlanenr for navigation through partially unknow environment;

-if the the boolean value use_adaptive_a_star passes through contructor is true, 
adaptive A* is utilized. Otherwise, repeated forward A* is utilized.

-Each time when the GetNextMove function is called from the main function(provided), it returns
the next best move calculated by repeated/adaptive A* algorithm with updated information
about the environment

*/
using namespace std;
GridPathPlanner::GridPathPlanner(PartiallyKnownGrid* grid, bool use_adaptive_a_star) {
	mygrid = grid;
	use_adaptive = use_adaptive_a_star;
	width = mygrid->GetWidth();
	height = mygrid->GetHeight();
	
	goal = grid->GetGoalLocation();

	//initialize hvector with manhattan distance
	hvector.resize(width);
	for (int x = 0; x < width; x++){
		hvector[x].resize(height);
	}
	
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			xyLoc temp = xyLoc(x,y);
			hvector[x][y] = calculate_manhatten_h(temp, goal);
		}
	}
	g_current_start = 0;

	last_num_expansion = 0;
	current_num_expansion = 0;
}
GridPathPlanner::~GridPathPlanner(){
	delete mygrid;
}

//GetNextMove returns the next move find by the repeated/forward A* Algorithm
xyLoc GridPathPlanner::GetNextMove(PartiallyKnownGrid* grid) {
	
	//repeated A*
	//initiate the open_list and closed_list for A* algorithm
	//goalMap and fMap for storing the g-value and f-value of the most recent finished round
	multiset<pair <xyLoc, pair<int, int> >, LessThanByFComparator> open_list;
	std::map<xyLoc,xyLoc,MapComp> closed_list; 
	std::map<xyLoc,int,MapComp> goalMap;
	std::map<xyLoc,int,MapComp> fMap;
	std::map<xyLoc,xyLoc,MapComp> ancestor; 
	current_num_expansion = 0;
	start = grid->GetCurrentLocation();

	int goal_distance = -1;
	int h = hvector[start.x][start.y];
	int g = g_current_start;
	int f = g + h;
	//update g value and f value
	goalMap.insert(pair<xyLoc, int>(start, g));
	fMap.insert(pair<xyLoc, int>(start, f));
	//update open list
	open_list.insert(make_pair(start, make_pair(f, g)));
	//record the node expanded from
	xyLoc p = start;
	while(!open_list.empty()){
		//1 remove the top priority element
		multiset<pair<xyLoc, pair<int, int> > >::iterator it = open_list.begin();
		pair<xyLoc, pair<int, int> > s = *it;
		open_list.erase(it);
		//2 add s to close list
		if(s.first == start ){
			closed_list.insert(make_pair(s.first, xyLoc(-1, -1)));
		}else{
			closed_list.insert(make_pair(s.first, ancestor.at(s.first)));
		}
		//3a if s = goal then trace back to start, STOP
		if(s.first==goal){
			//cout << "goal distanec: " << endl;
			try{
				goal_distance = goalMap.at(goal);
			}catch(exception& e){
				cout << "map exception line 103, find goal" << endl;
			}
			
			
			break;
		}else{
			current_num_expansion++;//as answered on piazza
		//3b
			/*Generate successors/neighbors of s, compute their f
			values, and add them to open_list if they are
			not in the closed_list (so we don’t re-explore), 
			or if they are already in the open list, 
			update them if they have a smaller f value*/
			//Generate successors of s
			std::vector<xyLoc> neighbors;
			neighbors.push_back(xyLoc(s.first.x+1, s.first.y));
			neighbors.push_back(xyLoc(s.first.x-1, s.first.y));
			neighbors.push_back(xyLoc(s.first.x, s.first.y+1));
			neighbors.push_back(xyLoc(s.first.x, s.first.y-1));
			for (int i = 0; i < neighbors.size(); i++) {
				xyLoc n = neighbors[i];
				//check location validity, blockage
				if (!grid->IsValidLocation(n) || grid->IsBlocked(n)) {
					neighbors[i] = neighbors.back();
					neighbors.pop_back();
					i--;
				}
			}
			
			if(neighbors.size()>0){
				for (int i = 0; i < neighbors.size(); i++) {
					//if they are not in the closed list
						g = goalMap.find(s.first)->second + 1;
						h = hvector[neighbors[i].x][neighbors[i].y];
						f = g + h;
						int old_f = -1;
						int old_g = -1;
						if(goalMap.find(neighbors[i]) != goalMap.end()){//if exist(either in open or close)
							old_g = goalMap.find(neighbors[i])->second;
						}		
						if(fMap.find(neighbors[i]) != fMap.end()){
							old_f = goalMap.find(neighbors[i])->second;
						}
						//add them to open_list if they are not inside close list
						if(closed_list.find(neighbors[i]) == closed_list.end()){
							multiset<pair<xyLoc, pair<int, int> > >::iterator it= open_list.find(make_pair(neighbors[i], make_pair(old_f, old_g)));
							if(it != open_list.end()){//if the location is inside the open_list
								//update them if they have a smaller f value as well as update g values
								if(f < old_f){
									open_list.erase(it);
									open_list.insert(make_pair(neighbors[i], make_pair(f, g)));	
									fMap.find(neighbors[i])->second = f;
									goalMap.find(neighbors[i])->second = g;
									ancestor.at(neighbors[i]).x =s.first.x;
									ancestor.at(neighbors[i]).y =s.first.y;
								}
							}else{//if the location is not inside the open list
								open_list.insert(make_pair(neighbors[i], make_pair(f, g)));
								fMap.insert(pair<xyLoc, int>(neighbors[i], f));
								goalMap.insert(pair<xyLoc, int>(neighbors[i], g));	
								ancestor.insert(pair<xyLoc, xyLoc>(neighbors[i],s.first));	
							}
						}
						//recode the node expanded from
						p.x = s.first.x;
						p.y = s.first.y;
				}
			}
			

			

			
		}
	}
	last_num_expansion = closed_list.size();
	xyLoc curr = goal;
	xyLoc nextStep;
	int this_goal_d = goalMap.at(goal);
	while(curr != start){

		try{
			//set h for adaptive a 
			if(use_adaptive==true){
				hvector[curr.x][curr.y]= this_goal_d - goalMap.at(curr);
			}
		}catch(exception& e){
			cout << "map exception line 188" << endl;
		}
		//iteration increment
		try{
			//increment
			curr = closed_list.at(curr);
			//set the next step to return
			if(closed_list.at(curr) == start){			
				nextStep = curr;
			}
		}catch(exception& e){
			cout << "map exception line 199" << endl;
			break;
		}
		
	}
	
	try{
		//remember the first g for next try
		if(nextStep.x != -1)
			g_current_start = goalMap.at(nextStep);

	}catch(exception& e){
		cout << "**Exception: nextStep: x: "<<nextStep.x << "y: " << nextStep.y << endl;
		cout << "map exception line 209" << endl;
	}

	if(nextStep.x == -1 && nextStep.y==-1){
		return goal;
	}
	if(use_adaptive==true){
		for(map<xyLoc,xyLoc,MapComp>::iterator it = closed_list.begin(); it!= closed_list.end(); ++it) {
			hvector[it->first.x][it->first.y] = this_goal_d - goalMap.at(it->first);
		}
	}
	return nextStep;
}

int GridPathPlanner::GetNumExpansions() {
	return last_num_expansion;
	
}

int GridPathPlanner::calculate_manhatten_h(xyLoc l1, xyLoc l2){

		int h = abs(l1.x - l2.x) + abs(l1.y - l2.y);
		return h;
}
