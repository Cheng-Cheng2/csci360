#ifndef GRID_PATH_PLANNER_H
#define GRID_PATH_PLANNER_H

#include "PartiallyKnownGrid.h"
#include <functional>
#include <vector>
#include <queue>
#include <map>
#include <utility> 
#include <set>
using namespace std;

struct LessThanByFComparator
{
  bool operator()(const pair<xyLoc, pair<int, int> >& lhs, const pair<xyLoc, pair<int, int> >& rhs) const
  {
    if (lhs.second.first != rhs.second.first){
    	return lhs.second.first < rhs.second.first;
    //break tie by g
    }else if(lhs.second.second != rhs.second.second){
    	return lhs.second.second < rhs.second.second;
    //break tie by xyLoc
    }else{
    	//bool lessequal = (lhs.first < rhs.first) || (lhs.first == rhs.first);
    	return lhs.first < rhs.first;
    }
  }
};

struct MapComp {
  bool operator() (const xyLoc& lhs, const xyLoc& rhs) const
  {return lhs<rhs;}
};

class GridPathPlanner{
public:
	GridPathPlanner(PartiallyKnownGrid* grid, bool use_adaptive_a_star = false);
	~GridPathPlanner();

	xyLoc GetNextMove(PartiallyKnownGrid* grid);
	int GetNumExpansions();
	int calculate_manhatten_h(xyLoc l1, xyLoc l2);
private:
	bool use_adaptive;
	PartiallyKnownGrid* mygrid;
	int width;
	int height;
	xyLoc start;
	xyLoc goal;
	xyLoc second;
	vector<vector<int> > hvector;
	int g_current_start;
	vector<int> expansions;
	int last_num_expansion;
	int current_num_expansion;
	/*priority_queue<pair<xyLoc, int> >, vector<pair<xyLoc, int> >, GreaterThanByFComparator > open_list;
	std::map<xyloc,xyloc,MapComp> closed_list; 
	std::map<xyloc,int,MapComp> goalMap; */
};

#endif
