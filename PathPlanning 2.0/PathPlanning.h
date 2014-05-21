//
//  PathPlanning.h
//  PathPlanning 2.0 beta
//
//  Created by Charles on 2014-04-26.
//  Copyright (c) 2014 Charles. All rights reserved.
//

#ifndef __PathPlanning_2_0__PathPlanning__
#define __PathPlanning_2_0__PathPlanning__
#define _DSORT_
#define _AMIN_
#include "Graph.h"
#include <algorithm>
#include <climits>
#define THREADS 9 // how many threads available to process map(fragments)
#define MEM 1024*1024*1024*3 // set maximum memory size
#define IndexError 109
#define PhantomException 91
#define NoSolutionException 99
class PathPlanning{
public: //setup
    PathPlanning( cv::Mat obstacleMap, GraphParameter p ); //done
    PathPlanning( std::vector<cv::Mat> obstacleMap, GraphParameter p ); // wait for parallel
    ~PathPlanning(); //half finish
public: //pathplanning
    void setDestination( vertex* destination ); //done
    vertex* setDestination( int y, int x ); //done
    void setStart( vertex* start ); //done
    vertex* setStart( int y, int x ); //done
    void BFS(); //done
    void AStar(); //done
    void Dijkstra(); //done
    vertex* mergeWayPoint( vertex* current );
    vertex* mergeWayPoint();
    void buildPath();
public: //display
    void showMap(); //done
    void showResult(); //done
    void printPath();
private: //parallel
    inline void setMap( int mapIndex ){ _currentMap = _map[mapIndex]; _currentMapIndex = mapIndex; }
    inline void unsetMap() { _currentMap = nullptr; }
private: //map
    Graph* _map[THREADS];
    Graph* _currentMap;
    int _currentMapIndex;
private: //searching information
    vertex* _start[THREADS];
    vertex* _destination[THREADS];
    vertex* _currentlocation;
    int _maxTravelDistance, _maxTravelDistanceCopy;
    inline void _setMaxTravelDistance(int maxDis){ _maxTravelDistance = maxDis; }
private: //heuristic
    double heuristic( vertex* next ); //done
    inline void setHeuristicValue( int weight ) { _heuristicWeight = weight; }
    double _heuristicWeight;
private: //merge path
    std::vector<cv::Point2d> _path[THREADS];
    vertex* _currentNode;
    int _voxelSize;
    inline std::vector<cv::Point2d> returnPath() { return _path[_currentMapIndex]; }
    inline void setVoxelSize(int v){ _voxelSize = v; }
private: //Error Handle
    void __MapIndexAlerm(); //done
    
};

#endif /* defined(__PathPlanning_2_0__PathPlanning__) */
