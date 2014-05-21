//
//  Structures.h
//  PathPlanning 2.0
//
//  Created by Charles on 2014-04-28.
//  Copyright (c) 2014 Charles. All rights reserved.
//

#ifndef __PathPlanning_2_0__Structures__
#define __PathPlanning_2_0__Structures__

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

    // 2 dimensional space
struct point2d{
    point2d(){
        x = 0;
        y = 0;
    }
    int x;
    int y;
};
struct GraphParameter{
    GraphParameter(){
        dilationType = 1;
        robotRadius = 0;
        voxelSize = 0;
        safeSpace = 0;
        maxTravelDistance = 1;
    }
    GraphParameter(int dilationType, int robotRadius, int voxelSize, int safeSpace,int maxTravelDistance);
    int dilationType;
    int robotRadius;
    int voxelSize;
    int safeSpace;
    int maxTravelDistance;
};

    //Graph G = (V,E) (|E|<<|V^2|, E=8V) using adjacency linked list
    //see - "Introduction to Algorithms" by Thomas H. Cormen "GRAPH" chapter
struct vertex{
    vertex(){
        color = "WHITE"; //instruction see - "Introduction to Algorithms" by Thomas H. Cormen "GRAPH" chapter
		weight = 0;
		pi = 0;
		de = 0;
		isObstacle = false; //is obstacle - true, not obstacle - false
		destination = false;
		set = "NONE"; //instruction see - A* algorithm @ Wikipedia - OPENSET & CLOSESET
		mark = "NOT_VISITED";
    }
    std::string color;
    std::string set;
    std::string mark; //for counting results - path length, use boolean or char if you want
    double weight;
    point2d position;
    vertex *pi;
    vertex *de;
    std::vector<vertex*> adjacent; //adjacency linked list
    bool isObstacle;
    bool destination;
};

#endif /* defined(__PathPlanning_2_0__Structures__) */
