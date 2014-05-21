//
//  Structures.cpp
//  PathPlanning 2.0
//
//  Created by Charles on 2014-04-28.
//  Copyright (c) 2014 Charles. All rights reserved.
//

#include "Structures.h"

GraphParameter::GraphParameter(int d, int r, int v, int s, int m){
    dilationType = d;
    robotRadius = r;
    voxelSize = v;
    safeSpace = s;
    maxTravelDistance = m;
}