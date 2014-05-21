//
//  main.cpp
//  PathPlanning 2.0
//
//  Created by Charles on 2014-04-25.
//  Copyright (c) 2014 Charles. All rights reserved.
//

#include <iostream>
#include "Graph.h"
#include "PathPlanning.h"
#include <vector>
#include <time.h>
#define path "/Users/charles/Documents/workspace/C++/PathPlanning 2.0/data/"
int main(int argc, const char * argv[])
{
    using namespace cv;
    using namespace std;
    Mat img;
    clock_t time;
    img = imread(path"obstacle2.png", CV_LOAD_IMAGE_GRAYSCALE);
    img.convertTo(img, CV_32F);
    if ( !img.data ){
        perror("failed load image.\n");
        exit(80);
    }

    GraphParameter p(1,15,1,1,50);
    PathPlanning iplan(img,p);
    iplan.setStart(460, 620);
    iplan.setDestination(320, 20);
    time = clock();
    iplan.AStar();
    time = clock() - time;
    iplan.buildPath();
    cout << "Time: " << (float)time / CLOCKS_PER_SEC << "sec.\n";
    iplan.showResult();
        //iplan.printPath();
    
    return 0;
}

