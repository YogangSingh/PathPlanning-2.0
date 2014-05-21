//
//  Graph.h
//  PathPlanning 2.0 beta
//
//  Created by Charles on 2014-04-25.
//  Copyright (c) 2014 Charles. All rights reserved.
//
//  HOW TO USE:
//
//  IMPORTANT: point2ds order: [y][x]; [0][0] at UPPER LEFT coner
//  setup: 1.use constructor create graph
//         2.set graph parameters
//  use:   0.loadMap is using default constructor
//         1.dilate obstacle map image
//         2.build map
//         3.set destination
//         4.run algorithm in path planning class
//         5.link path after algorithm finish
//         6.draw path if you like

#ifndef PathPlanning_2_0_Graph_h
#define PathPlanning_2_0_Graph_h

#include <opencv2/opencv.hpp>
#include "Structures.h"




class Graph{
    
public: //setup
    Graph( cv::Mat obstacleImage );
    Graph();
    ~Graph();
        //dialationType: 0 - Rectangle; 1 - Ellipse
        //robotRadius(mm), voxelSize(mm)
        //safe space(mm): space between robot and obstacle
public: //graph methods
    void loadMap( cv::Mat obstacleImage );
    void setup( GraphParameter p );
    void rebuildMap( GraphParameter p);
    void setDestination( vertex* destination );
    vertex* setDestination( int y, int x );
    vertex* createMapEntry( int y, int x );
    vertex* createMapEntry( point2d position );
    void createMapEntry( vertex* entry );
    void linkPath();
    void drawPath();
    void displayScanArea();
    void displayPath();
    void showMap();
public: //get parameters
    inline vertex* getDestination() { return _destination; }
    inline double getPathLength() { return _pathLength; }
    inline vertex** getObstacleMap() { return _obstacleMap; }
    inline double getDiagnolLength() { return _diagnolLength; }
    inline float getVoxelSize() { return _voxelSize; }
private: //map information
    vertex** _obstacleMap;
    cv::Mat _obstacleImage; //make own copy of obstaclemap, don't use pointer
    cv::Mat _obstacleImageCopy;
    cv::Mat _resultImage;
    int _mapHeight, _mapWidth;
    double _diagnolLength;
private: //path planning related
    vertex* _initial;
    vertex* _destination;
    double _pathLength;
    int _dilationType, _dilationSize;
    int _robotRadius;
    int _safeSpace;
    float _voxelSize;
private:
    void _setGraphParameters( int dilationType, int robotRadius, int voxelSize, int safeSpace );
    void _setGraphParameters( GraphParameter p );
    void _dilation();
    void _buildMap();
    void _linkPath( vertex* current );
    void _drawPath( vertex* current );
};

#endif
