//
//  Graph.cpp
//  PathPlanning 2.0
//
//  Created by Charles on 2014-04-25.
//  Copyright (c) 2014 Charles. All rights reserved.
//

#include "Graph.h"
    //constructor assign memery for graph
Graph::Graph( cv::Mat obstacleImage ){
    obstacleImage.copyTo(_obstacleImage);
    obstacleImage.copyTo(_obstacleImageCopy);
    obstacleImage.copyTo(_resultImage);
    _mapHeight = _obstacleImage.rows;
    _mapWidth = _obstacleImage.cols;
    _obstacleMap = new vertex* [_mapHeight];
    _pathLength = 0;
    _destination = nullptr;
    _initial = nullptr;
    _dilationSize = 0;
    _dilationType = 1;
    _safeSpace = 0;
    _voxelSize = 1;
    _diagnolLength = sqrt( _mapHeight * _mapHeight + _mapWidth * _mapWidth );
    for (int i=0; i<_mapHeight; i++) {
        _obstacleMap[i] = new vertex[_mapWidth];
    }
        //initialize graph
    for (int y=0; y<_mapHeight; y++) {
        for (int x=0; x<_mapWidth; x++) {
            _obstacleMap[y][x].position.y = y;
            _obstacleMap[y][x].position.x = x;
            _obstacleMap[y][x].isObstacle = false;
        }
    }
    if ( !_obstacleImage.data ){
        perror("Failed to read image.\n");
    }
}

Graph::Graph(){
    _obstacleMap = nullptr;
}
void Graph::loadMap(cv::Mat obstacleImage){
    if ( !_obstacleMap ) {
        _obstacleImage = obstacleImage;
        _mapHeight = _obstacleImage.rows;
        _mapWidth = _obstacleImage.cols;
        _obstacleMap = new vertex* [_mapHeight];
        _pathLength = 0;
        _destination = nullptr;
        _initial = nullptr;
        _dilationSize = 0;
        _dilationType = 1;
        _safeSpace = 0;
        _voxelSize = 1;
        _diagnolLength = sqrt( _mapHeight * _mapHeight + _mapWidth * _mapWidth );
        for (int i=0; i<_mapHeight; i++) {
            _obstacleMap[i] = new vertex[_mapWidth];
        }
            //initialize graph
        for (int y=0; y<_mapHeight; y++) {
            for (int x=0; x<_mapWidth; x++) {
                _obstacleMap[y][x].position.y = y;
                _obstacleMap[y][x].position.x = x;
                _obstacleMap[y][x].isObstacle = false;
            }
        }
        if ( !_obstacleImage.data ){
            perror("Failed to read image.\n");
        }
    }
    else
        printf("Can not reload map (Map already loaded).\n");

}

void Graph::_setGraphParameters(int dilationType, int robotRadius, int voxelSize, int safeSpace){
    _robotRadius = robotRadius;
    _voxelSize = voxelSize;
    _safeSpace = safeSpace;
    if ( dilationType == 0 ){
        _dilationType = cv::MORPH_RECT;
    }
    else if( dilationType == 1 ){
        _dilationType = cv::MORPH_ELLIPSE;
    }
    else{
        printf("Wrong dilation type setting, dilation type set to defult: ellipse.\n");
        _dilationType = cv::MORPH_ELLIPSE;
    }
    _dilationSize = (_robotRadius + _safeSpace) / _voxelSize;
}

void Graph::_setGraphParameters( GraphParameter p ){
    _robotRadius = p.robotRadius;
    _voxelSize = p.voxelSize;
    _safeSpace = p.safeSpace;
    if ( p.dilationType == 0 ){
        _dilationType = cv::MORPH_RECT;
    }
    else if( p.dilationType == 1 ){
        _dilationType = cv::MORPH_ELLIPSE;
    }
    else{
        printf("Wrong dilation type setting, dilation type set to defult: ellipse.\n");
        _dilationType = cv::MORPH_ELLIPSE;
    }
    _dilationSize = (_robotRadius + _safeSpace) / _voxelSize;
}

void Graph::_dilation(){
    cv::Mat element = getStructuringElement( _dilationType,
                                            cv::Size( 2 * _dilationSize+1, 2 * _dilationSize+1),
                                            cv::Point(_dilationSize,_dilationSize) );
    dilate(_obstacleImage,_obstacleImage,element);
}

void Graph::setup(GraphParameter p){
    _setGraphParameters(p);
    _dilation();
    _buildMap();
}

void Graph::rebuildMap(GraphParameter p){
    _obstacleImage = _obstacleImageCopy;
    _mapHeight = _obstacleImage.rows;
    _mapWidth = _obstacleImage.cols;
    _obstacleMap = new vertex* [_mapHeight];
    _pathLength = 0;
    _destination = nullptr;
    _initial = nullptr;
    _dilationSize = 0;
    _dilationType = 1;
    _safeSpace = 0;
    _voxelSize = 1;
    _diagnolLength = sqrt( _mapHeight * _mapHeight + _mapWidth * _mapWidth );
    for (int i=0; i<_mapHeight; i++) {
        _obstacleMap[i] = new vertex[_mapWidth];
    }
        //initialize graph
    for (int y=0; y<_mapHeight; y++) {
        for (int x=0; x<_mapWidth; x++) {
            _obstacleMap[y][x].position.y = y;
            _obstacleMap[y][x].position.x = x;
            _obstacleMap[y][x].isObstacle = false;
        }
    }
    if ( !_obstacleImage.data ){
        perror("Failed to read image.\n");
    }

    _setGraphParameters(p);
    _dilation();
    _buildMap();
}

void Graph::setDestination( vertex* destination){
    _obstacleMap[destination->position.y][destination->position.x].destination = true;
    _destination = &_obstacleMap[destination->position.y][destination->position.x];
    destination = _destination;
}

vertex* Graph::setDestination( int y, int x ){
    _obstacleMap[y][x].destination = true;
    _destination = &_obstacleMap[y][x];
    return _destination;
}

vertex* Graph::createMapEntry( int y, int x ){
    _initial = &_obstacleMap[y][x];
    return _initial;
}

vertex* Graph::createMapEntry( point2d position ){
    _initial = &_obstacleMap[position.y][position.x];
    return _initial;
}

void Graph::createMapEntry( vertex* entry ){
    entry = &_obstacleMap[entry->position.y][entry->position.x];
}

void Graph::linkPath(){
    if ( _destination->pi == NULL ) {
        perror("Link path failed, no path to destination\n");
    }
    else {
            //printf("Position x:%d, y:%d drawed.\n", _destination->pi->position.x, _destination->pi->position.y);
        _linkPath( _destination );
    }
}

void Graph::_linkPath( vertex* current ){
    if ( current->pi != NULL ) {
        current->pi->de = current;
            //printf("Position x:%d, y:%d drawed.\n", current->position.x, current->position.y);
        _linkPath( current->pi );
    }
}

void Graph::drawPath(){
    if ( _destination->pi == NULL ) {
        perror("Draw path failed, no path to destination!\n");
    }
    else{
        _drawPath( _destination );
        for (int y = 0; y < _resultImage.rows; y++) {
            for (int x = 0; x < _resultImage.cols; x++) {
                if(!_obstacleMap[y][x].isObstacle) _resultImage.at<float>(y,x) = 255;
                else _resultImage.at<float>(y,x) = 0;
            }
        }
    }
}

void Graph::displayPath(){
    cv::namedWindow("Path",cv::WINDOW_AUTOSIZE);
    cv::imshow("Path", _resultImage);
    cv::imwrite("./path.png", _resultImage);
    _obstacleImageCopy.copyTo(_resultImage);
}

void Graph::displayScanArea(){
    for (int y = 0; y < _resultImage.rows; y++) {
        for (int x = 0; x < _resultImage.cols; x++) {
            if( _obstacleMap[y][x].mark == "VISITED" ) _resultImage.at<float>(y,x) = 0;
            else _resultImage.at<float>(y,x) = 255;
        }
    }
    cv::namedWindow("Scanned Area",cv::WINDOW_AUTOSIZE);
    cv::imshow("Scanned Area", _resultImage);
    cv::imwrite("./scanArea.png", _resultImage);
    _obstacleImageCopy.copyTo(_resultImage);
}

void Graph::_drawPath( vertex* current ){
    if ( current->pi != NULL ) {
        if( ( current->position.x == current->pi->position.x &&
             current->position.y != current->pi->position.y ) ||
           ( current->position.x != current->pi->position.x &&
            current->position.y == current->pi->position.y ) )
            _pathLength += 1;
        else
            _pathLength += 1.414;
        current->isObstacle = true;
        _drawPath( current->pi );
    }
    
}

void Graph::showMap(){
    if ( _resultImage.data ) {
        cv::namedWindow("Obstacle Map",cv::WINDOW_AUTOSIZE);
        cv::imshow("Obstacle Map", _obstacleImageCopy);
        printf("Press any key to continue...\n");
        cv::waitKey(0);
    }
    else
    std::cout << "Can't read image!" << std::endl;
}

Graph::~Graph(){
    for ( int y=0; y<_mapHeight; y++ ) {
        delete [] _obstacleMap[y];
    }
    delete [] _obstacleMap;
    printf("Graph destroyed!\n");
}

void Graph::_buildMap(){
        //Adding neighbor nodes to adjacency linked list
    for( int y = 0; y<_mapHeight; y++ ){
        for( int x = 0; x<_mapWidth; x++ ){
                //Mark obstacle
            _obstacleMap[y][x].position.x = x;
            _obstacleMap[y][x].position.y = y;
            if( _obstacleImage.at<float>(y,x) > 0 )
                _obstacleMap[y][x].isObstacle = true;
            else
                _obstacleMap[y][x].isObstacle = false;
            
                //Add neighbor nodes to adjacency list
            if( _obstacleMap[y][x].position.x > 0 &&
               _obstacleMap[y][x].position.x < (_mapWidth - 1) &&
               _obstacleMap[y][x].position.y > 0 &&
               _obstacleMap[y][x].position.y < (_mapHeight - 1) ) //1
                {
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x-1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x+1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y][x-1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y][x+1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x-1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x+1]);
                
                }
            if( _obstacleMap[y][x].position.x == 0 &&
               _obstacleMap[y][x].position.y == 0 ) //2
                {
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y][x+1]);
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x]);
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x+1]);
                }
            if( _obstacleMap[y][x].position.x == 0 &&
               _obstacleMap[y][x].position.y > 0 &&
               _obstacleMap[y][x].position.y < (_mapHeight - 1) ) //3
                {
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x]);
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x+1]);
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y][x+1]);
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x]);
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x+1]);
                
                }
            if( _obstacleMap[y][x].position.x == 0 &&
               _obstacleMap[y][x].position.y == (_mapHeight - 1) ) //4
                {
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x]);
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x+1]);
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y][x+1]);
                }
            if( _obstacleMap[y][x].position.x > 0 &&
               _obstacleMap[y][x].position.x < (_mapWidth - 1) &&
               _obstacleMap[y][x].position.y == 0 ) //5
                {
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y][x-1]);
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y][x+1]);
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x-1]);
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x]);
                 _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x+1]);
                }
            if( _obstacleMap[y][x].position.x == (_mapWidth - 1) &&
               _obstacleMap[y][x].position.y == 0 ) //6
                {
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y][x-1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x-1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x]);
                }
            if( _obstacleMap[y][x].position.x == (_mapWidth - 1) &&
               _obstacleMap[y][x].position.y > 0 &&
               _obstacleMap[y][x].position.y < (_mapHeight - 1) ) //7
                {
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x-1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y][x-1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x-1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y+1][x]);
                }
            if(_obstacleMap[y][x].position.x == (_mapWidth - 1) &&
               _obstacleMap[y][x].position.y == (_mapHeight - 1) ) //8
                {
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x-1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y][x-1]);
                }
            if( _obstacleMap[y][x].position.x > 0 &&
               _obstacleMap[y][x].position.x < (_mapWidth - 1) &&
               _obstacleMap[y][x].position.y == (_mapHeight - 1) ) //9
                {
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x-1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y-1][x+1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y][x-1]);
                _obstacleMap[y][x].adjacent.push_back(&_obstacleMap[y][x+1]);
                }
            }
        }
    for( int y=0; y<_dilationSize; y++ ){
        for( int x =0; x<_mapWidth; x++ ){
             _obstacleMap[y][x].isObstacle = true;
        	}
        
        }
    for( int y=_mapHeight-_dilationSize; y<_mapHeight; y++){
        for( int x =0; x<_mapWidth; x++){
             _obstacleMap[y][x].isObstacle = true;
        	}
        
        }
    for( int x=0; x<_dilationSize; x++){
        for(int y=_dilationSize; y<_mapHeight-_dilationSize; y++){
             _obstacleMap[y][x].isObstacle = true;
        	}
        }
    for( int x = _mapWidth-_dilationSize; x<_mapWidth; x++){
        for( int y=_dilationSize; y<_mapHeight-_dilationSize; y++){
             _obstacleMap[y][x].isObstacle = true;
        	}
        }
}