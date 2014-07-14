//
//  PathPlanning.cpp
//  PathPlanning 2.0
//
//  Created by Charles on 2014-04-26.
//  Copyright (c) 2014 Charles. All rights reserved.
//

#include "PathPlanning.h"
bool sortFunction(vertex* L, vertex* R) { return L->weight < R->weight; }
PathPlanning::PathPlanning( cv::Mat obstacleMap, GraphParameter p ){
    _heuristicWeight = 200;
    _map[0] = new Graph(obstacleMap);
    _map[0]->setup(p);
    setMap(0);
    _maxTravelDistance = p.maxTravelDistance;
    _maxTravelDistanceCopy = _maxTravelDistance;
    _voxelSize = p.voxelSize;
}

PathPlanning::PathPlanning( std::vector<cv::Mat> obstacleMap, GraphParameter p ){
    _heuristicWeight = 100;
    _currentMapIndex = 0;
    for (std::vector<cv::Mat>::iterator it = obstacleMap.begin(); it != obstacleMap.end(); ++it ) {
        _map[_currentMapIndex] = new Graph(*it);
        _map[_currentMapIndex]->setup(p);
        _currentMapIndex++;
    }
        //parallel code to set map automatically
    setMap(0);
    _maxTravelDistance = p.maxTravelDistance;
    _maxTravelDistanceCopy = _maxTravelDistance;
    _voxelSize = p.voxelSize;
}

void PathPlanning::setDestination( vertex* destination ){
    try {
        __MapIndexAlerm();
        _currentMap->setDestination( destination );
        _destination[_currentMapIndex] = _currentMap->getDestination();
    } catch(int e) {
        if (e == IndexError) {
            printf("From: %s\n",__FUNCTION__); exit(IndexError);
        }
    }
}

vertex* PathPlanning::setDestination( int y, int x ){
    try {
        __MapIndexAlerm();
        _destination[_currentMapIndex] = _currentMap->setDestination( y, x );
    } catch (int e) {
        if (e == IndexError) {
            printf("From: %s\n",__FUNCTION__); exit(IndexError);
        }
    }
    return _currentMap->getDestination();
}

void PathPlanning::setStart( vertex* start ){
    try {
        __MapIndexAlerm();
        _currentMap->createMapEntry( start );
        _start[_currentMapIndex] = start;
    } catch (int e) {
        if (e == IndexError) {
            printf("From: %s\n",__FUNCTION__); exit(IndexError);
        }
    }
}

vertex* PathPlanning::setStart( int y, int x ){
    try {
        __MapIndexAlerm();
        _start[_currentMapIndex] = _currentMap->createMapEntry(y, x);
    } catch (int e) {
        if (e == IndexError) {
            printf("From: %s\n",__FUNCTION__); exit(IndexError);
        }
    }
    
    return _start[_currentMapIndex];
}

double PathPlanning::heuristic( vertex* next ){
    double cos(0),
        //vectors:
        //destination
    vd_x( _destination[_currentMapIndex]->position.x - \
         _currentlocation->position.x ),
    vd_y( _destination[_currentMapIndex]->position.y - \
         _currentlocation->position.y ),
        //neighbor vertexes
    vn_x( next->position.x - _currentlocation->position.x ),
    vn_y( next->position.y - _currentlocation->position.y );
#ifdef __SOURCE_DESTINATION__
        //source - destination
    double
    vs_x( _destination[_currentMapIndex]->position.x - \
         _start[_currentMapIndex]->position.x),
    vs_y( _destination[_currentMapIndex]->position.y - \
         _start[_currentMapIndex]->position.y );
#endif
        //vector amplitude
    double vd_val( sqrt( vd_x * vd_x + vd_y * vd_y ) ),
           vn_val( sqrt( vn_x * vn_x + vn_y * vn_y ) );
#ifdef __SOURCE_DESTINATION__
    double vs_val( sqrt( vs_x * vs_x + vs_y * vs_y ) );
#endif
    cos = ( ( vd_x * vn_x ) + ( vd_y * vn_y ) )/ \
            ( vd_val * vn_val );
    return ( 1 - (vd_val / _currentMap->getDiagnolLength()) ) \
            * _heuristicWeight * ( -cos + 1 );
}

void PathPlanning::AStar(){
    if (_start[_currentMapIndex]->isObstacle == true) {
        printf("Robot is located inside obstacle, throw exeption to call BFS.\n");
        throw PhantomException;
    }
    std::vector<vertex*> OPENSET;
    OPENSET.push_back(_start[_currentMapIndex]);
    _start[_currentMapIndex]->set = "OPEN";
    _start[_currentMapIndex]->weight = 0;
    _start[_currentMapIndex]->pi = 0;
    while (!OPENSET.empty()) {
        if (OPENSET.empty()) {
            printf("No nodes added in open set, no solution exists.\n");
            throw NoSolutionException;
        }
#ifdef _ASORT_
        std::sort(OPENSET.begin(), OPENSET.end(), sortFunction);
        _currentlocation = OPENSET.front();
        _currentlocation->mark = "VISITED";
        _currentlocation->set = "CLOSE";
        OPENSET.erase(OPENSET.begin());
#endif
#ifdef _AMIN_
        std::vector<vertex*>::iterator minVertex = OPENSET.begin();
        for (std::vector<vertex*>::iterator it = OPENSET.begin(); it != OPENSET.end(); ++it) {
            if ((*it)->weight < (*minVertex)->weight) {
                minVertex = it;
            }
        }
        _currentlocation = *minVertex;
        _currentlocation->mark = "VISITED";
        _currentlocation->set = "CLOSE";
        OPENSET.erase(minVertex);
#endif
        if (_currentlocation->destination) {
            printf("Solution given by A Star.(You can disable draw path feature in function: %s())\n",__FUNCTION__);
            _currentMap->linkPath();
            _currentMap->drawPath();
            return ;
        }
        for (int i = 0; i < _currentlocation->adjacent.size(); i++) {
            double heuristicValue = heuristic(_currentlocation->adjacent[i]);
            double estimate = 0;
            if ( (_currentlocation->position.x == _currentlocation->adjacent[i]->position.x && \
                  _currentlocation->position.y != _currentlocation->adjacent[i]->position.y) || \
                 (_currentlocation->position.x != _currentlocation->adjacent[i]->position.x && \
                  _currentlocation->position.y == _currentlocation->adjacent[i]->position.y )) {
                    estimate = _currentlocation->weight + heuristicValue + 1;
            }
            else {
                    estimate = _currentlocation->weight + heuristicValue + 1.41;
            }
            if ( _currentlocation->adjacent[i]->set == "NONE" && \
                 !_currentlocation->adjacent[i]->isObstacle ) {
                OPENSET.push_back(_currentlocation->adjacent[i]);
                _currentlocation->adjacent[i]->set = "OPEN";
                _currentlocation->adjacent[i]->weight = estimate;
                _currentlocation->adjacent[i]->pi = _currentlocation;
            }
            else if ( (_currentlocation->adjacent[i]->set == "OPEN" || \
                  _currentlocation->adjacent[i]->set == "CLOSE") && \
                  !_currentlocation->adjacent[i]->isObstacle ) {
                if (estimate < _currentlocation->adjacent[i]->weight) {
                    _currentlocation->adjacent[i]->weight = estimate;
                    _currentlocation->adjacent[i]->pi = _currentlocation;
                    if (_currentlocation->adjacent[i]->set == "CLOSE") {
                        _currentlocation->adjacent[i]->set = "OPEN";
                        _currentlocation->pi = _currentlocation->adjacent[i]->pi;
                        OPENSET.push_back(_currentlocation->adjacent[i]);
                    }
                }
            }
        }
    }
    printf("No Solution Found");
    throw NoSolutionException;
}


void PathPlanning::Dijkstra(){
    if (_start[_currentMapIndex]->isObstacle == true) {
        printf("Robot is located inside obstacle, throw exeption to call BFS.\n");
        throw PhantomException;
    }
    std::vector<vertex*> OPENSET;
    OPENSET.push_back(_start[_currentMapIndex]);
    _start[_currentMapIndex]->set = "OPEN";
    _start[_currentMapIndex]->weight = 0;
    _start[_currentMapIndex]->pi = 0;
    while (!OPENSET.empty()) {
        if (OPENSET.empty()) {
            printf("No nodes added in open set, no solution exists.\n");
            throw NoSolutionException;
        }
#ifdef _DSORT_
        std::sort(OPENSET.begin(), OPENSET.end(), sortFunction);
        _currentlocation = OPENSET.front();
        _currentlocation->mark = "VISITED";
        _currentlocation->set = "CLOSE";
        OPENSET.erase(OPENSET.begin());
#endif
#ifdef _DMIN_
        std::vector<vertex*>::iterator minVertex = OPENSET.begin();
        for (std::vector<vertex*>::iterator it = OPENSET.begin(); it != OPENSET.end(); ++it) {
            if ((*it)->weight < (*minVertex)->weight) {
                minVertex = it;
            }
        }
        _currentlocation = *minVertex;
        _currentlocation->mark = "VISITED";
        _currentlocation->set = "CLOSE";
        OPENSET.erase(minVertex);
#endif
        if (_currentlocation->destination) {
            printf("Solution given by Dijkstra.(You can disable draw path feature in function: %s())\n",__FUNCTION__);
            _currentMap->linkPath();
            _currentMap->drawPath();
            return ;
        }
        for (int i = 0; i < _currentlocation->adjacent.size(); i++) {
            double estimate = 0;
            if ( (_currentlocation->position.x == _currentlocation->adjacent[i]->position.x && \
                  _currentlocation->position.y != _currentlocation->adjacent[i]->position.y) || \
                (_currentlocation->position.x != _currentlocation->adjacent[i]->position.x && \
                 _currentlocation->position.y == _currentlocation->adjacent[i]->position.y )) {
                    estimate = _currentlocation->weight + 1;
                }
            else {
                estimate = _currentlocation->weight + 1.41;
            }
            if ( _currentlocation->adjacent[i]->set == "NONE" && \
                !_currentlocation->adjacent[i]->isObstacle ) {
                OPENSET.push_back(_currentlocation->adjacent[i]);
                _currentlocation->adjacent[i]->set = "OPEN";
                _currentlocation->adjacent[i]->weight = estimate;
                _currentlocation->adjacent[i]->pi = _currentlocation;
            }
            else if ( (_currentlocation->adjacent[i]->set == "OPEN" || \
                       _currentlocation->adjacent[i]->set == "CLOSE") && \
                     !_currentlocation->adjacent[i]->isObstacle ) {
                if (estimate < _currentlocation->adjacent[i]->weight) {
                    _currentlocation->adjacent[i]->weight = estimate;
                    _currentlocation->adjacent[i]->pi = _currentlocation;
                    if (_currentlocation->adjacent[i]->set == "CLOSE") {
                        _currentlocation->adjacent[i]->set = "OPEN";
                        _currentlocation->pi = _currentlocation->adjacent[i]->pi;
                        OPENSET.push_back(_currentlocation->adjacent[i]);
                    }
                }
            }
        }
    }
    printf("No Solution Found");
    throw NoSolutionException;
}



void PathPlanning::BFS(){
    bool found = false;
    _start[_currentMapIndex]->color = "GRAY";
    _start[_currentMapIndex]->weight = 0;
    _start[_currentMapIndex]->pi = NULL;
    std::vector<vertex*> GRAY;
    GRAY.push_back( _start[_currentMapIndex] );
    while (!GRAY.empty() && found == false ) {
        GRAY.front()->mark = "VISITED";
        for (std::vector<vertex*>::iterator it = GRAY.front()->adjacent.begin(); it != GRAY.front()->adjacent.end(); ++it) {
            if( (*it)->color == "WHITE" && (*it)->isObstacle == false ){
                (*it)->color = "GRAY";
                (*it)->weight += GRAY.front()->weight;
                (*it)->pi = GRAY.front();
                GRAY.push_back(*it);
            }
            GRAY.front()->color = "BLACK";
            if( (*it)->destination ){

                found = true;
                _currentMap->linkPath();
                _currentMap->drawPath();
                printf("Solution given by BFS.\n(You can disable draw path feature in function: %s())\n",__FUNCTION__);
            }
        }
        GRAY.erase(GRAY.begin());
    }
    if( found == false ) printf("Path not found.\n");

}

void PathPlanning::buildPath(){
    _currentNode = _start[_currentMapIndex];
    while(_currentNode->de != NULL){
        cv::Point2d temp;
        _currentNode = mergeWayPoint(_currentNode);
#ifdef _DEBUG_PATHPLANNING_
        std::cout << "Point " << _currentNode->position.x << ", " << _currentNode->position.y << " is added. \n";
#endif
        temp.x = _currentNode->position.x * _voxelSize;
        temp.y = _currentNode->position.y * _voxelSize;
        _path[_currentMapIndex].push_back(temp);
        if(_currentNode->de != NULL){
            _currentNode = _currentNode->de;
            _maxTravelDistance = _maxTravelDistanceCopy;
        }
    }
}

vertex* PathPlanning::mergeWayPoint(vertex* current){
    if (_maxTravelDistance>0 && current->de != NULL && current->de->de != NULL) {
        int dy10 = current->de->position.y - current->position.y,
            dx10 = current->de->position.x - current->position.x,
            dy21 = current->de->de->position.y - current->de->position.y,
            dx21 = current->de->de->position.x - current->de->position.x;
        if (dy10 * dx21 == dx10 * dy21) {
            if ( (current->position.x == current->de->position.x && \
                  current->position.y != current->de->position.y) || \
                    (current->position.x != current->de->position.x && \
                     current->position.y == current->de->position.y)) {
                        _maxTravelDistance -= _voxelSize;
            }
            else
                _maxTravelDistance -= 1.41 * _voxelSize;
            current = mergeWayPoint(current->de);
        }
        else {
            return current->de;
        }
        
    }
    return current;
}

vertex* PathPlanning::mergeWayPoint(){
    while (_maxTravelDistance>0 && _currentNode->de != NULL && _currentNode->de->de != NULL) {
        int dy10 = _currentNode->de->position.y - _currentNode->position.y,
        dx10 = _currentNode->de->position.x - _currentNode->position.x,
        dy21 = _currentNode->de->de->position.y - _currentNode->de->position.y,
        dx21 = _currentNode->de->de->position.x - _currentNode->de->position.x;
        if (dy10 * dx21 == dx10 * dy21) {
            if ( (_currentNode->position.x == _currentNode->de->position.x && \
                  _currentNode->position.y != _currentNode->de->position.y) || \
                (_currentNode->position.x != _currentNode->de->position.x && \
                 _currentNode->position.y == _currentNode->de->position.y)) {
                    _maxTravelDistance -= _voxelSize;
                }
            else
                _maxTravelDistance -= 1.41 * _voxelSize;
            _currentNode = _currentNode->de;
        }
        else {
#ifdef __Merge_Waypoint__
            vertex* tempVertex = _currentNode;
            int dx = _currentNode->position.x - _currentNode->de->position.x,
                dy = _currentNode->position.y - _currentNode->de->position.y;
#endif
            while (_maxTravelDistance>0 && _currentNode->de != NULL) {
                _maxTravelDistance -= 1.41 * _voxelSize;
                _currentNode = _currentNode->de;
            }
            
        }
            
    }
    return _currentNode;
}


void PathPlanning::showResult(){
    try {
        __MapIndexAlerm();
        _currentMap->displayPath();
        _currentMap->displayScanArea();
        
        
        printf("Path length: %f.\n",_currentMap->getPathLength());
        printf("Press any key to continue...\n");
        cv::waitKey(0);
    } catch (int e) {
        if (e == IndexError) {
            printf("From: %s\n",__FUNCTION__); exit(IndexError);
        }
    }
}

void PathPlanning::showMap(){
    try {
        __MapIndexAlerm();
        _map[_currentMapIndex]->displayPath();
        printf("Press any key to continue...\n");
        cv::waitKey(0);
    } catch (int e) {
        if (e == IndexError) {
            printf("From: %s\n",__FUNCTION__); exit(IndexError);
        }
    }
}

void PathPlanning::printPath(){
    std::cout << "Way Point in path:" << std::endl;
    for (std::vector<cv::Point2d>::iterator it = _path[_currentMapIndex].begin(); it != _path[_currentMapIndex].end(); ++it) {
        printf("x: %.0f, y: %.0f \n",(*it).x, (*it).y);
    }
}



void PathPlanning::__MapIndexAlerm(){
    if ( !_currentMap ) {
        printf("Map index is null, no map can load aborted.\n\
this->setMapIndex() never get called.\n\
IndexID:%d\n\
Where:%s \n",_currentMapIndex,__FUNCTION__);
        throw IndexError;
    }
}

PathPlanning::~PathPlanning(){
    for (int i = 0; i < THREADS; i++) {
        if ( _map[i] != NULL ) {
            printf("Map No.%d is destroyed!\n",i);
            delete _map[i];
        }
    }
}