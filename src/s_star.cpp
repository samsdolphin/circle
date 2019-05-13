#include "a_star/a_star.h"

using namespace std;
using namespace Eigen;

void gridPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    //data = new uint8_t[GLXY_SIZE];

    //memset(data, 0, GLXY_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr * [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++)
        {
            Vector3i tmpIdx(i,j,0);
            Vector3d pos = gridIndex2coord(tmpIdx);
            GridNodeMap[i][j] = new GridNode(tmpIdx, pos);
            //GridNodeMap[i][j]->occupancy = &data[i * GLY_SIZE + j];
        }
    }
}

void gridPathFinder::resetMap()
{   
    //ROS_WARN("expandedNodes size : %d", expandedNodes.size());
    for(auto tmpPtr:expandedNodes)
    {
        //tmpPtr->occupancy = 0; 
        tmpPtr->id = 0;
        tmpPtr->cameFrom = NULL;
        tmpPtr->gScore = INF;
        tmpPtr->fScore = INF;
    }

    for(auto ptr:openSet)
    {   
        //tmpPtr->occupancy = 0; 
        GridNodePtr tmpPtr = ptr.second;
        tmpPtr->id = 0;
        tmpPtr->cameFrom = NULL;
        tmpPtr->gScore = INF;
        tmpPtr->fScore = INF;
    }

    expandedNodes.clear();
    //ROS_WARN("local map reset finish");
}

double gridPathFinder::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{   
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0) {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double gridPathFinder::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{   
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double gridPathFinder::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{   
    return (node2->index - node1->index).norm();
}

double gridPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2, GridField* edf_grid_ptr)
{

    return tie_breaker * getDiagHeu(node1, node2);
}

vector<GridNodePtr> gridPathFinder::retrievePath(GridNodePtr current)
{   
    vector<GridNodePtr> path;
    path.push_back(current);

    while(current->cameFrom != NULL)
    {
        current = current -> cameFrom;
        path.push_back(current);
    }

    return path;
}

void gridPathFinder::AstarSearch(Vector3d start_pt, Vector3d end_pt, GridField* edf_grid_ptr)
{
    double w_v = 5;
    double w_d = 5;
    double w_a = 0.5;

    ros::Time time_1 = ros::Time::now();    

    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx = coord2gridIndex(end_pt);

    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr = new GridNode(end_idx, end_pt);

    openSet.clear();

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    Point cur, tar;

    cur.x = start_pt(0);
    cur.y = start_pt(1);
    cur.z = start_pt(2);
    tar.x = end_pt(0);
    tar.y = end_pt(1);
    tar.z = end_pt(2);

    double distance = (start_pt-end_pt).norm();
    double angle = 1;
    Vector3d path2 = end_pt - start_pt;

    startPtr -> gScore = 0;
    startPtr -> fScore = (w_d*distance + w_v*1/(edf_grid_ptr->getRayMin(cur, tar, 0.3)-0.3) + w_a*exp(-angle))*tie_breaker;
    //startPtr -> fScore = getHeu(startPtr, endPtr, edf_grid_ptr);
    startPtr -> id = 1; //put start node in open set
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) ); //put start in open set

    double tentative_gScore;

    int num_iter = 0;
    while ( !openSet.empty() )
    {
        num_iter ++;
        current = openSet.begin() -> second;
        //cout<<"current: ("<<current->index(0)<<" "<<current->index(1)<<" "<<current->index(2)<<"), fScore: "<<current->fScore<<endl;

        if(current->index(0) == endPtr->index(0)
        && current->index(1) == endPtr->index(1))
        {
            //cout << "total number of iteration used in Astar: " << num_iter  << endl;
            ros::Time time_2 = ros::Time::now();
            if((time_2 - time_1).toSec() > 0.1)
                ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            //cout<<endl;
            //cout<<"current end: "<<current->coord(0)<<" "<<current->coord(2)<<endl;
            //cout<<"real end: "<<endPtr->coord(0)<<" "<<endPtr->coord(2)<<endl;
            gridPath = retrievePath(current);
            return;
        }
        openSet.erase(openSet.begin());
        current -> id = -1; //move current node from open set to closed set.
        expandedNodes.push_back(current);
        for(int dx = -1; dx < 2; dx++)
            for(int dy = -1; dy < 2; dy++)
                {
                    if(dx == 0 && dy == 0)
                        continue;

                    Vector3i neighborIdx;
                    neighborIdx(0) = (current -> index)(0) + dx;
                    neighborIdx(1) = (current -> index)(1) + dy;

                    if(    neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE
                        || neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE)
                        continue;

                    neighborPtr = GridNodeMap[neighborIdx(0)][neighborIdx(1)];

                    if(neighborPtr -> id == -1)
                        continue; //in closed set.

                    double static_cost = sqrt(dx * dx + dy * dy);
                    
                    tentative_gScore = current -> gScore + static_cost; 

                    if(neighborPtr -> id != 1)
                    {
                        // discover a new node
                        neighborPtr -> id = 1;
                        neighborPtr -> cameFrom = current;
                        neighborPtr -> gScore = tentative_gScore;

                        Vector3d nc = gridIndex2coord(neighborPtr->index);
                        Point np;
                        np.x = nc(0);
                        np.y = nc(1);
                        np.z = nc(2);
                        if(edf_grid_ptr->getRayMin(np, tar, 0.3) > 0.3)
                        {
                            Vector3d path1 = neighborPtr->coord - current->coord;
                            angle = path1.dot(path2)/(path1.norm()*path2.norm());
                            distance = (nc - end_pt).norm();
                            neighborPtr -> fScore = neighborPtr -> gScore + (w_v*1/(edf_grid_ptr->getRayMin(np, tar, 0.3)-0.3) + w_d*distance + w_a*exp(-angle))*tie_breaker;
                        }
                        else
                            neighborPtr -> fScore = neighborPtr -> gScore + INF;

                        neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) ); //put neighbor in open set and record it.
                        continue;
                    }
                    else if(tentative_gScore <= neighborPtr-> gScore)
                    {
                        // in open set and need update
                        neighborPtr -> cameFrom = current;
                        neighborPtr -> gScore = tentative_gScore;
                        
                        Vector3d nc = gridIndex2coord(neighborPtr->index);
                        Point np;
                        np.x = nc(0);
                        np.y = nc(1);
                        np.z = nc(2);
                        if(edf_grid_ptr->getRayMin(np, tar, 0.3) > 0.3)
                        {
                            Vector3d path1 = neighborPtr->coord - current->coord;
                            angle = path1.dot(path2)/(path1.norm()*path2.norm());
                            distance = (nc-end_pt).norm();
                            neighborPtr -> fScore = tentative_gScore + (w_v*1/exp(edf_grid_ptr->getRayMin(np, tar, 0.3)-0.3) + w_d*distance + w_a*exp(-angle))*tie_breaker;
                        }
                        else
                            neighborPtr -> fScore = tentative_gScore + INF;

                        openSet.erase(neighborPtr -> nodeMapIt);
                        neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) ); //put neighbor in open set and record it.
                    }
                }
    }

    ros::Time time_2 = ros::Time::now();

    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
}

vector<Vector3d> gridPathFinder::getPath()
{
    vector<Vector3d> path;

    for(auto ptr: gridPath)
        path.push_back(gridIndex2coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}

void gridPathFinder::resetPath()
{
    gridPath.clear();
}
