#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "a_star/grid_cube_type.h"
#include "auto_chaser/Common.h"

class gridPathFinder
{
	private:
		
		double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
		double getManhHeu(GridNodePtr node1, GridNodePtr node2);
		double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
		double getHeu(GridNodePtr node1, GridNodePtr node2, GridField* edf_grid_ptr);

		std::vector<GridNodePtr> retrievePath(GridNodePtr current);

		double resolution, inv_resolution;
		double gl_xl, gl_yl;
		double gl_xu, gl_yu;
		double tie_breaker = 1.0 + 1.0 / 10000;

		std::vector<GridNodePtr> expandedNodes;
		std::vector<GridNodePtr> gridPath;

		std::vector<GridNodePtr> endPtrList;
    	std::vector<double> globalHeuList;

		int tmp_id_x, tmp_id_y, tmp_id_z;
		int GLX_SIZE, GLY_SIZE;
		int GLXY_SIZE;

		//uint8_t * data;

		GridNodePtr **GridNodeMap;
		std::multimap<double, GridNodePtr> openSet;

	public:
		gridPathFinder(int max_x_id, int max_y_id)
		{
			GLX_SIZE = max_x_id;
			GLY_SIZE = max_y_id;
			GLXY_SIZE  = GLY_SIZE * GLX_SIZE;
		};

		gridPathFinder(){};
		~gridPathFinder(){};

		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u);
		void AstarSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, GridField* edf_grid_ptr);

		void resetMap();
		void resetPath();

		std::vector<Eigen::Vector3d> getPath();

		GridNodePtr getGridNode(Eigen::Vector3d coord)
		{
			Eigen::Vector3i index = coord2gridIndex(coord);			
			return GridNodeMap[index(0)][index(1)];
		}

		GridNodePtr getGridNode(double x, double y, double z)
		{
			Eigen::Vector3d coord(x, y, z);
			Eigen::Vector3i index = coord2gridIndex(coord);			
			return GridNodeMap[index(0)][index(1)];
		}

		GridNodePtr getGridNode(int id_x, int id_y, int id_z)
		{
			return GridNodeMap[id_x][id_y];
		}

		GridNodePtr getGridNode(Eigen::Vector3i index)
		{
			return GridNodeMap[index(0)][index(1)];
		}

		inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i index) const
		{
		    Eigen::Vector3d pt;

		    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
		    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
		    pt(2) = 0.4;

		    return pt;
		};

		inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d pt) const
		{
		    Eigen::Vector3i idx;
		    idx <<  std::min( std::max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
		            std::min( std::max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
		            0;
		  
		    return idx;
		};
};
