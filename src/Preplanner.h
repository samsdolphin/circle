#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

#include <ooqp/QpGenData.h>
#include <ooqp/QpGenVars.h>
#include <ooqp/QpGenResiduals.h>
#include <ooqp/GondzioSolver.h>
#include <ooqp/QpGenSparseMa27.h>

#include "auto_chaser/Common.h"
#include "a_star/a_star.h"

class Preplanner
{
    private:
        // params
        string world_frame_id;
        chaser::PreplannerParams params;

        // grid fields
        Graph di_graph; // directed graph for preplanning
        DescriptorMap descriptor_map;
        vector<shared_ptr<GridField>> vsf_field_ptr_seq; // visibility score field sequence

        // ROS
        ros::Publisher pub_vsf_vis; // vsf field pub
        ros::Publisher pub_preplanned_path;
        ros::Publisher pub_waypoints;
        ros::Publisher pub_chaser;
        ros::Publisher pub_chaser_traj;

        // markers
        visualization_msgs::Marker marker_wpnts; // waypoints of preplanned path
        visualization_msgs::Marker markers_visibility_field_base;
        visualization_msgs::MarkerArray markers_visibility_field_seq;
        visualization_msgs::Marker chaser_traj;

        // path 
        nav_msgs::Path preplanned_path; // for visualization and path completion
        nav_msgs::Path chaser_path; // planned chaser path waypoints

        list<pair<double, Vector3d>> tar_obs;

        // Graph
        void graph_construct(GridField* global_edf,Point x0); // complete di_graph from vsf seq
        void compute_visibility_field_seq(GridField* global_edf, vector<Point> target_pnts, vector<Twist> target_vels); // local vsf seq generation 
        void compute_shortest_path();
        VertexPath dijkstra(Vertex v0, Vertex vf);
        FieldParams get_local_vsf_param_around_target(Point target_pnt, Twist target_vel); // generate grid field based on the preplanning params

    public:
        Preplanner();
        void init(ros::NodeHandle nh);
        void preplan(GridField* global_edf_ptr, vector<Point> target_pnts, vector<Twist> target_vels, Point chaser_init);
        void publish();
        VectorXd estimate_T(const list<pair<double, Vector3d>> &tar_ob);
        nav_msgs::Path get_preplanned_waypoints();
};