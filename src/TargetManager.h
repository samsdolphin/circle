#ifndef TARGET_MANAGER_H
#define TARGET_MANAGER_H

#include "traj_gen/PolyTrajGen.h"
#include <tf/transform_broadcaster.h>

class TargetManager
{
    private:
        PathPlanner planner;
        string target_frame_id;
        string world_frame_id;
        int mode; // 0(without gazebo), 1(with gazebo), 2(real)
        double min_z; 
        ros::Publisher pub_marker_waypoints;
        ros::Publisher pub_path;
        ros::Subscriber sub_waypoints;
        tf::TransformBroadcaster* br_ptr;
        visualization_msgs::MarkerArray wpnt_markerArray; // waypoints

        // current information after initialization 
        nav_msgs::Path global_path; // path
        nav_msgs::Path waypoints_seq; // waypoints

        TrajGenOpts traj_option; // trajectory generation option 

    public:
        vector<geometry_msgs::PoseStamped> queue; // waypoints for generating the global trajectory 

        // flags 
        bool is_insert_permit = false;
        bool is_path = false; // was path computed
        double cur_spline_eval_time = 0;
        double previous_elapsed = 0; // before publish control button pressed 
        double button_elapsed = 0; // after the button pressed again,

        // member functions
        TargetManager();
        void broadcast_target_tf(double time);
        void callback_waypoint(const geometry_msgs::PoseStampedConstPtr& waypoint);
        void clear_waypoint();
        vector<Point> eval_time_seq(VectorXd ts);
        nav_msgs::Path get_global_waypoints();
        bool global_path_generate(double tf);
        void init(ros::NodeHandle nh);
        void pop_waypoint();
        void queue_file_load(vector<geometry_msgs::PoseStamped>& wpnt_replace);
        void queue_file_load(int,vector<geometry_msgs::PoseStamped>&);
        void session(double t_eval);
};

#endif