#include "auto_chaser/Chaser.h"
#include "auto_chaser/ObjectHandler.h"

class Wrapper
{
    private:
        ros::Publisher pub_control_mav; // publish the control pose for UAV (trajectory_msgs)
        ros::Publisher pub_control_mav_vis; // publishing the control pose visualization(geometry_msgs)
        geometry_msgs::PoseStamped control_pose_mav; // this is used just for coordinate visualization

    public:
        int run_mode; // 0 --> without gazebo 
        ObjectsHandler objects_handler;
        Chaser chaser;

        Wrapper();
        void init(ros::NodeHandle nh);
        void session(double t); // publish things 
        bool trigger_chasing(TimeSeries chasing_knots); //TODO no information given from target manager 
        bool trigger_chasing(vector<Point> target_pt_seq, vector<Twist> target_vel_seq, TimeSeries chasing_knots); // information given from target manager         
        geometry_msgs::PoseStamped get_control_pose(double t_eval); // get the latest control pose 
        
        void pub_control_pose(double t_eval); 
        void pub_control_traj(double t_eval);
};