#ifndef OBJ_HANDLE_H
#define OBJ_HANDLE_H

#include "auto_chaser/Common.h"

class ObjectsHandler
{
    private:
        // ROS
        ros::Publisher pub_edf;
        ros::Subscriber sub_octomap;
        ros::Subscriber sub_chaser_init_pose; // user input from rviz
        ros::Subscriber sub_chaser_control_pose; // active only run mode 0
        ros::Subscriber sub_chaser;
        
        tf::TransformListener* tf_listener; // don't have initial copy constructor 
        tf::TransformBroadcaster* tf_talker; // chaser tf broadcast in mode 0

        visualization_msgs::Marker markers_edf; // Euclidean distance field
        visualization_msgs::Marker chaser_traj;

        // id         
        string world_frame_id;
        string chaser_frame_id;
        string target_frame_id;

        // topic
        string octomap_topic_name;    

        // objects
        Twist chaser_vel;
        Twist chaser_acc;
        PoseStamped target_pose;
        PoseStamped chaser_pose;

        DynamicEDTOctomap *edf_ptr;
        shared_ptr<GridField> edf_grid_ptr; // signed distance field
        shared_ptr<octomap::OcTree> octree_ptr;

        // parameter 
        double min_z; // the minimum height to be clamped  
        double chaser_init_z; // initial target hovering height 
        double edf_max_viz_dist;
        double edf_max_dist;
        int run_mode; // 0 (simulation without gazebo), 1 (simulation with gazebo)
        FieldParams edf_grid_params;  

    public:
        // flag
        bool is_octomap_full = false;
        bool is_chaser_recieved = false;
        bool is_map_recieved = false;        
        bool is_target_recieved = false;
        bool is_control_received = false;
        bool is_chaser_spawned = false;
        bool is_insert_permit = false;
        
        // this is added. The object_handler should also be able to distinguish whether the subscribed control pose is for hovering         
        // if the control pose was for hovering, Then it should be ignored. Instead the object_handler should trust the spawn position from user   
        bool is_path_solved = false;
        
        ObjectsHandler(){};
        ObjectsHandler(ros::NodeHandle nh);

        void init(ros::NodeHandle nh);
        void compute_edf();

        Twist get_chaser_velocity();
        Twist get_chaser_acceleration();
        string get_world_frame_id();
        GridField* get_edf_grid_ptr();
        PoseStamped get_target_pose();
        PoseStamped get_chaser_pose();
        octomap::OcTree* get_octree_obj_ptr();
        
        void octomap_callback(const octomap_msgs::Octomap& msg);
        void chaser_spawn(PoseStamped spawn_pose);
        void callback_chaser_init_pose(const geometry_msgs::PoseStampedConstPtr& chaser_init_pose);
        void callback_chaser_control_pose(const geometry_msgs::PoseStampedConstPtr& chaser_control_pose);
        void callback_chaser(const visualization_msgs::Marker& traj);
        void tf_update();
        void publish();
        vector<Point> get_prediction_seq();
};

#endif