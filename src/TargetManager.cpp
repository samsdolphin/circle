#include "target_manager/TargetManager.h"

TargetManager::TargetManager(){}

void TargetManager::init(ros::NodeHandle nh)
{
    // paramter parsing for option 
    nh.param<string>("world_frame_id",world_frame_id,"/world");
    nh.param<string>("target_frame_id",target_frame_id,"/target");
    nh.param<double>("target/safe_corridor_r",traj_option.safe_r,0.2);
    nh.param<int>("target/N_safe_pnts",traj_option.N_safe_pnts,2);
    nh.param<int>("target/objective_derivative",traj_option.objective_derivative,3);
    nh.param<int>("target/poly_order",traj_option.poly_order,6);
    nh.param<double>("target/w_deviation",traj_option.w_d,0.005);
    nh.param<bool>("target/is_waypoint_soft",traj_option.is_waypoint_soft,false);
    
    nh.param("min_z",min_z,0.4);   

    // register 
    pub_marker_waypoints = nh.advertise<visualization_msgs::MarkerArray>("target_waypoints",1);
    sub_waypoints = nh.subscribe("/target_waypoints",1,&TargetManager::callback_waypoint,this);
    pub_path = nh.advertise<nav_msgs::Path>("target_global_path",1);
    br_ptr = new tf::TransformBroadcaster();
}

void TargetManager::callback_waypoint(const geometry_msgs::PoseStampedConstPtr &pose)
{
    if(is_insert_permit)
    {
        ROS_INFO("[TargetManager] point received");
        queue.push_back(*pose);
        visualization_msgs::Marker marker;
        
        marker.action = 0;
        marker.header.frame_id = world_frame_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose = pose->pose;
        float scale = 0.1;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.r = 1;
        marker.color.a = 1;
        marker.id = queue.size();
        wpnt_markerArray.markers.push_back(marker);
        
        // lets print in board  
        string line = "[TargetManager] recieved point: " + to_string(pose->pose.position.x) + " , " + to_string(pose->pose.position.y);
        ROS_INFO(line.c_str());        
    }
    else
    {
        ROS_WARN("[TargetManager] insertion not allowed");
        // std::cout<<"insertion not allowed"<<std::endl;
    }
}

bool TargetManager::global_path_generate(double total_time)
{
    nav_msgs::Path waypoints;

    waypoints.poses = queue;
    TimeSeries knots(queue.size());
    if(queue.empty())
    {
        ROS_ERROR("[TargetManager] target waypoints empty");
        return false;
    }
    // waypoints update 
    waypoints_seq = waypoints;
    knots.setLinSpaced(queue.size(), 0, total_time);
    planner.path_gen(knots, waypoints, geometry_msgs::Twist(), geometry_msgs::Twist(), traj_option); 
    if(planner.is_spline_valid())
    {
        is_path = true; // is global target path generated
        global_path = planner.get_path();
        global_path.header.frame_id = world_frame_id;
        cout<<"[TargetManager] global path obtained"<<endl;    
        return true;
    }
    else
    {
        cout<<"[TargetManager] path generatoin failed."<<endl;  
        return false; 
    }
}

void TargetManager::session(double sim_time)
{
    pub_marker_waypoints.publish(wpnt_markerArray); // publish target waypoint markers
    if(is_path) // is global target path generated
    {
        pub_path.publish(global_path);
        broadcast_target_tf(sim_time);
    }
}

void TargetManager::clear_waypoint()
{
    queue.clear();
    wpnt_markerArray.markers.clear();
    ROS_INFO("[TargetManager] queue cleared ");    
}

void TargetManager::pop_waypoint()
{
    queue.pop_back();
    wpnt_markerArray.markers.pop_back();
    ROS_INFO("[TargetManager] queue pop ");    
}

void TargetManager::broadcast_target_tf(double sim_time)
{
    tf::TransformBroadcaster br; 

    Point sim_point = planner.point_eval_spline(sim_time);
    // Twist eval_vel = planner.vel_eval_spline(sim_time);

    tf::Transform transform;
    // float target_yaw = atan2(eval_vel.linear.y,eval_vel.linear.x);
    tf::Quaternion q;
    q.setRPY(0, 0, 0);

    transform.setOrigin(tf::Vector3(sim_point.x, sim_point.y, sim_point.z));
    transform.setRotation(q);
    br_ptr->sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame_id, target_frame_id));
}

void TargetManager::queue_file_load(vector<geometry_msgs::PoseStamped>& waypoint_queue)
{
    this->queue = waypoint_queue; // put waypoint_queue --> queue
    wpnt_markerArray.markers.clear();
   
    for(auto it=waypoint_queue.begin(); it<waypoint_queue.end(); it++)
    {    
        visualization_msgs::Marker marker;

        marker.action = 0;
        marker.header.frame_id  = world_frame_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.pose = it->pose;

        //std::cout<< it->pose.position.x <<"\t"<< it->pose.position.y <<"\t"<<it->pose.position.z<<std::endl;

        float scale = 0.1;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.r = 1;
        marker.color.b = 1;

        marker.color.a = 1;
        marker.id = wpnt_markerArray.markers.size();
        wpnt_markerArray.markers.push_back(marker);
    }

    std::cout<<"[TargetManager] target waypoints loaded"<<endl;
}

nav_msgs::Path TargetManager::get_global_waypoints()
{
    // let's process the heights of target 
    for(auto it=waypoints_seq.poses.begin(); it<waypoints_seq.poses.end(); it++)
        it->pose.position.z = min_z + 0.001;
        
    return waypoints_seq;
}

vector<Point> TargetManager::eval_time_seq(VectorXd ts)
{
    vector<Point> point_seq;
    
    for (int i=0; i<ts.size(); i++)
    {
        Point temp_p = planner.point_eval_spline(ts(i));
        point_seq.push_back(temp_p);
        
        //ROS_INFO("[TargetManager] eval_point %f, %f, %f", temp_p.x, temp_p.y, temp_p.z);
    }

    return point_seq;
}

vector<Twist> TargetManager::eval_vel_seq(VectorXd ts)
{
    vector<Twist> twist_seq;
    
    for (int i=0; i<ts.size(); i++)
    {
        Twist temp_v = planner.vel_eval_spline(ts(i));
        twist_seq.push_back(temp_v);
        //ROS_INFO("[TargetManager] eval_twist %f, %f, %f", temp_v.linear.x, temp_v.linear.y, temp_v.linear.z);
    }

    return twist_seq;
}