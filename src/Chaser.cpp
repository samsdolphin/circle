#include "auto_chaser/Chaser.h"

Chaser::Chaser():is_complete_chasing_path(false){}

void Chaser::init(ros::NodeHandle nh)
{
    preplanner.init(nh);
    smooth_planner.init(nh);

    // retreieve initial hovering command 
    nh.param("chaser_init_x",spawn_x,0.0);
    nh.param("chaser_init_y",spawn_y,0.0);
    nh.param("chaser_init_z",hovering_z,1.0);
}

bool Chaser::chase_update(GridField* edf_grid_ptr,
                          vector<Point> target_pnts,
                          vector<Twist> target_vels,
                          Point chaser_x0,
                          Twist chaser_v0,
                          Twist chaser_a0,
                          TimeSeries knots)
{    
    bool result = false;
    
    /*--- PHASE 1 PRE-PLANNING ---*/
    ros::Time begin = ros::Time::now();
    preplanner.preplan(edf_grid_ptr, target_pnts, target_vels, chaser_x0);
    ros::Time end = ros::Time::now();
    cout<<endl;
    cout<<"[Chaser] preplanning completed in "<<(end-begin).toSec()*1000<<"ms"<<endl;
    nav_msgs::Path waypoints = preplanner.get_preplanned_waypoints();

    if(waypoints.poses.size())
    {
        /*--- PHASE 2 SMOOTH-PLANNING ---*/
        begin = ros::Time::now();
        smooth_planner.traj_gen(knots, waypoints, chaser_v0, chaser_a0);
        if(smooth_planner.planner.is_spline_valid())
        {
            end = ros::Time::now();
            cout<<"[Chaser] smooth path completed in "<<(end-begin).toSec()*1000<<"ms"<<endl;
            is_complete_chasing_path = true;
            return true;
        }
        else 
            ROS_ERROR("[Chaser] smooth path incompleted.");
    }
    else
        ROS_ERROR("[Chaser] preplanning failed");

    return false;
}

void Chaser::session(double t)
{
    preplanner.publish(); // publish markers vsf_seq, waypoints, preplanned path
    smooth_planner.publish(); // publish chasing smooth path, corridor, knots
}

Point Chaser::eval_point(double t_eval)
{    
    return smooth_planner.planner.point_eval_spline(t_eval);        
}

Twist Chaser::eval_velocity(double t_eval)
{
    return smooth_planner.planner.vel_eval_spline(t_eval);        
}

Twist Chaser::eval_acceleration(double t_eval)
{
    return smooth_planner.planner.accel_eval_spline(t_eval);        
}

/**
 * @brief obtains the latest control point. yaw will be selected from wrapper with information of target   
 * 
 * @param t_eval evaluation time 
 * @return Point the control point 
 */
Point Chaser::get_control_point(double t_eval)
{
    if(this->is_complete_chasing_path)
        return smooth_planner.planner.point_eval_spline(t_eval); 
    else
    {
        // hovering command at the spawning position with desired height      
        Point hovering_point;
        hovering_point.x = spawn_x;
        hovering_point.y = spawn_y;
        hovering_point.z = hovering_z;        
        return hovering_point;
    }
}