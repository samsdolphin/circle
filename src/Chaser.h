#include "auto_chaser/Preplanner.h"
#include "auto_chaser/SmoothPlanner.h"

class Chaser
{
    private:
        Preplanner preplanner;
        SmoothPlanner smooth_planner;
        // we will get them from parameter handle
        double spawn_x, spawn_y, hovering_z;

        // subroutines
        void preplan();
        void path_complete();

    public:
        bool is_complete_chasing_path; // is there any complete chasing path

        Chaser();
        void init(ros::NodeHandle nh);
        bool chase_update(GridField* edf_grid_ptr,
                          vector<Point> target_pnts,
                          vector<Twist> target_vels,
                          Point chaser_x0,
                          Twist chaser_v0,
                          Twist chaser_a0,
                          TimeSeries knots); // load control point 
        void session(double t); // publish information 

        // evaluation with current 
        Point eval_point(double t_eval);
        Twist eval_velocity(double t_eval);
        Twist eval_acceleration(double t_eval);
        Point get_control_point(double t_eval); // get the current control point 
};