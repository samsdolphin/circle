#include "auto_chaser/Common.h"
#include "traj_gen/PolyTrajGen.h"

class SmoothPlanner
{
    public:
        // objects
        PathPlanner planner;
        TrajGenOpts option;

        // ros 
        ros::Publisher pub_path; // publihser for path 
        ros::Publisher pub_chasing_corridor; 
        ros::Publisher pub_knots;  
        
        string world_frame_id;
        nav_msgs::Path chasing_smooth_path;
        visualization_msgs::Marker chasing_corridor;
        visualization_msgs::Marker marker_knots;

        SmoothPlanner();
        void init(ros::NodeHandle nh);
        void traj_gen(TimeSeries knots,nav_msgs::Path waypoints,Twist v0,Twist a0);
        void publish();
};