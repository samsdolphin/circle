#include "auto_chaser/ObjectHandler.h"

ObjectsHandler::ObjectsHandler(ros::NodeHandle nh){};

void ObjectsHandler::init(ros::NodeHandle nh)
{
    // parameters
    nh.param<string>("world_frame_id",this->world_frame_id,"/world");
    nh.param<string>("target_frame_id",this->target_frame_id,"/target__base_footprint");
    nh.param<string>("chaser_frame_id",this->chaser_frame_id,"/firefly/base_link"); 

    // for chaser spawning 
     
    // edf grid param
    nh.param("min_z",min_z,0.4);   
    nh.param("chaser_init_z",chaser_init_z,1.0);             
    nh.param("edf_max_dist",edf_max_dist,2.0);  
    nh.param("edf_max_plot_dist",edf_max_viz_dist,0.5);  
    nh.param("edf_resolution",edf_grid_params.resolution,0.5);  
    nh.param("edf_stride_resolution",edf_grid_params.ray_stride_res,0.3);  
    nh.param("run_mode",run_mode,0);  

    target_pose.header.frame_id = world_frame_id;
    chaser_pose.header.frame_id = world_frame_id;
    markers_edf.header.frame_id = world_frame_id;

    markers_edf.action = visualization_msgs::Marker::ADD;
    markers_edf.type = visualization_msgs::Marker::CUBE_LIST;      
    markers_edf.pose.orientation.x = 0;
    markers_edf.pose.orientation.y = 0;
    markers_edf.pose.orientation.z = 0;
    markers_edf.pose.orientation.w = 1;                  
    markers_edf.scale.x = edf_grid_params.resolution;
    markers_edf.scale.y = edf_grid_params.resolution;
    markers_edf.scale.z = edf_grid_params.resolution;
    
    // topics 
    tf_listener = new (tf::TransformListener);
    tf_talker = new (tf::TransformBroadcaster);

    pub_edf = nh.advertise<visualization_msgs::Marker>("edf_grid",1);

    // octomap
    nh.param("is_octomap_full",this->is_octomap_full,true);
    octree_ptr.reset(new octomap::OcTree(0.1)); // arbitrary init
    if(is_octomap_full)
        sub_octomap = nh.subscribe("/octomap_full",1,&ObjectsHandler::octomap_callback,this);   
    else
        sub_octomap = nh.subscribe("/octomap_binary",1,&ObjectsHandler::octomap_callback,this);   

    sub_chaser_init_pose = nh.subscribe("/chaser_init_pose",1,&ObjectsHandler::callback_chaser_init_pose,this);
    sub_chaser_control_pose = nh.subscribe("mav_pose_desired",1,&ObjectsHandler::callback_chaser_control_pose,this);
    
    cout<<"[ObjectHandler] Object handler initialized"<<endl; 
}

void ObjectsHandler::octomap_callback(const octomap_msgs::Octomap& msg)
{
    // we receive only once from octoamp server
    if(not is_map_recieved)
    {
        // octomap subscribing
        octomap::AbstractOcTree* octree;

        if(is_octomap_full)
            octree = octomap_msgs::fullMsgToMap(msg);
        else
            octree = octomap_msgs::binaryMsgToMap(msg);

        this->octree_ptr.reset((dynamic_cast<octomap::OcTree*>(octree)));

        cout<<"[ObjectHandler] octomap received"<<endl;
        double x, y, z;
        octree_ptr.get()->getMetricMin(x, y, z);
        octomap::point3d boundary_min(x, y, z); 
        boundary_min.z() = min_z;
        octree_ptr.get()->getMetricMax(x, y, z);
        octomap::point3d boundary_max(x, y, z); 
        bool unknownAsOccupied = false;

        ros::Time begin = ros::Time::now();
        edf_ptr = new DynamicEDTOctomap(edf_max_dist, octree_ptr.get(), boundary_min, boundary_max, unknownAsOccupied); // copy occupancy data
        edf_ptr->update(); // compute distance map
        ros::Time end = ros::Time::now();
        cout<<"[ObjectHandler] dynamic EDT computed in "<<(end-begin).toSec()<<"s"<<endl;

        // generate homogenous grid 
        edf_grid_params.x0 = boundary_min.x();
        edf_grid_params.y0 = boundary_min.y();
        edf_grid_params.z0 = min_z;
        edf_grid_params.lx = boundary_max.x() - boundary_min.x();
        edf_grid_params.ly = boundary_max.y() - boundary_min.y();
        edf_grid_params.lz = boundary_max.z() - min_z;
        edf_grid_ptr.reset(new GridField(edf_grid_params));
        compute_edf();

        is_map_recieved = true;
    }
};

// retrive 
PoseStamped ObjectsHandler::get_target_pose()
{
    PoseStamped pose(target_pose); 
    pose.pose.position.z = min_z; 
    return pose;
};

Twist ObjectsHandler::get_chaser_velocity() {return chaser_vel;};
Twist ObjectsHandler::get_chaser_acceleration() {return chaser_acc;};
string ObjectsHandler::get_world_frame_id() {return world_frame_id;};
GridField* ObjectsHandler::get_edf_grid_ptr() {return edf_grid_ptr.get();}; // returns the stored pointer
PoseStamped ObjectsHandler::get_chaser_pose() {return chaser_pose;};
octomap::OcTree* ObjectsHandler::get_octree_obj_ptr() {return octree_ptr.get();};

// callback 
void ObjectsHandler::tf_update()
{    
    if(run_mode == 1)
    {
        // mode 1 : gazebo simulation mode 
        // chaser(from gazebo) and target(from target manager) to be listened.  
        string objects_frame_id[2];
        objects_frame_id[0] = target_frame_id;
        objects_frame_id[1] = chaser_frame_id;
        
        for(int i=0;i<2;i++)
        {
            tf::StampedTransform transform;    
            // 
            try
            {
                tf_listener->lookupTransform(world_frame_id,objects_frame_id[i],ros::Time(0), transform);
                PoseStamped pose_stamped;
                pose_stamped.header.stamp = ros::Time::now();
                pose_stamped.header.frame_id = world_frame_id;

                pose_stamped.pose.position.x = transform.getOrigin().getX();
                pose_stamped.pose.position.y = transform.getOrigin().getY();
                pose_stamped.pose.position.z = transform.getOrigin().getZ();

                pose_stamped.pose.orientation.x = transform.getRotation().getX();
                pose_stamped.pose.orientation.y = transform.getRotation().getY();
                pose_stamped.pose.orientation.z = transform.getRotation().getZ();
                pose_stamped.pose.orientation.w = transform.getRotation().getW();

                if(i==0)
                    {ROS_INFO_ONCE("[ObjectsHandler] tf of target received. "); is_target_recieved = true; target_pose = pose_stamped;} 
                else
                    {ROS_INFO_ONCE("[ObjectsHandler] tf of chaser received. "); is_chaser_recieved = true; chaser_pose = pose_stamped;}  
            }
            catch (tf::TransformException ex)
            {
                if(i==0)
                    ROS_ERROR_ONCE("[ObjectsHandler] tf of target does not exist. ",ex.what());  
                else
                    ROS_ERROR_ONCE("[ObjectsHandler] tf of chaser does not exist. ",ex.what());  
            }
        }
    }
    else
    {
        // mode 0 : no gazebo mode  
        // target to be listened and chaser to be broadcast  

        // 1) target tf listen from target manager  
        tf::StampedTransform transform;    
        try
        {
            tf_listener->lookupTransform(world_frame_id, target_frame_id, ros::Time(0), transform);
            PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = world_frame_id;

            pose_stamped.pose.position.x = transform.getOrigin().getX();
            pose_stamped.pose.position.y = transform.getOrigin().getY();
            pose_stamped.pose.position.z = transform.getOrigin().getZ();

            pose_stamped.pose.orientation.x = transform.getRotation().getX();
            pose_stamped.pose.orientation.y = transform.getRotation().getY();
            pose_stamped.pose.orientation.z = transform.getRotation().getZ();
            pose_stamped.pose.orientation.w = transform.getRotation().getW();        

            ROS_INFO_ONCE("[ObjectsHandler] tf of target received.");
            is_target_recieved = true;
            target_pose = pose_stamped;
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR_ONCE("[ObjectsHandler] tf of target does not exist.",ex.what());  
        }    
            
        if(is_chaser_spawned)
        {
            // 2) chaser tf broadcasting
            tf::Quaternion q;
            q.setX(chaser_pose.pose.orientation.x);
            q.setY(chaser_pose.pose.orientation.y);
            q.setZ(chaser_pose.pose.orientation.z);
            q.setW(chaser_pose.pose.orientation.w);
            
            transform.setOrigin(tf::Vector3(chaser_pose.pose.position.x,chaser_pose.pose.position.y,chaser_pose.pose.position.z));
            transform.setRotation(q);
            tf_talker->sendTransform(tf::StampedTransform(transform,ros::Time::now(),world_frame_id,chaser_frame_id));        
        }
    }
}

void ObjectsHandler::compute_edf()
{
    for(int ix=0; ix<edf_grid_ptr.get()->Nx; ix++)
        for(int iy=0; iy<edf_grid_ptr.get()->Ny; iy++)
            for(int iz=0; iz<edf_grid_ptr.get()->Nz; iz++)
            {
                Point eval_pnt = edf_grid_ptr.get()->getCellPnt(Vector3i(ix, iy, iz));  
                // query edf value from edf mapper                       
                float dist_val = edf_ptr->getDistance(octomap::point3d(eval_pnt.x, eval_pnt.y, eval_pnt.z));

                // edf value assign to homogenous grid  
                edf_grid_ptr.get()->field_vals[ix][iy][iz] = dist_val;

                // marker generation
                if(dist_val < edf_max_viz_dist)
                {
                    // color 
                    std_msgs::ColorRGBA color;                    
                    get_color_dist(dist_val, color, edf_max_viz_dist);

                    // marker 
                    markers_edf.points.push_back(eval_pnt);
                    markers_edf.colors.push_back(color);                    
                }
            }    
}

void ObjectsHandler::publish() {pub_edf.publish(markers_edf);}

void ObjectsHandler::chaser_spawn(PoseStamped spawn_pose)
{
    ROS_INFO_ONCE("[ObjectsHandler] spawning chaser"); 
    
    is_chaser_recieved = true;
    is_chaser_spawned = true;    
    
    if(run_mode == 0)
    { // without gazebo : update chaser pose
        chaser_pose = spawn_pose;
        chaser_pose.pose.position.z = chaser_init_z;
    }
    else
    { // with gazebo : nothing happen 
        ROS_WARN("[ObjectsHandler] gazebo mode. No virtual spawning happens");        
    }
}

void ObjectsHandler::callback_chaser_init_pose(const geometry_msgs::PoseStampedConstPtr& chaser_init_pose)
{
    chaser_spawn(*chaser_init_pose);    
}

/*
    Callback function for control pose from wrapper.
    This is intended to replace the currnet chaser pose directly with desired pose
*/
void ObjectsHandler::callback_chaser_control_pose(const geometry_msgs::PoseStampedConstPtr& chaser_control_pose)
{
    if(run_mode == 0 and is_path_solved) // is path solved in Wrapper::trigger_chasing
        chaser_pose = *chaser_control_pose;
}

vector<Point> ObjectsHandler::get_prediction_seq() {}