#include "auto_chaser/Preplanner.h"

Vector3d chase_start_point(0, 0, 0);
double clength = 0.0;

Preplanner::Preplanner(){};

void Preplanner::init(ros::NodeHandle nh)
{
    // preplanner params parsing 
    nh.param("w_v",params.w_v,5.0);       
    nh.param("w_d",params.w_d,1.5);            
    nh.param("r_safe",params.r_safe,0.3);
    nh.param("min_z",params.min_z,0.4);
    nh.param("vs_min",params.vs_min,0.2);
    nh.param("vsf_resolution",params.vsf_resolution,0.25);
    nh.param("d_connect_max",params.d_connect_max,3.0);

    nh.param("max_tracking_distance",params.d_trakcing_max,4.0);
    nh.param("min_tracking_distance",params.d_trakcing_min,1.0);
    nh.param("des_tracking_distance",params.d_trakcing_des,1.0);
    nh.param("max_azim",params.max_azim,(3.141592/4));
    nh.param("min_azim",params.min_azim,(3.141592/7));

    // world_frame_id 
    nh.param<string>("world_frame_id",world_frame_id,"/world");
    nh.param<string>("world_frame_id",markers_visibility_field_base.header.frame_id,"/world");
    nh.param<string>("world_frame_id",preplanned_path.header.frame_id,"/world");
    nh.param<string>("world_frame_id",chaser_path.header.frame_id,"/world");
    //nh.param<string>("world_frame_id",chaser_traj.header.frame_id,"/world");

    //
    chaser_traj.header.frame_id = markers_visibility_field_base.header.frame_id;
    chaser_traj.action = visualization_msgs::Marker::ADD;
    chaser_traj.pose.orientation.w = 1.0;
    chaser_traj.id = 1;
    chaser_traj.type = visualization_msgs::Marker::LINE_STRIP;
    chaser_traj.scale.x = 0.05;
    chaser_traj.scale.y = 0.05;
    chaser_traj.scale.z = 0.05;
    chaser_traj.color.r = chaser_traj.color.a = 1.0;

    // waypoints 
    marker_wpnts.header.frame_id = markers_visibility_field_base.header.frame_id;
    marker_wpnts.ns = "waypoints";
    marker_wpnts.id = 0;
    marker_wpnts.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_wpnts.color.r = 14.0/255.0;
    marker_wpnts.color.g = 50.0/255.0;
    marker_wpnts.color.b = 1.0;
    marker_wpnts.color.a = 0.3;
    marker_wpnts.pose.orientation.w = 1.0;
    double scale = 0.08;
    marker_wpnts.scale.x = scale;
    marker_wpnts.scale.y = scale;
    marker_wpnts.scale.z = scale;    

    // marker base
    visualization_msgs::Marker marker;
    marker.header.frame_id = markers_visibility_field_base.header.frame_id;;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE_LIST;      
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;                  
    marker.scale.x = params.vsf_resolution;
    marker.scale.y = params.vsf_resolution;
    marker.scale.z = params.vsf_resolution;
    markers_visibility_field_base = marker;

    // ros initialize 
    pub_vsf_vis = nh.advertise<visualization_msgs::MarkerArray>("vsf_grid_seq",1);
    pub_waypoints = nh.advertise<visualization_msgs::Marker>("preplanned_waypoints",1);
    pub_preplanned_path = nh.advertise<nav_msgs::Path>("preplanned_path",1);
    pub_chaser = nh.advertise<nav_msgs::Path>("chaser_planned_path",1);
    pub_chaser_traj = nh.advertise<visualization_msgs::Marker>("traj_viz", 1);
};

FieldParams Preplanner::get_local_vsf_param_around_target(Point target_pnt, Twist target_vel)
{
    FieldParams vsf_param;    
    double lx,ly,lz;
    lx = ly = 4*params.d_trakcing_max ;
    lz = params.d_trakcing_max * sin(params.max_azim) - params.d_trakcing_min * sin(params.min_azim) ;

    vsf_param.x0 = target_pnt.x - lx/2;
    vsf_param.y0 = target_pnt.y - ly/2;
    vsf_param.z0 = target_pnt.z;
    vsf_param.lx = lx;
    vsf_param.ly = ly;
    vsf_param.lz = lz;
    
    vsf_param.resolution = params.vsf_resolution;
    vsf_param.ray_stride_res =  params.vsf_resolution; // not used for vsf grid 

    return vsf_param;
};

void Preplanner::compute_visibility_field_seq(GridField* edf_grid_ptr, vector<Point> target_pnts, vector<Twist> target_vels)
{
    vsf_field_ptr_seq.resize(target_pnts.size());
    float numeric_threshold = 1e-2;
    int t = 1;
    float max_score = -1;  // for visualization 
    // for each target pnt
    for(auto it=target_pnts.begin(); it<target_pnts.end(); it++, t++)
    {
        // get local conservative grid map around the current target point
        //int VSF_MODE = 1;
        try
        {
            vsf_field_ptr_seq[t-1].reset(new GridField(get_local_vsf_param_around_target(*it, target_vels[t-1]))); 
        }
        catch(bool ex)
        {
            ROS_ERROR("failed to make visibility scoric field");
        }
        // field value update with edf grid 
        for(int ix=0; ix<vsf_field_ptr_seq[t-1].get()->Nx; ix++)
            for(int iy=0; iy<vsf_field_ptr_seq[t-1].get()->Ny; iy++)
                for(int iz=0; iz<vsf_field_ptr_seq[t-1].get()->Nz; iz++)
                {
                    // assign visibilty value with minimum clamping to evaluated node 
                    Point eval_pnt = vsf_field_ptr_seq[t-1].get()->getCellPnt(Vector3i(ix, iy, iz));      
                    float vs = edf_grid_ptr->getRayMin(*it, eval_pnt, params.vs_min); // visibility score from distance field                    
                    vsf_field_ptr_seq[t-1].get()->field_vals[ix][iy][iz] = vs;

                    // let's save the point if certain condition is satified (for graph construction)                
                    Vector3i pnt_idx_in_edf = edf_grid_ptr->getCellIdx(eval_pnt);
                    float edf_val = edf_grid_ptr->field_vals[pnt_idx_in_edf(0)][pnt_idx_in_edf(1)][pnt_idx_in_edf(2)];  
                    Vector3f bearing_vec = (geo2eigen(eval_pnt) - geo2eigen(*it)); 
                    float relative_dist = bearing_vec.norm();                      
                    float azim = atan2(bearing_vec(2), Vector2f(bearing_vec(0), bearing_vec(1)).norm());
                    
                    if(edf_val > params.r_safe && // safe
                       relative_dist > params.d_trakcing_min && // tracking spec
                       relative_dist < params.d_trakcing_max && // tracking spec
                       vs > params.vs_min + numeric_threshold  && // non-occlusion
                       azim < params.max_azim) // tracking spec
                        vsf_field_ptr_seq[t-1].get()->saved_points.push_back(eval_pnt);
                    
                    if(vs >= max_score)
                        max_score = vs;
                }
    }
    cout<<"[Preplanner] nodes at each time are: ";
    for(int i=0; i<4; i++)
        cout<<vsf_field_ptr_seq[i].get()->saved_points.size()<<" ";
    cout<<endl;

    // save the markers

    // marker initialization     
    markers_visibility_field_seq.markers.clear();    
    markers_visibility_field_base.header.stamp = ros::Time::now();
    markers_visibility_field_base.header.frame_id = world_frame_id;
    markers_visibility_field_base.points.clear();
    markers_visibility_field_base.colors.clear();
    t = 1;

    for(auto it=target_pnts.begin(); it<target_pnts.end(); it++,t++)
    {
        // for time
        markers_visibility_field_base.ns = "time_"+to_string(t);

        // we draw only saved points from above 
        for(auto it_node=vsf_field_ptr_seq[t-1].get()->saved_points.begin();
            it_node<vsf_field_ptr_seq[t-1].get()->saved_points.end();
            it_node++)
        {
            Vector3i key = vsf_field_ptr_seq[t-1].get()->getCellIdx(*it_node);
            float vs = vsf_field_ptr_seq[t-1].get()->field_vals[key(0)][key(1)][key(2)];
            // cout<<vs<<endl;
            // marker update
            std_msgs::ColorRGBA color;
            get_color((vs-params.vs_min)/(max_score-params.vs_min), color.r, color.g, color.b);            
            color.a = 0.1;

            markers_visibility_field_base.colors.push_back(color);
            markers_visibility_field_base.points.push_back(*it_node);
        }

        markers_visibility_field_seq.markers.push_back(markers_visibility_field_base);
        markers_visibility_field_base.points.clear();
        markers_visibility_field_base.colors.clear();
    }
}

void Preplanner::graph_construct(GridField* edf_grid_ptr, Point x0)
{    
    // init graph with the initial position of chaser
    di_graph = Graph();
    descriptor_map.clear();
    WeightMap weightmap = get(boost::edge_weight, di_graph);
    
    vector<Node<Point>> prev_layer, useful_layer;
    Node<Point> initial_node;
    initial_node.value = x0;
    initial_node.name = "x0";
    prev_layer.push_back(initial_node);
    
    Vertex v0 = boost::add_vertex(x0, di_graph); // Point x0 is NamePorperty of Graph
    descriptor_map.insert(make_pair(VertexName("x0"), v0));

    int H = vsf_field_ptr_seq.size(); // total prediction horizon
    int N_edge = 0;
    int N_edge_sub = 0;
    bool is_push = 0;
    Edge e;
    bool inserted;

    // in case of t = 0, we don't need (just current step). 
    for(int t=1; t<H; t++)
    {
        N_edge_sub = 0;
        GridField* cur_vsf_ptr = vsf_field_ptr_seq[t].get();        
        vector<Node<Point>> cur_layer = cur_vsf_ptr->generate_node(t); // generate vector<Node<Point>> from vsf_ptr
        
        for(auto it_cur=cur_layer.begin(); it_cur<cur_layer.end(); it_cur++)
        {
            // step 1:  let's register the node(pnt, name) in the current layer into graph 
            Point cur_pnt = it_cur->value;
            Vector3f cur_vec = geo2eigen(cur_pnt);
            Vertex cur_vert = boost::add_vertex(cur_pnt, di_graph); // add vertex with vertex_property_type
            descriptor_map.insert(make_pair(it_cur->name, cur_vert));
            
            // call the previous layer
            GridField* prev_vsf_ptr = vsf_field_ptr_seq[t-1].get();
            // step 2: let's connect with previous layer and add edges 
            for(auto it_prev=prev_layer.begin(); it_prev<prev_layer.end(); it_prev++)
            {
                Vertex prev_vert = descriptor_map[it_prev->name];
                Point prev_pnt = it_prev->value;
                Vector3f prev_vec = geo2eigen(prev_pnt);

                // this condition should be satisfied to be connected 
                if(((cur_vec-prev_vec).norm() < params.d_connect_max) && (edf_grid_ptr->getRayMin(cur_pnt, prev_pnt, 0) > params.r_safe))
                {
                    float weight = (cur_vec-prev_vec).norm()
                                   + params.w_v * 1/sqrt(cur_vsf_ptr->getRayMean(cur_pnt, prev_pnt) * prev_vsf_ptr->getRayMean(prev_pnt, cur_pnt))
                                   + params.w_d * pow(((geo2eigen(cur_vsf_ptr->getCentre()) - cur_vec).norm() - params.d_trakcing_des), 2);                     
                    //boost::add_edge(prev_vert, cur_vert, weight, di_graph);
                    //ROS_INFO_ONCE("cur: %f, pre: %f", cur_vsf_ptr->getRayMean(cur_pnt, prev_pnt), prev_vsf_ptr->getRayMean(prev_pnt, cur_pnt));
                    boost::tie(e, inserted) = boost::add_edge(prev_vert, cur_vert, di_graph);
                    weightmap[e] = weight;
                    if(weight < 1e-4)
                        ROS_WARN("weight is zero");
                    is_push = true;
                    N_edge++;
                    N_edge_sub++;
                }
            }
            if(is_push)
            {
                useful_layer.push_back(*it_cur);
                is_push = false;
            }
        }

        prev_layer = useful_layer;
        cout<<"[Preplanner] connected edge: "<<N_edge_sub<<", node: "<<useful_layer.size()<<endl;
        useful_layer.clear();
    }
    
    cout<<"[Preplanner] total number of edges "<<N_edge<<endl;

    // graph finishing 
    GridField* prev_vsf_ptr = vsf_field_ptr_seq[H-1].get();
    Vertex vf = boost::add_vertex(Point(), di_graph);
    descriptor_map.insert(make_pair(VertexName("xf"), vf));

    // step3 : let's connect with previous layer 
    for(auto it_prev=prev_layer.begin(); it_prev<prev_layer.end(); it_prev++)
    {
        // prev_layer 
        Vertex prev_vert = descriptor_map[it_prev->name];
        // this condition should be satisfied to be connected 
        //boost::add_edge(prev_vert, vf, 0, di_graph);
        boost::tie(e, inserted) = boost::add_edge(prev_vert, vf, di_graph);
        weightmap[e] = 0;
    }
}

VertexPath Preplanner::dijkstra(Vertex v0, Vertex vf)
{
    // Create things for Dijkstra
    vector<Vertex> predecessors(boost::num_vertices(di_graph)); // To store parents
    vector<Weight> distances(boost::num_vertices(di_graph)); // To store distances

    IndexMap indexMap = boost::get(boost::vertex_index, di_graph); // get property map objects from a graph
    NameMap nameMap = boost::get(boost::vertex_name, di_graph);

    PredecessorMap predecessorMap(&predecessors[0], indexMap);
    DistanceMap distanceMap(&distances[0], indexMap);    

    boost::dijkstra_shortest_paths(di_graph, v0, boost::distance_map(distanceMap).predecessor_map(predecessorMap));

    typedef vector<Graph::edge_descriptor> PathType;

    PathType path;
    Vertex v = vf; // We want to start at the destination and work our way back to the source

    for(Vertex u=predecessorMap[v]; // Start by setting 'u' to the destintaion node's predecessor
        u!=v; // Keep tracking the path until we get to the source
        v=u, u=predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
    {
        pair<Graph::edge_descriptor, bool> edgePair = boost::edge(u, v, di_graph);
        Graph::edge_descriptor edge = edgePair.first;
        path.push_back(edge);
    }

    if(path.size())
    {
        cout<<"[Preplanner] path exists"<<endl;
        // Write shortest path
        float totalDistance = 0;

        VertexPath vertex_path1;
        VertexPath vertex_path2;

        for(PathType::reverse_iterator pathIterator=path.rbegin(); pathIterator!=path.rend(); ++pathIterator)
        {
            //ROS_INFO("path insertion");
            vertex_path1.push_back(nameMap[boost::source(*pathIterator, di_graph)]);
            vertex_path2.push_back(nameMap[boost::target(*pathIterator, di_graph)]);
        }

        vertex_path1.push_back(vertex_path2.back());
        return vertex_path1;
    }
    else
    {
        ROS_WARN("[Preplanner] path does not exist. returning zero length path. ");
        return VertexPath();
    }
}

void Preplanner::compute_shortest_path()
{
    cout<<"[Preplanner] shortest path requested"<<endl;
    VertexPath solution_seq = dijkstra(descriptor_map["x0"], descriptor_map["xf"]);
    // if path exist 
    if(solution_seq.size())
    {
        solution_seq.pop_back();

        // from graph path to real path 
        preplanned_path.poses.clear();
        // marker update  
        marker_wpnts.points.resize(solution_seq.size());
        marker_wpnts.colors.resize(solution_seq.size());  

        for(auto it=solution_seq.begin(); it<solution_seq.end(); it++)
        {
            //cout<<"Dijkstra: "<<it->x<<" "<<it->y<<" "<<it->z<<endl;
            geometry_msgs::PoseStamped pose_stamped;

            pose_stamped.pose.position = *it;
            preplanned_path.poses.push_back(pose_stamped);

            marker_wpnts.colors.push_back(marker_wpnts.color);
            marker_wpnts.points.push_back(*it);
        }
    }
    else
        ROS_WARN("[Preplanner] The preplanning couldn't be updated. (smooth planner may try to make path on old preplan.) ");
}

void Preplanner::preplan(GridField* edf_grid_ptr, vector<Point> target_pnts, vector<Twist> target_vels, Point chaser_init)
{
    for(auto it=target_pnts.begin(); it<target_pnts.end(); it++)
        it->z = params.min_z + 1e-3;

    ros::Time begin = ros::Time::now();
    compute_visibility_field_seq(edf_grid_ptr, target_pnts, target_vels);
    graph_construct(edf_grid_ptr, chaser_init);
    compute_shortest_path();
    ros::Time end = ros::Time::now();
    cout<<"[Preplanner] Dijkstra completed in "<<(end-begin).toSec()*1000<<"ms"<<endl;


    int grid_length = ceil(params.d_trakcing_max/params.vsf_resolution);
    gridPathFinder grid_path_finder(2*grid_length, 2*grid_length);
    Vector3d global_xyz_l, global_xyz_u, start_pt, end_pt, half_side;

    half_side << params.d_trakcing_max, params.d_trakcing_max, (params.d_trakcing_max * sin(params.max_azim) - params.d_trakcing_min * sin(params.min_azim))/2;
    unsigned int i = 0;

    if(chase_start_point(0) != 0)
        start_pt = chase_start_point;
    else
        start_pt << chaser_init.x, chaser_init.y, 0.4;
    end_pt << target_pnts[i].x, target_pnts[i].y, target_pnts[i].z;
    global_xyz_l << start_pt - half_side;
    global_xyz_u << start_pt + half_side;

    grid_path_finder.initGridMap(params.vsf_resolution, global_xyz_l, global_xyz_u);

    VertexPath solution_seq;

    grid_path_finder.AstarSearch(start_pt, end_pt, edf_grid_ptr);

    vector<Vector3d> path = grid_path_finder.getPath();
    unsigned int j = 0;
    Point waypoint;

    for(; j < path.size(); j++)
        if((path[j] - end_pt).norm() <= 0.5)
        {
            waypoint.x = path[j](0);
            waypoint.y = path[j](1);
            waypoint.z = path[j](2);
            break;
        }

    chaser_path.poses.clear();
    tar_obs.clear();

    if(path.size())
    {
        for(unsigned int k = 0; k <= j; k++)
        {
            geometry_msgs::PoseStamped pose_stamped;
            Point pt;
            pt.x = path[k](0);
            pt.y = path[k](1);
            pt.z = path[k](2);
            pose_stamped.pose.position = pt;
            chaser_path.poses.push_back(pose_stamped);
        }
    }
    
    for(int m = 1; m < 4; m++)
    {
        gridPathFinder grid_path_finder(2*grid_length, 2*grid_length);

        start_pt << waypoint.x, waypoint.y, waypoint.z;
        end_pt << target_pnts[i+m].x, target_pnts[i+m].y, target_pnts[i+m].z;

        global_xyz_l << start_pt - half_side;
        global_xyz_u << start_pt + half_side;

        grid_path_finder.initGridMap(params.vsf_resolution, global_xyz_l, global_xyz_u);
        grid_path_finder.AstarSearch(start_pt, end_pt, edf_grid_ptr);
        vector<Vector3d> path = grid_path_finder.getPath();

        for(j = 0; j < path.size(); j++)
            if((path[j] - end_pt).norm() <= 0.5)
            {
                waypoint.x = path[j](0);
                waypoint.y = path[j](1);
                waypoint.z = path[j](2);
                break;
            }

        if(path.size())
        {
            for(unsigned int k = 0; k < j; k++)
            {
                geometry_msgs::PoseStamped pose_stamped;
                Point pt;
                pt.x = path[k](0);
                pt.y = path[k](1);
                pt.z = path[k](2);
                pose_stamped.pose.position = pt;
                chaser_path.poses.push_back(pose_stamped);
            }
            if(m == 3)
            {
                geometry_msgs::PoseStamped pose_stamped;
                Point pt;
                pt.x = path[j](0);
                pt.y = path[j](1);
                pt.z = path[j](2);
                pose_stamped.pose.position = pt;
                chaser_path.poses.push_back(pose_stamped);
            }
        }
    }
    
    Vector3d temp_pt;
    double time;
    for(int k = 0; k < chaser_path.poses.size(); k++)
    {
        if(k == 0)
        {
            Vector3d pt(chaser_path.poses[k].pose.position.x, chaser_path.poses[k].pose.position.y, chaser_path.poses[k].pose.position.z);
            time = 0;
            temp_pt = pt;
            tar_obs.push_back(make_pair(time, pt));
        }
        if(k%3 == 0 && k > 0 && k+3 <= chaser_path.poses.size())
        {
            double px = chaser_path.poses[k].pose.position.x + chaser_path.poses[k+1].pose.position.x + chaser_path.poses[k+2].pose.position.x;
            double py = chaser_path.poses[k].pose.position.y + chaser_path.poses[k+1].pose.position.y + chaser_path.poses[k+2].pose.position.y;
            double pz = chaser_path.poses[k].pose.position.z + chaser_path.poses[k+1].pose.position.z + chaser_path.poses[k+2].pose.position.z;
            Vector3d pt(px/3, py/3, pz/3);
            time += (pt-temp_pt).norm();
            temp_pt = pt;
            tar_obs.push_back(make_pair(time, pt));
        }
        if(k+3 > chaser_path.poses.size()-1)
        {
            k = chaser_path.poses.size() - 1;
            Vector3d pt(chaser_path.poses[k].pose.position.x, chaser_path.poses[k].pose.position.y, chaser_path.poses[k].pose.position.z);
            time += (pt-temp_pt).norm();
            tar_obs.push_back(make_pair(time, pt));
        }
    }

    VectorXd coef = estimate_T(tar_obs);
    double t_lm = 1*(tar_obs.back().first-tar_obs.front().first);
    //cout<<"coef: "<<coef.transpose()<<endl;
    //cout<<"t_lm: "<<t_lm<<endl;

    Point parent;
    Vector3d cur_pt(0,0,0);
    Matrix<double, 1, 6> t_p;
    t_p << 1, t_lm, pow(t_lm, 2), pow(t_lm, 3), pow(t_lm, 4), pow(t_lm, 5);
    parent.x = t_p*coef.segment<6>(0);
    parent.y = t_p*coef.segment<6>(6);
    parent.z = t_p*coef.segment<6>(12);
    chase_start_point << parent.x, parent.y, parent.z;
    int ste = 0;
    
    for(double dT=0; dT<t_lm*1.001; dT+=t_lm/20)
    {
        Matrix<double, 1, 6> t_p;
        t_p << 1, dT, pow(dT, 2), pow(dT, 3), pow(dT, 4), pow(dT, 5);
        parent.x = t_p*coef.segment<6>(0);
        parent.y = t_p*coef.segment<6>(6);
        parent.z = t_p*coef.segment<6>(12);
        /*
        if(ste<12)
            parent.z = coef(ste);
        if(ste==12)
            parent.z = t_lm;
        */
        if(cur_pt(0)!=0)
        {
            clength += sqrt(pow(cur_pt(0)-parent.x,2)+pow(cur_pt(1)-parent.y,2));
            cur_pt << parent.x, parent.y, parent.z;
        }
        else
            cur_pt << parent.x, parent.y, parent.z;
        chaser_traj.points.push_back(parent);
        ste++;
    }
    //cout<<"clength: "<<clength<<endl;
    pub_chaser_traj.publish(chaser_traj);
    chaser_traj.points.clear();

    pub_chaser.publish(chaser_path);
/*
    if(solution_seq.size())
    {
        cout<<"solution_seq: "<<solution_seq.size()<<endl;
        preplanned_path.poses.clear();
        marker_wpnts.points.resize(solution_seq.size());
        marker_wpnts.colors.resize(solution_seq.size());  

        for(auto it=solution_seq.begin(); it<solution_seq.end(); it++)
        {
            geometry_msgs::PoseStamped pose_stamped;

            pose_stamped.pose.position = *it;
            preplanned_path.poses.push_back(pose_stamped);

            marker_wpnts.colors.push_back(marker_wpnts.color);
            marker_wpnts.points.push_back(*it);
        }
    }
*/
}

nav_msgs::Path Preplanner::get_preplanned_waypoints() {return preplanned_path;}

void Preplanner::publish()
{
    pub_vsf_vis.publish(markers_visibility_field_seq); // vsf seq
    pub_waypoints.publish(marker_wpnts); // waypoints
    pub_preplanned_path.publish(preplanned_path); // preplanned path
}

VectorXd Preplanner::estimate_T(const list<pair<double, Vector3d>> &tar_ob)
{
    int _NUM_P = tar_ob.size();
    VectorXd coef, _coef, _P;
    double t_start = 0.0;
    double t_lm = 0.0;
    _P = VectorXd::Zero(3*_NUM_P);
    MatrixXd _T = MatrixXd::Zero(3*_NUM_P, 18);
    MatrixXd _t = MatrixXd::Zero(1, 6);
    MatrixXd __T = MatrixXd::Zero(18, 18);
    MatrixXd __t = MatrixXd::Zero(4, 4);
    MatrixXd _para = MatrixXd::Zero(4, 4);
    t_start = tar_ob.front().first;
    t_lm = 2*(tar_ob.back().first-tar_ob.front().first);

    __t << 4, 6, 8, 10,
           6, 12, 18, 24,
           8, 18, 28.8, 40,
           10, 24, 40, 57.1429;

    _para << t_lm, pow(t_lm, 2), pow(t_lm, 3), pow(t_lm, 4),
             pow(t_lm, 2), pow(t_lm, 3), pow(t_lm, 4), pow(t_lm, 5),
             pow(t_lm, 3), pow(t_lm, 4), pow(t_lm, 5), pow(t_lm, 6),
             pow(t_lm, 4), pow(t_lm, 5), pow(t_lm, 6), pow(t_lm, 7);

    for (int i=0; i<3; i++)
        __T.block<4, 4>(6*i+2, 6*i+2) = __t.cwiseProduct(_para);

    for (list<pair<double, Vector3d>>::const_iterator it=tar_ob.begin(), end=tar_ob.end() ; it!=end; ++it)
    {
        double dT = it->first-t_start;
        int i = distance(tar_ob.begin(), it);

        _t << 1, pow(dT, 1), pow(dT, 2), pow(dT, 3), pow(dT, 4), pow(dT, 5);
        _T.block<1, 6>(3*i, 0) = _t;
        _T.block<1, 6>(3*i+1, 6) = _t;
        _T.block<1, 6>(3*i+2, 12) = _t;
        _P.segment<3>(3*i) = it->second;
    }

    int nx = 18;
    int my = 0;
    int mz = 0;
    int nnzA = 0;
    int nnzC = 0;
    int nnzQ = 0;
    int irowA[my], jcolA[my], irowC[mz], jcolC[mz];
    double dA[my], bA[my], dC[mz];

    double c[nx];
    VectorXd vtemp;
    vtemp = -_T.transpose()*_P;
    for (int i=0; i<nx; i++)
        c[i] = vtemp(i);
    MatrixXd _Q = MatrixXd::Zero(18, 18);
    _Q = _T.transpose()*_T+0.1*__T;

    vector<pair<pair<int, int>, double> > tmp;
    for (int i=0; i<_Q.rows(); i++)
        for (int j=0; j<_Q.cols(); j++)
            if (j<=i && _Q(i, j)>1e-2)
                nnzQ++;

    int irowQ[nnzQ], jcolQ[nnzQ], i_q = 0;
    double dQ[nnzQ];

    tmp.resize(nnzQ);
    for (int i=0; i<_Q.rows(); i++)
        for (int j=0; j<_Q.cols(); j++)
            if (j<=i && _Q(i, j)>1e-2)
                tmp[i_q++] = make_pair(make_pair(i, j), _Q(i, j));

    sort(tmp.begin(), tmp.end());

    for (unsigned int i=0; i<tmp.size(); ++i)
    {
        irowQ[i] = tmp[i].first.first;
        jcolQ[i] = tmp[i].first.second;
        dQ[i] = tmp[i].second;
    }

    double xupp[nx];
    char ixupp[nx];
    for (int i = 0 ; i < nx; i++)
        xupp[i] = 0.0;
    memset(ixupp, 0, sizeof(ixupp));

    double xlow[nx];    
    char ixlow[nx];
    for (int i = 0; i < nx; i++)
        xlow[i] = 0.0;
    memset(ixlow, 0, sizeof(ixlow));

    QpGenSparseMa27 *qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);
    QpGenData *prob = (QpGenData*)qp->copyDataFromSparseTriple(c, irowQ, nnzQ, jcolQ, dQ,
                                                               xlow, ixlow, xupp, ixupp,
                                                               irowA, nnzA, jcolA, dA, bA,
                                                               irowC, nnzC, jcolC, dC,
                                                               xlow, ixlow, xupp, ixupp);
    QpGenVars *vars = (QpGenVars*) qp->makeVariables(prob);
    QpGenResiduals *resid = (QpGenResiduals*) qp->makeResiduals(prob);
    GondzioSolver *s = new GondzioSolver(qp, prob);
    //s->monitorSelf();
    int status = s->solve(prob, vars, resid);
    delete s;
    delete qp;
    coef = VectorXd::Zero(nx);
    double d[nx];
    vars->x->copyIntoArray(d);
    for (int i=0; i<nx; i++)
        coef(i) = d[i];
    return coef;
}