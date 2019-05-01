#include "auto_chaser/Preplanner.h"

Preplanner::Preplanner(){};

void Preplanner::init(ros::NodeHandle nh)
{
    // preplanner params parsing 
    nh.param("w_v",params.w_v,5.0);       
    nh.param("w_d",params.w_d,1.5);            
    nh.param("r_safe",params.r_safe,0.5);
    nh.param("min_z",params.min_z,0.4);
    nh.param("vs_min",params.vs_min,0.3);
    nh.param("vsf_resolution",params.vsf_resolution,0.7);
    nh.param("d_connect_max",params.d_connect_max,2.5);

    nh.param("max_tracking_distance",params.d_trakcing_max,4.0);
    nh.param("min_tracking_distance",params.d_trakcing_min,1.0);
    nh.param("des_tracking_distance",params.d_trakcing_des,2.5);
    nh.param("max_azim",params.max_azim,(3.141592/4));
    nh.param("min_azim",params.min_azim,(3.141592/7));

    // world_frame_id 
    nh.param<string>("world_frame_id",world_frame_id,"/world");
    nh.param<string>("world_frame_id",markers_visibility_field_base.header.frame_id,"/world");
    nh.param<string>("world_frame_id",preplanned_path.header.frame_id,"/world");

    // marker initialize 
    
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

    // vsf_grid_seq 

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
};

FieldParams Preplanner::get_local_vsf_param_around_target(Point target_pnt, Twist target_vel)
{
    FieldParams vsf_param;    
    double lx, ly, lz;
    lx = ly = 4*params.d_trakcing_max;
    lz = params.d_trakcing_max * sin(params.max_azim) - params.d_trakcing_min * sin(params.min_azim);
    vsf_param.x0 = target_pnt.x - lx/2;
    vsf_param.y0 = target_pnt.y - ly/2;
    vsf_param.z0 = target_pnt.z;
    double x = target_vel.linear.x;
    double y = target_vel.linear.y;
    //cout<<"[Preplanner] x: "<<x<<", y: "<<y<<endl;
    if(y>0 && atan(abs(x/y))<PI/4)
    {
        vsf_param.lx = lx;
        vsf_param.ly = (1 + abs(x/y)) * 2 * params.d_trakcing_max;
        vsf_param.lz = lz;
        //cout<<"ly: "<<vsf_param.ly<<endl;
    }
    else if(x>0 && atan(abs(y/x))<PI/4)
    {
        vsf_param.lx = (1 + abs(y/x)) * 2 * params.d_trakcing_max;
        vsf_param.ly = ly;
        vsf_param.lz = lz;
        //cout<<"lx: "<<vsf_param.lx<<endl;
    }
    else
    {
        vsf_param.lx = lx;
        vsf_param.ly = ly;
        vsf_param.lz = lz;
        //cout<<"normal"<<endl;
    }
    
    vsf_param.resolution = params.vsf_resolution;
    vsf_param.ray_stride_res = params.vsf_resolution; // not used for vsf grid 

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
    
    vector<Node<Point>> prev_layer, useful_layer;
    Node<Point> initial_node;
    initial_node.value = x0;
    initial_node.name = "x0";
    prev_layer.push_back(initial_node);
    
    Vertex_d v0 = boost::add_vertex(x0, di_graph); // Point x0 is NamePorperty of Graph
    descriptor_map.insert(make_pair(VertexName("x0"), v0));

    int H = vsf_field_ptr_seq.size(); // total prediction horizon
    int N_edge = 0;
    int N_edge_sub = 0;
    bool is_push = 0;

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
            Vertex_d cur_vert = boost::add_vertex(cur_pnt, di_graph); // add vertex with vertex_property_type
            descriptor_map.insert(make_pair(it_cur->name, cur_vert));
            
            // call the previous layer
            GridField* prev_vsf_ptr = vsf_field_ptr_seq[t-1].get();
            // step 2: let's connect with previous layer and add edges 
            for(auto it_prev=prev_layer.begin(); it_prev<prev_layer.end(); it_prev++)
            {
                Vertex_d prev_vert = descriptor_map[it_prev->name];
                Point prev_pnt = it_prev->value;
                Vector3f prev_vec = geo2eigen(prev_pnt);

                // this condition should be satisfied to be connected 
                if(((cur_vec-prev_vec).norm() < params.d_connect_max) && (edf_grid_ptr->getRayMin(cur_pnt, prev_pnt, 0) > params.r_safe))
                {
                    float weight = (cur_vec-prev_vec).norm()
                                   + params.w_v * 1/sqrt(cur_vsf_ptr->getRayMean(cur_pnt, prev_pnt) * prev_vsf_ptr->getRayMean(prev_pnt, cur_pnt))
                                   + params.w_d * pow(((geo2eigen(cur_vsf_ptr->getCentre()) - cur_vec).norm() - params.d_trakcing_des), 2);                     
                    boost::add_edge(prev_vert, cur_vert, weight, di_graph);
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
    Vertex_d vf = boost::add_vertex(Point(), di_graph);
    descriptor_map.insert(make_pair(VertexName("xf"), vf));

    // step3 : let's connect with previous layer 
    for(auto it_prev=prev_layer.begin(); it_prev<prev_layer.end(); it_prev++)
    {
        // prev_layer 
        Vertex_d prev_vert = descriptor_map[it_prev->name];
        // this condition should be satisfied to be connected 
        boost::add_edge(prev_vert, vf, 0, di_graph);
    }
}

VertexPath Preplanner::dijkstra(Vertex_d v0, Vertex_d vf)
{
    // Create things for Dijkstra
    vector<Vertex_d> predecessors(boost::num_vertices(di_graph)); // To store parents
    vector<Weight> distances(boost::num_vertices(di_graph)); // To store distances

    IndexMap indexMap = boost::get(boost::vertex_index, di_graph); // get property map objects from a graph
    NameMap nameMap = boost::get(boost::vertex_name, di_graph);

    PredecessorMap predecessorMap(&predecessors[0], indexMap);
    DistanceMap distanceMap(&distances[0], indexMap);    

    boost::dijkstra_shortest_paths(di_graph, v0, boost::distance_map(distanceMap).predecessor_map(predecessorMap));

    typedef vector<Graph::edge_descriptor> PathType;

    PathType path;
    Vertex_d v = vf; // We want to start at the destination and work our way back to the source

    for(Vertex_d u=predecessorMap[v]; // Start by setting 'u' to the destintaion node's predecessor
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
    // set the height of the moving target 
    for(auto it=target_pnts.begin(); it<target_pnts.end(); it++)
        it->z = params.min_z + 1e-3;

    compute_visibility_field_seq(edf_grid_ptr, target_pnts, target_vels);  
    graph_construct(edf_grid_ptr, chaser_init);        
    compute_shortest_path();   
}

nav_msgs::Path Preplanner::get_preplanned_waypoints() {return preplanned_path;}

void Preplanner::publish()
{
    pub_vsf_vis.publish(markers_visibility_field_seq); // vsf seq
    pub_waypoints.publish(marker_wpnts); // waypoints
    pub_preplanned_path.publish(preplanned_path); // preplanned path
}