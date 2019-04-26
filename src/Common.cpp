#include "auto_chaser/Common.h"

std::string GetCurrentWorkingDir( void )
{
    char buff[FILENAME_MAX];
    GetCurrentDir( buff, FILENAME_MAX );
    std::string current_working_dir(buff);
    return current_working_dir; 
}

vector<Point> extract_pnts_from_path(nav_msgs::Path path)
{
    vector<Point> pnt_seq;
    for(auto it = path.poses.begin(); it<path.poses.end();it++)
        pnt_seq.push_back(it->pose.position);

    return pnt_seq;
};

Vector3f geo2eigen(const Point& pnt)
{
    return Vector3f(pnt.x, pnt.y, pnt.z);
};

void get_color_dist(float dist_val, std_msgs::ColorRGBA& color, float max_plot_dist_val)
{
    // error region 
    if(dist_val < 0)
    {
        color.r = 0.5;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 0.2;
    }
    else
    {
        color.r = pow(dist_val/max_plot_dist_val, 3);
        color.g = pow(dist_val/max_plot_dist_val, 3);
        color.b = pow(dist_val/max_plot_dist_val, 3);
    }
    // plot only cells in this bound
    if(dist_val < max_plot_dist_val)
        color.a = 0.2;
};

void get_color(float x_in, float& r, float& g, float& b)
{
    // Only important if the number of colors is small. In which case the rest is
    // still wrong anyway
    // x = linspace(0,1,jj)' * (1-1/jj) + 1/jj;
    const double rone = 0.8;
    const double gone = 1.0;
    const double bone = 1.0;
    float x = x_in;
    x = (x_in<0 ? 0 : (x>1 ? 1 : x));

    if (x<1. / 8.)
    {
        r = 0;
        g = 0;
        b = bone*(0.5 + (x) / (1. / 8.)*0.5);
    }
    else if (x<3. / 8.)
    {
        r = 0;
        g = gone*(x - 1. / 8.) / (3. / 8. - 1. / 8.);
        b = bone;
    }
    else if (x<5. / 8.)
    {
        r = rone*(x - 3. / 8.) / (5. / 8. - 3. / 8.);
        g = gone;
        b = (bone - (x - 3. / 8.) / (5. / 8. - 3. / 8.));
    }
    else if (x<7. / 8.)
    {
        r = rone;
        g = (gone - (x - 5. / 8.) / (7. / 8. - 5. / 8.));
        b = 0;
    }
    else
    {
        r = (rone - (x - 7. / 8.) / (1. - 7. / 8.)*0.5);
        g = 0;
        b = 0;
    }
}

GridField::GridField(){};
GridField::GridField(FieldParams param)
{
    // size and grid resoluton does not change 
    params = param;
    float lx,ly,lz;
    lx = params.lx;
    ly = params.ly;
    lz = params.lz;
    float res = params.resolution;
    node_xs = VectorXf((int)floor(lx/res));
    node_ys = VectorXf((int)floor(ly/res));
    node_zs = VectorXf((int)floor(lz/res));
    
    Nx = node_xs.size();
    Ny = node_ys.size();
    Nz = node_zs.size();
    
    // node generate
    for (int i = 0;i<Nx;i++)
        node_xs(i) = params.x0 + params.resolution/2 + params.resolution*i;
    for (int i = 0;i<Ny;i++)
        node_ys(i) = params.y0 + params.resolution/2 + params.resolution*i;
    for (int i = 0;i<Nz;i++)
        node_zs(i) = params.z0 + params.resolution/2 + params.resolution*i;            

    // mesh 
    for (int ix=0; ix<Nx; ix++)
        for (int iy=0; iy<Ny; iy++)
            for (int iz=0; iz<Nz; iz++)
            {
                Point pnt;
                pnt.x = node_xs(ix);
                pnt.y = node_ys(iy);
                pnt.z = node_zs(iz);
                pnts_list.push_back(pnt);
            }

    // field array generate
    field_vals = new float**[Nx];
    for (int x=0; x<Nx; x++)
    {
        field_vals[x] = new float*[Ny];
        for (int y=0; y<Ny; y++)
            field_vals[x][y] = new float[Nz];     
    }
};

vector<Node<Point>> GridField::generate_node(int prefix)
{
    // the node names are t{prefix}n{local index}
    vector<Node<Point>> nodes;
    int local_id;

    if(saved_points.size())
    {
        for(auto it=saved_points.begin(); it<saved_points.end(); it++, local_id++)
        {
            Node<Point> node;
            node.name = "t" + to_string(prefix) + "n" + to_string(local_id);
            node.value = *it;
            nodes.push_back(node);
        }
    }
    else
    {
        ROS_WARN("node generation failed. no saved points in grid field.");
    }
    return nodes;
}

void GridField::setOrigin(Point X0)
{
    // node reset 
    for (int i = 0;i<Nx;i++)
        node_xs(i) = X0.x + params.resolution/2 + params.resolution*i;
    for (int i = 0;i<Ny;i++)
        node_ys(i) = X0.y + params.resolution/2 + params.resolution*i;
    for (int i = 0;i<Nz;i++)
        node_zs(i) = X0.z + params.resolution/2 + params.resolution*i;   
}

Point GridField::getOrigin()
{
    Point pnt;
    pnt.x = node_xs(0);
    pnt.y = node_ys(0);
    pnt.z = node_zs(0);
    return pnt;        
}

Point GridField::getCentre()
{
    Point pnt;
    pnt.x = node_xs(0) + params.lx/2;
    pnt.y = node_ys(0) + params.ly/2;
    pnt.z = node_zs(0) + params.lz/2;
    return pnt;        
}

Point GridField::getCellPnt(Vector3i idx)
{    
    Point pnt;
    
    if(idx(0)<0 or idx(0)>Nx-1 or idx(1)<0 or idx(1)>Ny-1 or idx(2) < 0 or idx(2) > Nz-1)
    {
        pnt.x = -999;
        pnt.y = -999;
        pnt.z = -999;
        ROS_WARN("[Grid field]: referencing index is out of bound");    
    }
    else
    {
        pnt.x = node_xs(idx(0));
        pnt.y = node_ys(idx(1));
        pnt.z = node_zs(idx(2)); 
    }
    return pnt;
}

Vector3i GridField::getCellIdx(Point pnt)
{
    Vector3i idx;

    float x0 = node_xs(0)-params.resolution/2;
    float y0 = node_ys(0)-params.resolution/2;
    float z0 = node_zs(0)-params.resolution/2;
    float xf = node_xs(node_xs.size()-1)+params.resolution/2;
    float yf = node_ys(node_ys.size()-1)+params.resolution/2;
    float zf = node_zs(node_zs.size()-1)+params.resolution/2;

    if(!(x0>pnt.x || xf<pnt.x || y0>pnt.y || yf<pnt.y || z0>pnt.z || zf<pnt.z))
    {
        idx(0) = (int)(floor((pnt.x - x0)/params.resolution));
        idx(1) = (int)(floor((pnt.y - y0)/params.resolution));
        idx(2) = (int)(floor((pnt.z - z0)/params.resolution));
    }
    else // out of range
    {
        idx.setConstant(-1);
        // cout<<"[Grid field] Warnning: referencing point is out of bound."<<endl;    
    }        
    return idx;        
}

vector<Vector3i> GridField::getRayIdx(Point pnt1,Point pnt2)
{
    vector<Vector3i> ray_idx;
    Vector3f pnt1_vec(pnt1.x,pnt1.y,pnt1.z);
    Vector3f pnt2_vec(pnt2.x,pnt2.y,pnt2.z);
    // current end of the ray 
    Vector3f cur_vec = pnt1_vec;

    float ray_check_res = this->params.ray_stride_res;
    Vector3f stride_vec = (pnt2_vec - pnt1_vec).normalized()*ray_check_res; 
    float cur_ray_len=0;
    float tot_length = (pnt2_vec - pnt1_vec).norm();
    // traverse this ray 
    while(cur_ray_len <= tot_length)
    {            
        cur_vec = cur_vec + stride_vec;
        cur_ray_len = cur_vec.norm();
        Point cur_end_pnt;
        cur_end_pnt.x = cur_vec(0);
        cur_end_pnt.y = cur_vec(1);
        cur_end_pnt.z = cur_vec(2);            
        ray_idx.push_back(Vector3i(getCellIdx(cur_end_pnt)));
    }

    return ray_idx;        
}

float GridField::getValue(Point pnt)
{
    Vector3i idx = getCellIdx(pnt);
    return field_vals[idx(0)][idx(1)][idx(2)];
}

float GridField::getRayMin(Point pnt1, Point pnt2, float vs_min)
{       
    // vector<Vector3i> ray_idx = getRayIdx(pnt1,pnt2);  // it will take more time 
    Vector3f pnt1_vec(pnt1.x, pnt1.y, pnt1.z); // target point
    Vector3f pnt2_vec(pnt2.x, pnt2.y, pnt2.z); // eval_point
    // current end of the ray 
    Vector3f cur_vec = pnt1_vec;
    
    float tot_length = (pnt2_vec - pnt1_vec).norm();
    
    Vector3f stride_vec;        
    if(tot_length == 0)
        stride_vec.setZero();
    else 
        stride_vec = (pnt2_vec - pnt1_vec).normalized()*params.ray_stride_res; 

    float cur_ray_len = 0;
    // traverse this ray 
    float min_val = getValue(pnt1); // let's assume we don't have any minus value in the field 
    
    while(cur_ray_len < tot_length)
    {            
        cur_vec = cur_vec + stride_vec;
        cur_ray_len = (cur_vec - pnt1_vec).norm();
        Point cur_end_pnt;
        cur_end_pnt.x = cur_vec(0);
        cur_end_pnt.y = cur_vec(1);
        cur_end_pnt.z = cur_vec(2);     
        Vector3i idx = getCellIdx(cur_end_pnt);    

        // out of bound 
        if((idx(0)>=Nx || idx(1)>=Ny || idx(2)>=Nz || idx(0)==-1))
            break; // sometimes, the ray may cross out of bound (for most case because z0 >> 0)            
        float val = field_vals[idx(0)][idx(1)][idx(2)];
        
        if(val <= vs_min)
            return vs_min;

        if(val < min_val)
            min_val = val;            
    }
    return min_val;        
}
    

float GridField::getRayMean(Point pnt1, Point pnt2)
{
    // vector<Vector3i> ray_idx = getRayIdx(pnt1,pnt2);  // it will take more time 
    Vector3f pnt1_vec(pnt1.x, pnt1.y, pnt1.z);
    Vector3f pnt2_vec(pnt2.x, pnt2.y, pnt2.z);
    // current end of the ray 
    Vector3f cur_vec = pnt1_vec;

    float tot_length = (pnt2_vec - pnt1_vec).norm();
        
    Vector3f stride_vec;        
    if(tot_length == 0)
        stride_vec.setZero();
    else 
        stride_vec = (pnt2_vec - pnt1_vec).normalized()*params.ray_stride_res; 

    float cur_ray_len=0;
    // traverse this ray 
    float accum_sum = 0.0f; // let's assume we don't have any minus value
    int N_count = 0;
    while(cur_ray_len <= tot_length)
    {            
        cur_vec = cur_vec + stride_vec;
        cur_ray_len = (cur_vec - pnt1_vec).norm();
        Point cur_end_pnt;
        cur_end_pnt.x = cur_vec(0);
        cur_end_pnt.y = cur_vec(1);
        cur_end_pnt.z = cur_vec(2);     
        Vector3i idx = getCellIdx(cur_end_pnt);    
        
        if((idx(0)>=Nx || idx(1)>=Ny || idx(2)>=Nz || idx(0) == -1))
        {
            if(N_count == 0)
            {
                //ROS_WARN("the ray start point is out of range, returning 0");
                return 0;
            }
            return accum_sum/N_count; // sometimes, the ray may cross out of bound (for most case because z0 >> 0)            
        }
        accum_sum += field_vals[idx(0)][idx(1)][idx(2)];
        N_count++ ;
                    
    }
    return (accum_sum/N_count);        
}


void GridField::updateCell(Point pnt, float val)
{
    Vector3i idx = getCellIdx(pnt);
    if(idx(0) != -1)
        field_vals[idx(0)][idx(1)][idx(2)] = val; 
    else
    {
        ROS_WARN("[Gridmap] indexing out of range, no assign done");
    }
}

void GridField::updateCell(Vector3i idx, float val)
{
    field_vals[idx(0)][idx(1)][idx(2)] = val; 
}

