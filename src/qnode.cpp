#include "qnode.h"
// constructor 
QNode::QNode(int argc,char** argv,const std::string &name):init_argc(argc),init_argv(argv),node_name(name){}

QNode::~QNode(){}

void QNode::ros_comms_init()
{    
    ros::NodeHandle nh("~");
    nh.param("pred_horizon",pred_horizon,float(4.0));
    nh.param("early_end_time",early_end_time,0.1);
    nh.param<int>("pred_seq",pred_seq,4);

    // target manager init
    this->target_manager.init(nh);

    // wrapper init
    this->chaser_wrapper.init(nh);
}

void QNode::shutdown() 
{
    if(ros::isStarted()) 
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::on_init()
{
    ros::init(init_argc,init_argv,node_name);
    if(!ros::master::check()) 
        return false;

    ros::start(); // our node handles go out of scope, so we want to control shutdown explicitly.
    ros_comms_init();
    start();
    return true;
}

void QNode::run()
{
    ros::Rate loop_rate(50);
    writeOnBoard("waiting EDF computation...");

    while(ros::ok())
    {
        double sim_time;

        if(is_in_session) // simulation activate button clicked
        {
            /*
                previous_elapsed = time passed between simulatoin button clicked twice
                button_click_time = simulation button click time
            */
            sim_time = previous_elapsed + (ros::Time::now() - button_click_time).toSec();
            //cout<<"sim_time: "<<sim_time<<endl;

            bool trigger_condition;
            if(prediction_mode == 0) // receive future segment informatoin from target manager
            {
                /*
                    last_tigger_time = last QNode::trigger(sim_time) time, initial = 0
                    early_end_time = 0.1
                    simulation_end_time = 20
                */
                trigger_condition = (last_tigger_time == 0 or (sim_time - last_tigger_time) > pred_horizon - early_end_time ) and sim_time < simulation_end_time;
                //cout<<"trigger_condition: "<<trigger_condition<<endl;
            }
            else
                trigger_condition = false; // TODO for real prediction case

            // if triggered do the followings 
            if(trigger_condition)                                
                trigger(sim_time);
        }
        else
            sim_time = previous_elapsed; 

        // publish the current information
        target_manager.session(sim_time);
        chaser_wrapper.session(sim_time);

        // chaser information board
        // is octomap received
        if(chaser_wrapper.objects_handler.is_map_recieved and (not is_said_edf))
        {
            writeOnBoard("EDF loaded");
            is_said_edf = true;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    Q_EMIT rosShutdown();
}

bool QNode::trigger_one_shot(double tf)
{
    // target global waypoitns  
    vector<Point> target_seq = extract_pnts_from_path(target_manager.get_global_waypoints());
    if (target_seq.size() == 0)
    {
        this->writeOnBoard("size of provided target sequence is zero. please generate target traj first.");
        return false;
    }
    if(not chaser_wrapper.objects_handler.is_chaser_spawned)
    {
        this->writeOnBoard("chaser has not been spawned.");
        return false;
    }
    if(not chaser_wrapper.objects_handler.is_map_recieved)
    {
        this->writeOnBoard("octomap or edf has not been uploaded.");
        return false;
    }    

    // trigger chasing
    VectorXd knots;    
    knots.setLinSpaced(target_manager.queue.size(),0,tf);
    vector<Twist> target_vel_seq;
    for(int i=0; i<4; i++)
        target_vel_seq.push_back(Twist()); // TOTALLY NOT USED
    bool is_success = chaser_wrapper.trigger_chasing(target_seq, target_vel_seq, knots);        
}

bool QNode::trigger(double t_cur)
{
    last_tigger_time = t_cur;
    VectorXd pred_horizon_vec(pred_seq);
    pred_horizon_vec.setLinSpaced(pred_seq,t_cur,t_cur + pred_horizon); // pred_horizon = 4
    cout<<"pred_horizon_vec :"<<pred_horizon_vec.transpose()<<endl;
    vector<Point> target_pt_seq = target_manager.eval_time_seq(pred_horizon_vec); // predicted target position at pred_horizon_vec
    vector<Twist> target_vel_seq = target_manager.eval_vel_seq(pred_horizon_vec); // predicted target velocity at pred_horizon_vec
    
    if(target_pt_seq.size() == 0)
    {
        this->writeOnBoard("size of provided target sequence is zero. please generate target traj first.");
        return false;
    }
    if(not chaser_wrapper.objects_handler.is_chaser_spawned)
    {
        this->writeOnBoard("chaser has not been spawned.");
        return false;
    }
    if(not chaser_wrapper.objects_handler.is_map_recieved)
    {
        this->writeOnBoard("octomap or edf has not been uploaded.");
        return false;
    }    

    return chaser_wrapper.trigger_chasing(target_pt_seq, target_vel_seq, pred_horizon_vec); 
}