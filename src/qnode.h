#ifndef QNODE_H
#define QNODE_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <QThread>
#include <QStringListModel>

#include <auto_chaser/Common.h>
#include <auto_chaser/Wrapper.h>
#include <target_manager/TargetManager.h>

class QNode : public QThread
{
	Q_OBJECT

	public:
		// memebers
		TargetManager target_manager;
		Wrapper chaser_wrapper;

		QNode(int argc,char** argv,const std::string &name);
		~QNode();

		bool on_init();
		void shutdown();
		void run(); // the entire while loop 
		bool trigger_one_shot(double tf); // this triggers chasing path for entire target path (0-tf)
		bool trigger(double t_cur); // triggers chasing update at a defined frequnecy 
		QStringListModel* loggingModel() {return &logging;}
		const std::string& nodeName() {return node_name;}

		// initialize the waypoints in queue
		void wpnts_init();
		// logging directory
		string write_path;

		// flags
		bool is_connected = false; // to ros
		bool is_in_session = false; // is in simulation session
		bool is_said_edf = false;

		// time  
		double button_elapsed=0; // after the button pressed again,
		double record_dt = 0.5;
		ros::Time button_click_time; 
		double previous_elapsed = 0.0;
		double last_tigger_time = 0;
		double simulation_end_time;
		double early_end_time = 0.1;
		 
		// params
		bool arrow_record_switch = true;
		float pred_horizon;

		// 0: receive future segment informatoin from target manager
		// 1: predict informatoin by object handler 
		int prediction_mode = 0; 
		int pred_seq = 4;

	Q_SIGNALS:
		void loggingUpdated();
		void rosShutdown();
		void writeOnBoard(QString);

	protected:
		void ros_comms_init();
		int init_argc;
		char** init_argv;
		QStringListModel logging;
		const std::string node_name;
};

#endif // QNODE_H