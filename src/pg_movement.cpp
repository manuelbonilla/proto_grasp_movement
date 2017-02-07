#include <pg_movement.h>

pg_movement::pg_movement()
{

	sub_which_finger_ = n_.subscribe("/which_finger", 28, &pg_movement::callWichFinger, this);

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("vito_anchor","/rviz_visual_markers"));  
    visual_tools_->deleteAllMarkers();
    pub_command_ = n_.advertise<lwr_controllers::CartesianImpedancePoint>("/right_arm/cartesian_impedance_controller/command",0);
	hand_publisher_= n_.advertise<std_msgs::Float64>("/right_hand/hand_position", 1000);
	sub_error_joint_trajectory_ = n_.subscribe("/right_arm/joint_trajectory_controller/state", 100, &pg_movement::checkError, this);
	flag_error_joint_trajectory_ = false;

    degree_ = 0;

    flag_which_finger_ = false;

  	pkg_path_ = ros::package::getPath("proto_grasp_movement");


  	std::vector<std::string> cartisian_controller, joint_controller;
    joint_controller.push_back("joint_trajectory_controller");
    cartisian_controller.push_back("cartesian_impedance_controller");


  	if(switchControllers(joint_controller,cartisian_controller))
  	{
  		std::cout << "switch to joint_trajectory_controller Done!" << std::endl;
  		homePosition();
  	    getchar();
  	}
  	else
  	{
  		ROS_ERROR("switch to joint_trajectory_controller controllers ERROR!");
  		ros::shutdown();
  	}


  	if(switchControllers(cartisian_controller,joint_controller))
  	{
  		std::cout << "switch to cartesian_impedance_controller Done!" << std::endl;
  	}
  	else
  	{
  		ROS_ERROR("switch to cartesian_impedance_controller controllers ERROR!");
  		ros::shutdown();
  	}





    // init msg for publisher
  	zero_stiffness_.x = 0;
    zero_stiffness_.y = 0;
    zero_stiffness_.z = 0;
    zero_stiffness_.rx = 0;
    zero_stiffness_.ry = 0;
    zero_stiffness_.rz = 0;

    zero_wrench_.force.x = 0;
    zero_wrench_.force.y = 0;
    zero_wrench_.force.z = 0;
    zero_wrench_.torque.x = 0;
    zero_wrench_.torque.y = 0;
    zero_wrench_.torque.z = 0;

    msg_.k_FRI = zero_stiffness_;
    msg_.d_FRI = zero_stiffness_;
    msg_.f_FRI = zero_wrench_;

}

pg_movement::~pg_movement()
{

}

//------------------------------------------------------------------------------------------
//                                                                         switchControllers
//------------------------------------------------------------------------------------------
bool pg_movement::switchControllers(std::vector<std::string> target_controllers, std::vector<std::string> running_controllers)
{
	std::string controllers_name;
    ros::ServiceClient client_list_controllers = n_.serviceClient<controller_manager_msgs::ListControllers>("/right_arm/controller_manager/list_controllers",true);
    controller_manager_msgs::ListControllers srv_list_controllers;

    ros::ServiceClient client_switch_controllers = n_.serviceClient<controller_manager_msgs::SwitchController>("/right_arm/controller_manager/switch_controller",true);
    controller_manager_msgs::SwitchController srv_switch_controllers;

    srv_switch_controllers.request.start_controllers = target_controllers;
    srv_switch_controllers.request.stop_controllers  = running_controllers;
    srv_switch_controllers.request.strictness        = controller_manager_msgs::SwitchController::Request::STRICT;

    ros::spinOnce();
    if (client_list_controllers.call(srv_list_controllers.request, srv_list_controllers.response))
    {
   		for (int i=0; i<srv_list_controllers.response.controller.size(); i++)
   		{
   			std::cout << "loaded controllers "<< i << "  ";
   			std::cout << srv_list_controllers.response.controller[i].name << "  " << srv_list_controllers.response.controller[i].state << std::endl;
   		   	if(srv_list_controllers.response.controller[i].name == target_controllers[0] && srv_list_controllers.response.controller[i].state == "stopped")
 			{	
 				client_switch_controllers.call(srv_switch_controllers.request,srv_switch_controllers.response);
 				return srv_switch_controllers.response.ok;
 			}
   		}
    }

}



//------------------------------------------------------------------------------------------
//                                                                              homePosition
//------------------------------------------------------------------------------------------
void pg_movement::homePosition()
{
	ros::Publisher pub_trajectory_controller =  n_.advertise<trajectory_msgs::JointTrajectory>("/right_arm/joint_trajectory_controller/command",1000);;
	trajectory_msgs::JointTrajectory msg;

	std::vector<std::string> joint_names;
	joint_names.resize(7);
	joint_names[0] = "right_arm_a1_joint";
	joint_names[1] = "right_arm_a2_joint";
	joint_names[2] = "right_arm_a3_joint";
	joint_names[3] = "right_arm_a4_joint";
	joint_names[4] = "right_arm_a5_joint";
	joint_names[5] = "right_arm_a6_joint";
	joint_names[6] = "right_arm_e1_joint";
	
	msg.joint_names = joint_names;

    std::vector<float> joint_position;
    joint_position.resize(7);
    joint_position[0] =  -47 * M_PI/180;  
    joint_position[1] =   -5 * M_PI/180;  
    joint_position[2] =  118 * M_PI/180; 
    joint_position[3] =  -40 * M_PI/180; 
    joint_position[4] =   69 * M_PI/180; 
    joint_position[5] =  -50 * M_PI/180; 
    joint_position[6] =  151 * M_PI/180; 

    trajectory_msgs::JointTrajectoryPoint joint_point;
    for (int i=0; i<7 ; i++)
    {
    	joint_point.positions.push_back(joint_position[i]);
    }
    joint_point.time_from_start = ros::Duration(1);

    msg.points.push_back(joint_point);

    ros::Rate r(10);
    while(!flag_error_joint_trajectory_)
    {
     	pub_trajectory_controller.publish(msg);
     	ros::spinOnce();
     	r.sleep();
    }

    std::cout << "\r\n\n\n\033[32m\033[1mHOME POSITION \033[0m" << std::endl;
}


//------------------------------------------------------------------------------------------
//                                                                            kukaTrajectory
//------------------------------------------------------------------------------------------
void pg_movement::kukaTrajectory()
{
    // Create pose
    float x=0;
    float y=0;
    float z=0;
    float theta=0;
    float h=0;
    float A=0.1;

    float offset_x = -1;
    float offset_y = 0.5;
    float offset_z = 0.15;

    float delta_degree = 0.5;

      
	if(degree_<=360)
	{
	    // rviz print
	    degree_+=delta_degree;
	    theta = degree_*M_PI/180;
	    x =  offset_x + A*cos(theta);
	    y =  offset_y + A*sin(theta);
	    z =  offset_z + h;

	    pose_ = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) ; // rotate along "AXIS" axis by 90 degrees
	    pose_.translation() = Eigen::Vector3d( x, y, z ); // translate x,y,z
	        
	    printf("Publishing in Circle   %f\n", degree_);
	    visual_tools_->publishAxis(pose_, 0.1, 0.01,"axis");
	    visual_tools_->trigger();

	    // set control 
	    tf::poseEigenToMsg(pose_, msg_.x_FRI);
	    pub_command_.publish(msg_);
	    ros::spinOnce();
	}
	else
	{   
	    visual_tools_->deleteAllMarkers();
	    degree_=0;
	}
}


//------------------------------------------------------------------------------------------
//                                                                                   manager
//------------------------------------------------------------------------------------------
void pg_movement::manager()
{

	if(!flag_which_finger_)
	{
		handClosure(0.0);
		kukaTrajectory();
		cg_ = 0;

	}
	else
	{	
		if(cg_==0)
		{
			openAndLoadDB(Q_,T_);
		    visual_tools_->deleteAllMarkers();
		}
		graspDatabase();
		cg_ ++ ;

		if(cg_ == rows_num_)
		{
			handClosure(1);
			ros::shutdown();
		}
	}


	ros::spinOnce();
}



//------------------------------------------------------------------------------------------
//                                                                             graspDatabase
//------------------------------------------------------------------------------------------
void pg_movement::graspDatabase()
{
    Eigen::Affine3d pose_grasp;
 
	float x = -T_(cg_,0);		
	float y = -T_(cg_,1);
	float z = -T_(cg_,2);
	Eigen::Quaternion<double> q_tmp(Q_(cg_,3),Q_(cg_,0), Q_(cg_,1), Q_(cg_,2));
    pose_grasp = q_tmp.toRotationMatrix() * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()); // rotate along "AXIS" axis by 90 degrees
	pose_grasp.translation() = pose_.translation() + Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d(x, y, z); // translate x,y,z


    printf("Publishing Movement");
    visual_tools_->publishAxis(pose_grasp, 0.1, 0.01,"axis");
    visual_tools_->trigger();

    // set control 
    tf::poseEigenToMsg(pose_grasp, msg_.x_FRI);
    pub_command_.publish(msg_);
    ros::spinOnce();
}

//------------------------------------------------------------------------------------------
//                                                                               openAndLoad
//------------------------------------------------------------------------------------------
void pg_movement::openAndLoadDB(Eigen::MatrixXf &Qrotation, Eigen::MatrixXf &translation)
{
	int i=0, j=0;
	std::string s, s1, s2, s3, line, line1, line2;
	std::vector<std::string> q_vector_string;
	std::vector<std::string> t_vector_string;

	// finger_name_ = "index";
	s1 = pkg_path_ + "/measurements/" + finger_name_ + "_rotation.txt";
	s2 = pkg_path_ + "/measurements/" + finger_name_ + "_translation.txt";
	s3 = pkg_path_ + "/measurements/" + finger_name_ + "_gyro.txt";

	//open files
	std::ifstream rotation_file, translation_file, gyro_file;
	rotation_file.open(s1.c_str());
	translation_file.open(s2.c_str());
	gyro_file.open(s3.c_str());

	//check file
	if(rotation_file.is_open())
		std::cout << "rotation_file is open" << std::endl;
	else
		std::cout <<" error occured while opening rotation_file" << std::endl;

	if(translation_file.is_open())
		std::cout << "translation_file is open" << std::endl;
	else
		std::cout <<" error occured while opening translation_file" << std::endl;
	
	if(gyro_file.is_open())
		std::cout << "gyro_file is open" << std::endl;
	else
		std::cout <<" error occured while opening gyro_file" << std::endl;


	//take rows number
	rows_num_=0;
	for (int p=0; std::getline(gyro_file, line); p++)
		rows_num_++;
		
	// record data
	Qrotation.resize(rows_num_,4); 
	translation.resize(rows_num_,3);

	// load translation matrix
	for (i=0; std::getline(translation_file, line2); i++)
	{
		boost::split(t_vector_string, line2, boost::is_any_of("\t\n"));

		for (j=0; j<3; j++)
		{
			float n = std::atof(t_vector_string[j].c_str());
			translation(i,j) = n;
		}
	}


	//load rotation matrix -> Quaternion
	for (i=0; std::getline(rotation_file, line1); i++)
	{
		boost::split(q_vector_string, line1, boost::is_any_of("\t\n"));
		for (j=0; j<4; j++)
		{
			float n = std::atof(q_vector_string[j].c_str());
			Qrotation(i,j) = n;
		}
		Qrotation.row(i) /= Qrotation.row(i).norm();
	}

	// build offset values
	Eigen::Vector4f Qoff(Qrotation(0,0), Qrotation(0,1), Qrotation(0,2), Qrotation(0,3)) ;
	Eigen::Vector3f Toff(translation(0,0), translation(0,1), translation(0,2));

	// compute translation offset
	for(i=0; i<rows_num_ ; i++ )
		translation.row(i) -= Toff;

	for(i=0; i<rows_num_ ; i++ )
		Qrotation.row(i) = QxQ(Qoff, ConjQ(Qrotation.row(i)));

}

//------------------------------------------------------------------------------------------
//                                                                               handClosure
//------------------------------------------------------------------------------------------
void pg_movement::handClosure(float v)
{
	std_msgs::Float64 msg;
	msg.data = v;
	hand_publisher_.publish(msg);
	ros::spinOnce();
}


//------------------------------------------------------------------------------------------
//                                                                                  callBack
//------------------------------------------------------------------------------------------
void pg_movement::callWichFinger(std_msgs::String msg)
{	
	if(msg.data == "little")
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if(msg.data == "ring")
	{
		flag_which_finger_ = true;
		/*  TO DO - READ GRASP DATABASE	*/
	}
	else if(msg.data == "middle")
	{
		flag_which_finger_ = true;
		/*  TO DO - READ GRASP DATABASE	*/
	}
	else if(msg.data == "index")
	{
		flag_which_finger_ = true;
		finger_name_ = msg.data;
	}
	else if(msg.data == "thumb")
	{
		flag_which_finger_ = true;
		/*  TO DO - READ GRASP DATABASE	*/
	}
	else
	{
		flag_which_finger_ = false;
		/*  TO DO - READ GRASP DATABASE	*/
	}
}


//------------------------------------------------------------------------------------------
//                                                                                subscriber
//------------------------------------------------------------------------------------------
void pg_movement::checkError(control_msgs::JointTrajectoryControllerState msg)
{
	Eigen::VectorXd err;
	err.resize(msg.error.positions.size());
	for (int i=0; i<err.size(); i++)
	{
		err(i) = msg.error.positions[i];
	} 

	if(err.norm()<0.01)
	{
		std::cout << "joint position norm " << err.norm() << std::endl;
		flag_error_joint_trajectory_ =true;
	}
	else
	{
		flag_error_joint_trajectory_ =false;
	}

}

//------------------------------------------------------------------------------------------
//                                                                          useful functions
//------------------------------------------------------------------------------------------

// ================================================================================= QxQ
Eigen::Vector4f pg_movement::QxQ(Eigen::Vector4f tmp1, Eigen::Vector4f tmp2) 
{
    Eigen::Vector4f tmp;
    Eigen::Vector4f Q_1(tmp1(3), tmp1(0), tmp1(1), tmp1(2));
    Eigen::Vector4f Q_2(tmp2(3), tmp2(0), tmp2(1), tmp2(2));

    tmp(0) = Q_1(0)*Q_2(0) - (Q_1(1)*Q_2(1) + Q_1(2)*Q_2(2) + Q_1(3)*Q_2(3)); //w
    tmp(1) = Q_1(0)*Q_2(1) + Q_1(1)*Q_2(0) + (Q_1(2)*Q_2(3) - Q_1(3)*Q_2(2)); //x
    tmp(2) = Q_1(0)*Q_2(2) + Q_1(2)*Q_2(0) + (Q_1(3)*Q_2(1) - Q_1(1)*Q_2(3)); //y
    tmp(3) = Q_1(0)*Q_2(3) + Q_1(3)*Q_2(0) + (Q_1(1)*Q_2(2) - Q_1(2)*Q_2(1)); //z
    
    Eigen::Vector4f Q_out(tmp(1), tmp(2), tmp(3), tmp(0));
    return Q_out;
}

// ================================================================================= ConjQ
Eigen::Vector4f pg_movement::ConjQ(Eigen::Vector4f Q_in) 
{
    Eigen::Vector4f Q_out;
    Q_out(0) = -Q_in(0);
    Q_out(1) = -Q_in(1);
    Q_out(2) = -Q_in(2);
    Q_out(3) = Q_in(3);
    return Q_out;
}