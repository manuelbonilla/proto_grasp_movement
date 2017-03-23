#include <pg_movement.h>

pg_movement::pg_movement()
{
	sub_which_finger_ = n_.subscribe("/which_finger", 28, &pg_movement::callWichFinger, this);

	visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("vito_anchor", "/rviz_visual_markers"));
	visual_tools_->deleteAllMarkers();
	pub_command_ = n_.advertise<lwr_controllers::CartesianImpedancePoint>("/right_arm/cartesian_impedance_controller/command", 0);
	hand_publisher_ = n_.advertise<std_msgs::Float64>("/right_hand/hand_position", 1000);
	flag_which_finger_ = flag_grasp_ = false;

	trajectory_type = 1;
	n_.param<int>("/trajectory_type", trajectory_type, 1);



	pkg_path_ = ros::package::getPath("proto_grasp_movement");

	/*
	flag_error_joint_trajectory_ = false;
	sub_error_joint_trajectory_ = n_.subscribe("/right_arm/joint_trajectory_controller/state", 100, &pg_movement::checkError, this);
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

	std::cout << "\n\nwait controller and press enter!" << std::endl;
	getchar();
	*/

	// init msg for publisher
	zero_stiffness_.x = 800.0;
	zero_stiffness_.y = 800.0;
	zero_stiffness_.z = 800.0;
	zero_stiffness_.rx = 50.0;
	zero_stiffness_.ry = 50.0;
	zero_stiffness_.rz = 50.0;
	msg_.k_FRI = zero_stiffness_;

	zero_stiffness_.x = 0.2;
	zero_stiffness_.y = 0.2;
	zero_stiffness_.z = 0.2;
	zero_stiffness_.rx = 0.2;
	zero_stiffness_.ry = 0.2;
	zero_stiffness_.rz = 0.2;
	msg_.d_FRI = zero_stiffness_;

	zero_wrench_.force.x = 0;
	zero_wrench_.force.y = 0;
	zero_wrench_.force.z = 0;
	zero_wrench_.torque.x = 0;
	zero_wrench_.torque.y = 0;
	zero_wrench_.torque.z = 0;
	msg_.f_FRI = zero_wrench_;

	visual_tools_->deleteAllMarkers();


	A_ = 0.15;

	offset_x_ = -0.92;
	offset_y_ = 0.7;
	offset_z_ = 0.135; //0.123 side

	delta_degree_ = 1;
	degree_ = 0;


	handClosure(0.0);
	homePosition();
}

pg_movement::~pg_movement()
{

}


void pg_movement::homePosition()
{
	// get current transofrmation between "vito_anchor" and "right_palm_link"
	std::string link_from = "right_hand_palm_link";
	std::string link_to   = "vito_anchor";

	tf::TransformListener listener;
	tf::StampedTransform t;
	bool flag_tf = false;

	while (!flag_tf)
	{
		ros::spinOnce();
		try
		{
			listener.waitForTransform(link_from, link_to, ros::Time(0), ros::Duration(1)); //ros::Duration(2.5)
			listener.lookupTransform(link_to, link_from,  ros::Time(0), t);
			flag_tf = true;
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	Eigen::Quaterniond q_init;
	Eigen::Vector3d v_init;
	tf::quaternionTFToEigen(t.getRotation(), q_init);
	tf::vectorTFToEigen(t.getOrigin(), v_init);

	Eigen::Affine3d pose_init, pose_home;

	pose_init = Eigen::Affine3d(q_init);
	pose_init.translation() = Eigen::Vector3d(v_init(0), v_init(1), v_init(2)); // translate x,y,z
	//put a quaternion instead of rpy in pose_home
	pose_home = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()); // rotate along "AXIS" axis by 90 degrees
	pose_home.translation() = Eigen::Vector3d( offset_x_ + A_ , offset_y_ , offset_z_ ); // translate x,y,z

	interpolation(pose_init, pose_home);
	//waiting for interpolation...

	std::cout << "\r\n\n\n\033[32m\033[1mHome Position \033[0m" << std::endl;
	std::cout << "\r\n\n\n\033[32m\033[1mPress to continue.. \033[0m" << std::endl;

	getchar();

}


//------------------------------------------------------------------------------------------
//                                                                             interpolation
//------------------------------------------------------------------------------------------
void pg_movement::interpolation(Eigen::Affine3d x_start, Eigen::Affine3d x_finish)
{
	float th = 0.001;	//trade error
	float alpha = 0.015;
	float translation_error = 1;
	float c = 0; //for slerp

	Eigen::Affine3d x_next, x_now, x_prev;

	x_now  = x_start;
	x_prev = x_start;

	geometry_msgs::Pose x_finish_frame, x_now_frame;

	// read quaternion from GeometryPoseMsg and convert them to Eigen::Quaterniond
	//passing from affine3d to geometry_msg
	tf::poseEigenToMsg(x_now, x_now_frame);
	tf::poseEigenToMsg(x_finish, x_finish_frame);

	Eigen::Quaterniond q_start(x_now_frame.orientation.w, x_now_frame.orientation.x, x_now_frame.orientation.y, x_now_frame.orientation.z);
	Eigen::Quaterniond q_finish(x_finish_frame.orientation.w, x_finish_frame.orientation.x, x_finish_frame.orientation.y, x_finish_frame.orientation.z);
	Eigen::Quaterniond q_err;


	ros::Rate r(60);
	while (translation_error >= th || c == 1 )
	{
		// update orientation
		if (c <= 1)
			q_err = q_start.slerp(c, q_finish);

		x_next = Eigen::AngleAxisd(q_err);

		// updata translation
		if (translation_error > th)
			x_next.translation() = (x_finish.translation() - x_now.translation() ) * alpha + x_prev.translation();

		// visual tool
		visual_tools_->publishAxis(x_next, 0.1, 0.01, "axis");
		visual_tools_->trigger();

		// set control
		tf::poseEigenToMsg(x_next, msg_.x_FRI);
		pub_command_.publish(msg_);
		ros::spinOnce();

		// update error
		translation_error = std::abs((x_finish.translation() - x_now.translation() ).norm());

		// updata frames
		x_prev = x_now;
		x_now = x_next;
		c += 0.01;
		r.sleep();
	}

	// update global pose_ for next steps
	pose_ = x_finish;
}


//------------------------------------------------------------------------------------------
//                                                                            	  kukaCircle
//------------------------------------------------------------------------------------------
void pg_movement::kukaCircle()
{
	// Create pose
	float x = 0;
	float y = 0;
	float z = 0;
	float theta = 0;
	float h = 0;


	if (degree_ <= 360)
	{
		// rviz print
		degree_ += delta_degree_;
		theta = degree_ * M_PI / 180;
		x =  offset_x_ + A_ * cos(theta);
		y =  offset_y_ + A_ * sin(theta);
		z =  offset_z_ + h;

		pose_ = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) ; // rotate along "AXIS" axis by 90 degrees
		pose_.translation() = Eigen::Vector3d( x, y, z ); // translate x,y,z

		printf("Publishing in Circle   %f\n", degree_);
		visual_tools_->publishAxis(pose_, 0.1, 0.01, "axis");
		visual_tools_->trigger();

		// set control
		tf::poseEigenToMsg(pose_, msg_.x_FRI);
		pub_command_.publish(msg_);
		ros::spinOnce();
	}
	else
	{
		visual_tools_->deleteAllMarkers();
		degree_ = 0;
	}
}


//------------------------------------------------------------------------------------------
//                                                                            	  	kukaRect
//------------------------------------------------------------------------------------------
void pg_movement::kukaRect()
{
	// Create pose
	float x = 0;
	float y = 0;
	float z = 0;
	float theta = 0;
	float h = 0;

	if (degree_ < 360)
	{
		// rviz print
		degree_ += delta_degree_;
		theta = degree_ * M_PI / 180;

		if (degree_ >= 0 && degree_ <= 70)
		{
			x =  offset_x_ + A_ * cos(theta);
			y =  offset_y_;
			visual_tools_->deleteAllMarkers();
		}
		else if (degree_ > 70 && degree_ <= 180)
		{
			x = offset_x_ + A_ * cos(70 * M_PI / 180);
			y = offset_y_ + A_ * cos(theta) - A_ * cos(70 * M_PI / 180);
			visual_tools_->deleteAllMarkers();
		}
		else if (degree_ > 180 && degree_ <= 250)
		{
			x = offset_x_ + A_ * cos(70 * M_PI / 180) + A_ * cos(theta) - A_ * cos(180 * M_PI / 180);
			y = offset_y_  + A_ * cos(180 * M_PI / 180) - A_ * cos(70 * M_PI / 180);
			visual_tools_->deleteAllMarkers();
		}
		else if (degree_ > 250 && degree_ <= 360)
		{
			x = offset_x_ + A_ * cos(70 * M_PI / 180) + A_ * cos(250 * M_PI / 180) - A_ * cos(180 * M_PI / 180);
			y = offset_y_ + A_ * cos(180 * M_PI / 180) - A_ * cos(70 * M_PI / 180) - A_ * cos(250 * M_PI / 180) + A_ * cos(theta);
			visual_tools_->deleteAllMarkers();
		}


		std::cout << "x:  " << x <<  "  y: " << y << std::endl;

		z =  offset_z_ + h;
		pose_ = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) ; // rotate along "AXIS" axis by 90 degrees
		pose_.translation() = Eigen::Vector3d( x, y, z ); // translate x,y,z

		printf("Publishing in Rect   %f\n", degree_);
		visual_tools_->publishAxis(pose_, 0.1, 0.01, "axis");
		visual_tools_->trigger();

		// set control
		tf::poseEigenToMsg(pose_, msg_.x_FRI);
		pub_command_.publish(msg_);
		ros::spinOnce();
	}
	else
	{
		visual_tools_->deleteAllMarkers();
		degree_ = 0;
	}

	if (degree_ == 0 || degree_ == 70 || degree_ == 180 || degree_ == 250)
		sleep(1);
}



//------------------------------------------------------------------------------------------
//                                                                             tondoDatabase
//------------------------------------------------------------------------------------------
void pg_movement::tondoDatabase()
{
	Eigen::Affine3d pose_grasp;

	float x, y, z, angle;

	openTondoDatabase(x, y, z, angle);

	pose_grasp =  Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(angle * M_PI / 180, Eigen::Vector3d::UnitZ()); // rotate along "AXIS" axis by 90 degrees
	pose_grasp.translation() = pose_.translation() + Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d(x, y, z); // translate x,y,z

	interpolation(pose_, pose_grasp);
}


//------------------------------------------------------------------------------------------
//                                                                         openTondoDatabase
//------------------------------------------------------------------------------------------
void pg_movement::openTondoDatabase(float& x, float& y, float& z, float& angle)
{
	if (finger_name_ == "frontal_thumb")
	{
		x = 0.005;
		y = 0.045;
		z = -0.020;
		angle = 35;
	}
	else if (finger_name_ == "frontal_index")
	{
		x = 0.13;
		y = 0.015;
		z = 0.02;
		angle = -15;
	}
	else if (finger_name_ == "frontal_middle")
	{
		x = 0.135;
		y = -0.015;
		z = 0.025;
		angle = 12;
	}
	else if (finger_name_ == "frontal_ring")
	{
		x = 0.12;
		y = -0.045;
		z = 0.015;
		angle = 5;
	}
	else if (finger_name_ == "frontal_little")
	{
		x = 0.11;
		y = -0.07;
		z = 0.03;
		angle = -11;
	}
	else if (finger_name_ == "side_index")
	{
		x = 0.060; //0.070
		y = 0.038; //0.04
		z = 0.015; //0.020
		angle = 45;
	}
	else if (finger_name_ == "side_little")
	{
		x = 0.1;
		y = -0.03;
		z = 0.05;
		angle = -35;
	}
	else if (finger_name_ == "side_thumb")
	{
		x = -0.005;
		y = 0.04;
		z = -0.03;
		angle = 40;
	}
	else if (finger_name_ == "vertical_thumb")
	{
		x = -0.005;
		y = 0.045;
		z = -0.04;
		angle = 45;
	}
	else if (finger_name_ == "vertical_index")
	{
		x = 0.060; // 0.065
		y = 0.015;
		z = -0.02;
		angle = 20;
	}
	else if (finger_name_ == "vertical_ring")
	{
		x = 0.075;
		y = -0.025;
		z = -0.005;
		angle = 6;
	}
	else if (finger_name_ == "vertical_little")
	{
		x = 0.075;
		y = -0.04;
		z = -0.01;
		angle = -16;
	}
	else if (finger_name_ == "vertical_middle")
	{
		x = 0.085;
		y = -0.005;
		z = -0.04; //0.015
		angle = 13;
	}
	else
	{
		x = 0;
		y = 0;
		z = 0;
		angle = 0;
		ROS_ERROR("Error in Alessandro Tondo Database");
	}
}



//------------------------------------------------------------------------------------------
//                                                                            finishPosition
//------------------------------------------------------------------------------------------
void pg_movement::finishPosition(float z)
{
	Eigen::Affine3d pose_finish;

	pose_finish = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()); // rotate along "AXIS" axis by 90 degrees
	pose_finish.translation() = pose_.translation() + Eigen::Vector3d(0, 0, z); // translate x,y,z

	interpolation(pose_, pose_finish);

	std::cout << "\r\n\n\n\033[32m\033[1mGrasped Object\033[0m" << std::endl;
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

	if (msg.data == "frontal_thumb")				//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "frontal_middle")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "frontal_ring")			//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "frontal_index")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "frontal_little")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "side_index")			//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "side_thumb")			//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "side_little")			//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_thumb")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_middle")     //////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_ring")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_index")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_little")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else
	{
		flag_which_finger_ = false;
		/* NOTHING TO DO */
	}
}


//------------------------------------------------------------------------------------------
//                                                                                   manager
//------------------------------------------------------------------------------------------
void pg_movement::manager()
{

	if (!flag_which_finger_  && !flag_grasp_ )
	{
		// handClosure(0.0);

		switch (trajectory_type) {
		case 1:
			kukaRect();
			ROS_INFO_STREAM("Using trajectory: " << trajectory_type);
			break;
		case 2:
			kukaCircle();
			ROS_INFO_STREAM("Using trajectory: " << trajectory_type);
			break;
		default:
			ROS_INFO("No trajectoye chossen");
		}
	}
	else
	{
		flag_grasp_ = true;

		tondoDatabase();

		handClosure(1);

		visual_tools_->deleteAllMarkers();
		sleep(2);

		finishPosition(0.08);

		std::cout << "\r\n\n\n\033[32m\033[1mPress to open the hand... \033[0m" << std::endl;
		getchar();
		handClosure(0.0);

		ros::shutdown();
	}


	ros::spinOnce();
}





//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//
//                                        NEW DATABASE
//
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------
//                                                                                subscriber
//------------------------------------------------------------------------------------------
/*void pg_movement::checkError(control_msgs::JointTrajectoryControllerState msg)
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
}*/


//------------------------------------------------------------------------------------------
//                                                                               openAndLoad
//------------------------------------------------------------------------------------------
/*void pg_movement::openAndLoadDB(Eigen::MatrixXf &Qrotation, Eigen::MatrixXf &translation)
{
	int i=0, j=0;
	std::string s, s1, s2, s3, line, line1, line2;
	std::vector<std::string> q_vector_string;
	std::vector<std::string> t_vector_string;

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
}*/




//------------------------------------------------------------------------------------------
//                                                                             graspDatabase
//------------------------------------------------------------------------------------------
/*void pg_movement::graspDatabase()
{
	Eigen::Affine3d pose_grasp;

	float x = -T_(cg_,0);
	float y = -T_(cg_,1);
	float z = -T_(cg_,2);
	Eigen::Quaternion<double> q_tmp(Q_(cg_,3),Q_(cg_,0), Q_(cg_,1), Q_(cg_,2));
	pose_grasp = q_tmp.toRotationMatrix() * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()); // rotate along "AXIS" axis by 90 degrees
	// pose_grasp =  Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()); // rotate along "AXIS" axis by 90 degrees
	pose_grasp.translation() = pose_.translation() + Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d(x, y, z); // translate x,y,z

	printf("Publishing Movement");
	visual_tools_->publishAxis(pose_grasp, 0.1, 0.01,"axis");
	visual_tools_->trigger();

	// set control
	tf::poseEigenToMsg(pose_grasp, msg_.x_FRI);
	pub_command_.publish(msg_);
	ros::spinOnce();
}*/

//------------------------------------------------------------------------------------------
//                                                                manager  with new database
//------------------------------------------------------------------------------------------
/*void pg_movement::manager()
{

	if(!flag_which_finger_  && !flag_grasp_ )
	{
		handClosure(0.0);
		// kukaCircle();
		kukaRect();
		// cg_ = 0; //to select the file row
	}
	else
	{
		flag_grasp_ = true;
		if(cg_==0)
		{
			openAndLoadDB(Q_,T_);
		    visual_tools_->deleteAllMarkers();
		}
		graspDatabase();
		cg_ ++ ;
		std::cout << "rows: " << rows_num_ << "    cg_ " << cg_ << std::endl;
		if(cg_ == rows_num_)
		{
			handClosure(1.0);
			getchar();
			handClosure(0.0);
			ros::shutdown();
		}
	}

	ros::spinOnce();
}*/




//------------------------------------------------------------------------------------------
//                                                                         switchControllers
//------------------------------------------------------------------------------------------
/*bool pg_movement::switchControllers(std::vector<std::string> target_controllers, std::vector<std::string> running_controllers)
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
}*/



//------------------------------------------------------------------------------------------
//                                                                              homePosition
//------------------------------------------------------------------------------------------
/*void pg_movement::homePosition()
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
    // joint_position[0] =   -3 * M_PI/180;
    // joint_position[1] =   -8 * M_PI/180;
    // joint_position[2] =   88 * M_PI/180;
    // joint_position[3] =  -26 * M_PI/180;
    // joint_position[4] =   53 * M_PI/180;
    // joint_position[5] =  -37 * M_PI/180;
    // joint_position[6] =  113 * M_PI/180;
    joint_position[0] =  -0.77;
    joint_position[1] =  -0.38;
    joint_position[2] =  -1.72;
    joint_position[3] =   1.54;
    joint_position[4] =   0.38;
    joint_position[5] =   0.11;
    joint_position[6] =  -0.24;

    trajectory_msgs::JointTrajectoryPoint joint_point;
    for (int i=0; i<7 ; i++)
    {
    	joint_point.positions.push_back(joint_position[i]);
    }
    joint_point.time_from_start = ros::Duration(3);

    msg.points.push_back(joint_point);

    ros::Rate r(10);
    int count_pub = 0;
    do
    {
     	pub_trajectory_controller.publish(msg);
     	ros::spinOnce();
     	r.sleep();
     	count_pub++;
     	std::cout << "pub: " << count_pub << std::endl;
    }while(!flag_error_joint_trajectory_  || count_pub!=10);

    std::cout << "\r\n\n\n\033[32m\033[1mHOME POSITION \033[0m" << std::endl;
    sleep(3);
} */

//------------------------------------------------------------------------------------------
//                                                                          useful functions
//------------------------------------------------------------------------------------------
// ================================================================================= QxQ
/*Eigen::Vector4f pg_movement::QxQ(Eigen::Vector4f tmp1, Eigen::Vector4f tmp2)
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
}*/
