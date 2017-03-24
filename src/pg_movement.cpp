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
	n_.param<double>("/traj_time", traj_time, 2.0);



	pkg_path_ = ros::package::getPath("proto_grasp_movement");


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
}

pg_movement::~pg_movement()
{

}


void pg_movement::homePosition()
{
	// get current transofrmation between "vito_anchor" and "right_palm_link"
	std::string link_from = "/right_arm_7_link";
	std::string link_to   = "/world";

	tf::TransformListener listener;
	tf::StampedTransform t;
	// bool flag_tf = false;

	// while (!flag_tf)
	// {
	ros::spinOnce();
	try
	{
		listener.waitForTransform(link_from, link_to, ros::Time(0), ros::Duration(20)); //ros::Duration(2.5)
		listener.lookupTransform(link_to, link_from,  ros::Time(0), t);
		// flag_tf = true;
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}
	// }

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

	interpolation(pose_init, pose_home, traj_time);
	//waiting for interpolation...

	std::cout << "\r\n\n\n\033[32m\033[1mHome Position \033[0m" << std::endl;
	std::cout << "\r\n\n\n\033[32m\033[1mPress to continue.. \033[0m" << std::endl;

	getchar();

}


//------------------------------------------------------------------------------------------
//                                                                             interpolation
//------------------------------------------------------------------------------------------
int pg_movement::interpolation(Eigen::Affine3d x_start, Eigen::Affine3d x_finish, double traj_time_local)
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

	double frec(300.0);
	ros::Rate r(frec);
	while (c <= 1 )
	{
		// update orientation
		double ctanh(std::tanh(4*c));
		if (c <= 1)
			q_err = q_start.slerp(c, q_finish);

		x_next = Eigen::AngleAxisd(q_err);

		// updata translation
		if (translation_error > th)
			x_next.translation() = x_now.translation() + ctanh * (x_finish.translation() - x_now.translation() ) ;

		// visual tool
		visual_tools_->publishAxis(x_next, 0.1, 0.01, "axis");
		visual_tools_->trigger();

		// set control
		tf::poseEigenToMsg(x_next, msg_.x_FRI);
		pub_command_.publish(msg_);
		ros::spinOnce();

		ctanh += (1.0 / frec) / traj_time_local;

		if(flag_which_finger_ && !flag_grasp_)
		{
			pose_ = x_next;
			return 0;
		}

		r.sleep();
	}

	// update global pose_ for next steps
	pose_ = x_finish;
	return 1;
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

	double box_size(0.15);
	Eigen::Affine3d x_new;
	int flag_local = 1;
	while( flag_local == 1)
	{
		x_new = pose_;
		x_new.translation() = x_new.translation() + Eigen::Vector3d(-box_size, 0.0, 0.0); // translate x,y,z =
		if(interpolation(pose_, x_new, traj_time)==0){
			flag_local = 0.0;
			break;
		}
		sleep(1.0);
		x_new = pose_;
		x_new.translation() = x_new.translation() + Eigen::Vector3d(0.0, -box_size, 0.0); // translate x,y,z =
		if(interpolation(pose_, x_new, traj_time)==0){
			flag_local = 0.0;
			break;
		}
		sleep(1.0);
		x_new = pose_;
		x_new.translation() = x_new.translation() + Eigen::Vector3d(box_size, 0.0, 0.0); // translate x,y,z =
		if(interpolation(pose_, x_new, traj_time)==0){
			flag_local = 0.0;
			break;
		}
		sleep(1.0);
		x_new = pose_;
		x_new.translation() = x_new.translation() + Eigen::Vector3d(0.0, box_size, 0.0); // translate x,y,z =
		if(interpolation(pose_, x_new, traj_time)==0){
			flag_local = 0.0;
			break;
		}
		sleep(1.0);
	}
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
	ROS_INFO_STREAM("Executing primitive");
	interpolation(pose_, pose_grasp, traj_time);
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

	interpolation(pose_, pose_finish, traj_time);

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

		flag_grasp_ =  false;

		std::cout << "\r\n\n\n\033[32m\033[1mPress to open the hand... \033[0m" << std::endl;
		getchar();
		handClosure(0.0);

		ros::shutdown();
	}


	ros::spinOnce();
}



