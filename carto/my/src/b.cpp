f2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate loop_rate(16);
	int num = 0;
	while (ros::ok())
    {
		geometry_msgs::TransformStamped ti1;
		geometry_msgs::TransformStamped ti2;
		geometry_msgs::TransformStamped t1;
		geometry_msgs::TransformStamped t2;
		
		t2.transform.translation.x=2.0;
		t2.transform.translation.y=2.0;
		t2.transform.translation.z=0.0;
		t2.transform.rotation.x=0.0;
		t2.transform.rotation.y=0.0;
		t2.transform.rotation.z=0.0;
		t2.transform.rotation.w=1.0;

		if (num>10)
        {
			t1 = tfBuffer.lookupTransform("map", "base_link",ros::Time(0));
			Eigen::Affine3d pose = tf2::transformToEigen(t1);
			Eigen::Affine3d pose_inverse = pose.inverse();
        	ti1 = tf2::eigenToTransform(pose_inverse);

			tf::Quaternion quat;
	        tf::quaternionMsgToTF(ti1.transform.rotation, quat);
			double roll, pitch, yaw;
    		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			//std::cout<<tiq.transform.translation.x<<" "<<ti1.transform.translation.y<<" "<<yaw<<std::endl;

			Eigen::Affine3d t_in= tf2::transformToEigen(t2);
			Eigen::Affine3d t_out;
            tf2::doTransform (t_in, t_out, ti1);//将t_in转换到base_link下
			ti2 = tf2::eigenToTransform(t_out);
			
		}
	    cartographer_ros_msgs::LandmarkEntry landmark1,landmark2;
	    cartographer_ros_msgs::LandmarkList landmarks;
    	landmark1.translation_weight = 1.0;
    	landmark1.rotation_weight = 0.0;
    	landmark1.id = "1000000";

    	landmark2.translation_weight = 1.0;
    	landmark2.rotation_weight = 0.0;
    	landmark2.id = "1000001";

		float randa = (random(100)-50)*0.001;
		/*float ang=2*asin(t.transform.rotation.z);
		landmark.tracking_from_landmark_transform.position.x = -t.transform.translation.x*cos(ang) - t.transform.translation.y*sin(ang);
    	landmark.tracking_from_landmark_transform.position.y = t.transform.translation.x*sin(ang) - t.transform.translation.y*cos(ang);
    	landmark.tracking_from_landmark_transform.position.z = t.transform.translation.z;
    	landmark.tracking_from_landmark_transform.orientation.x = -t.transform.rotation.x;
    	landmark.tracking_from_landmark_transform.orientation.y = -t.transform.rotation.y;
    	landmark.tracking_from_landmark_transform.orientation.z = -t.transform.rotation.z;
    	landmark.tracking_from_landmark_transform.orientation.w = t.transform.rotation.w;*/

    	landmark1.tracking_from_landmark_transform.position.x = ti1.transform.translation.x + randa;
    	landmark1.tracking_from_landmark_transform.position.y = ti1.transform.translation.y + randa;
    	landmark1.tracking_from_landmark_transform.position.z = ti1.transform.translation.z;
    	landmark1.tracking_from_landmark_transform.orientation.x = ti1.transform.rotation.x;
    	landmark1.tracking_from_landmark_transform.orientation.y = ti1.transform.rotation.y + randa*0.1;
    	landmark1.tracking_from_landmark_transform.orientation.z = ti1.transform.rotation.z;
    	landmark1.tracking_from_landmark_transform.orientation.w = ti1.transform.rotation.w;

		landmark2.tracking_from_landmark_transform.position.x = ti2.transform.translation.x + randa;
    	landmark2.tracking_from_landmark_transform.position.y = ti2.transform.translation.y + randa;
    	landmark2.tracking_from_landmark_transform.position.z = ti2.transform.translation.z;
    	landmark2.tracking_from_landmark_transform.orientation.x = ti2.transform.rotation.x;
    	landmark2.tracking_from_landmark_transform.orientation.y = ti2.transform.rotation.y + randa*0.1;
    	landmark2.tracking_from_landmark_transform.orientation.z = ti2.transform.rotation.z;
    	landmark2.tracking_from_landmark_transform.orientation.w = ti2.transform.rotation.w;

		landmarks.header.stamp = ros::Time::now();
		landmarks.header.frame_id = "base_link";
		landmarks.landmarks.push_back(landmark1);
        //landmarks.landmarks.push_back(landmark2);
    	landmarks_pub.publish(landmarks);
    	ros::spinOnce();
    	loop_rate.sleep();
		num++;
    }
