 bool move_to_target(velocity_smooth::smooth_srv::Request &req,
                        velocity_smooth::smooth_srv::Response &res)
    {   cmd_vel_pub.publish(geometry_msgs::Twist());
    	theta = req.theta - initial_angle_;
    	x=req.x-initial_x_;
    	y=req.y-initial_y_;
    	k=(req.y-initial_y_)/(req.x-initial_x_);
    	//test
    //	theta = 2* PI;
       ROS_INFO("remain angle : %f",      theta);
    	T= 
            (sqrt(theta/(6.0*beta_max_))) < T_max_ ? (sqrt(theta/(6*beta_max_))): T_max_;
    	Tx= 
    	    (sqrt(x/(6.0*ax_max_))) < Tx_max_ ? (sqrt(x/(6*ax_max_))): Tx_max_;
    	Ty=
    	    (sqrt(y/(6.0*ay_max_))) < Ty_max_ ? (sqrt(y/(6*ay_max_))): Ty_max_;
        IS_LONG_WAY_theta=
      		(sqrt(theta/(6.0*beta_max_))) < T_max_ ? false : true ;
        IS_LONG_WAY_x=
      		(sqrt(x/(6.0*ax_max_))) < Tx_max_ ? false : true ;
        IS_LONG_WAY_y=
            (sqrt(y/(6.0*ay_max_))) < Ty_max_ ? false : true ;
     //   if (IS_LONG_WAY_theta)
     // 	{
    //	   	T1=(theta-6*beta_max_* T_max_* T_max_)/(2.0 * beta_max_ * T_max_);
  	//	}
   	//	else    T1=0;
        theta_a_ = 3 * beta_max_ * T * T;
        x_a_ = 3 * ax_max_ * Tx * Tx;
        y_a_ = 3 * ay_max_ * Ty * Ty;
        goal_in_map.pose.position.x = req.x;
        goal_in_map.pose.position.y = req.y;
        geometry_msgs::Quaternion  quat = tf::createQuaternionMsgFromYaw(req.theta);
     //   geometry_msgs::Quaternion  quat = tf::createQuaternionMsgFromYaw(theta + initial_angle_);
        goal_in_map.pose.orientation = quat;
        error_x = error_y = 10;
        res.mark= 0;    //the position is not reached!
    //    while(fabs(error_x) > 0.01 || fabs(error_y) > 0.01 || fabs(error_theta) > 0.1)
         ros::Rate r(20); // 20 hz
         ros::Time begin = ros::Time::now();
         double time;
         while (ros::ok())     	  
        {
        	 ros::Duration curr=ros::Time::now()-begin;
			 time=curr.toSec();
        	 ros::spinOnce();
        	 //gain the current error of x y theta;
        	            try{
        	               listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(100.0) );
        	               listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        	                }
        	            catch (tf::TransformException ex){
        	               ROS_ERROR("%s",ex.what());
        	               ros::Duration(1.0).sleep();
        	               }          
        	            geometry_msgs::Quaternion quat0;
        	                quat0.x =  transform.getRotation().getX();
        	                quat0.y =  transform.getRotation().getY();
        	                quat0.z =  transform.getRotation().getZ();
        	                quat0.w =  transform.getRotation().getW();    	                
        	            current_theta_ = tf::getYaw(quat0);
        	            current_x_=transform.getOrigin().x();
                	    current_y_=transform.getOrigin().y();
        	            ROS_INFO("current_theta :  current_x : current_y %f %f %f",      current_theta_,current_x_,current_y_);
        	            
        	            listener.transformPose("/base_link",ros::Time(0), goal_in_map, "/map", goal_in_base);
        	          
        	            error_x = goal_in_base.pose.position.x;
        	            error_y = goal_in_base.pose.position.y;
        	            error_theta = tf::getYaw(goal_in_base.pose.orientation);
        	 //accelaration for rotation
        	            if(time < T)
        	           	 {
        	           	    w=(beta_max_/(2.0*T)) *(time * time);
        	           		         
        	           	 }
        	           	else if(time<(2.0*T))
        	           	 {
        	           	    w=beta_max_*(time-T/2.0);
        	           	 }
        	           	 else if(time<(3.0*T))
        	           	 {				  
        	           	    w=beta_max_*(-(time *time)/(2.0*T) + 3.0*time - 2.5*T);
        	           	 }
        	           	 else
        	           	 {
        	           	  if (IS_LONG_WAY_theta)
        	           	  {
        	                if((fabs(error_theta-theta_a_)>0.05) && (!first_mark))  // error limit need change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
        	           	    {       	           	           	       
        	                	w = w_max_;        	           	                      	
        	           	    }
        	                else 
        	                {   
        	                	first_mark=true;
        	                	if(t1_vector_.size() == 0)
        	                    {
        	                	   t1_vector_.push_back(time);
        	                 	   t1= time;  
        	                    }	
        	            
							   
//        	                	if (time < (t1+T))
//        	                	{  
//        	                		w=beta_max_*(-((t1+3*T-time) *(t1+3*T-time))/(2.0*T) + 3.0*(t1+3*T-time) - 2.5*T);  	                		
//        	                	}
//        	                	else if (time < (t1+2*T))
//        	                	{
//        	                		w=beta_max_*(t1+3*T-time-T/2.0);
//        	                	}
//        	                	else if (time < (t1+3*T))
//        	                	{
//        	                		 w=(beta_max_/(2.0*T)) *((t1+3*T-time)  * (t1+3*T-time) );
//        	                	}
//        	                	else w=0;
        	                	
        	                	if (!theta_final_arrive_mark)
        	                    {
        	                        w = (w_max_ / theta_a_) * (error_theta - 0.05);
        	                        if(fabs(error_theta)<0.06)  theta_first_arrive_mark=true;
        	                        	                 	                	  
        	                    }
        	                    else w = 0;
        	                }
        	           	  }
        	           	  else
        	              {   
        	           		  if (error_theta_vector_.size() == 0)
        	                  {
        	            	    error_theta_vector_.push_back(error_theta);
        	            	    error_theta_mark= error_theta;
        	                  }
        	           		  if(!theta_final_arrive_mark)
        	           		  {
        	           			  w = (2.0 * beta_max_ * T)*(error_theta / error_theta_mark);
        	           	         if(fabs(error_theta)<0.06)  theta_first_arrive_mark=true;
        	           		  }  
        	           		  else w = 0; 
        	        
        	           	  }
								 
        	           	 }
        	            // accelaration for x_movement
        	                                if(time < Tx)
        	                 	           	 {
        	                 	           	    vx_map_=(ax_max_/(2.0*Tx)) *(time * time);
        	                 	           		         
        	                 	           	 }
        	                 	           	else if(time<(2.0*Tx))
        	                 	           	 {
        	                 	           	    vx_map_=ax_max_*(time-Tx/2.0);
        	                 	           	 }
        	                 	           	 else if(time<(3.0*Tx))
        	                 	           	 {				  
        	                 	           	    vx_map_=ax_max_*(-(time *time)/(2.0*Tx) + 3.0*time - 2.5*Tx);
        	                 	           	 }
        	                 	           	 else
        	                 	           	 {
        	                 	           	  if(IS_LONG_WAY_x)
        	                 	           	  {
        	                 	                if((fabs(req.x - current_x_ -x_a_)>0.02) && (!x_mark))  // error limit need change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
        	                 	           	    {       	 double test;
        	                 	           	    test=req.x-current_x_-x_a_;
        	                 	               ROS_INFO("test %f ",
        	                 	                	      test);
        	                 	                	vx_map_= vx_max_;        	           	                      	
        	                 	           	    }
        	                 	                else 
        	                 	                {   
        	                 	                	x_mark=true;
        	                 	                	if(t2_vector_.size() == 0)
        	                 	                    {
        	                 	                	   t2_vector_.push_back(time);
        	                 	                 	   t2= time;  
        	                 	                    }	
        	                 	                    
//        	                 	                	if (time < (t2+Tx))
//        	                 	                	{  
//        	                 	                		vx_map_=ax_max_*(-((t2 + 3*Tx-time) *(t2+3*Tx-time))/(2.0*Tx) + 3.0*(t2+3*Tx-time) - 2.5*Tx);  	                		
//        	                 	                	}
//        	                 	                	else if (time < (t2+2*Tx))
//        	                 	                	{
//        	                 	                		vx_map_=ax_max_*(t2 + 3*Tx-time-Tx/2.0);
//        	                 	                	}
//        	                 	                	else if (time < (t2+3*Tx))
//        	                 	                	{
//        	                 	                		vx_map_=(ax_max_/(2.0*Tx)) *((t2+3*Tx-time)  * (t2+3*Tx-time) );
//        	                 	                	}
//        	                 	                	else vx_map_=0;
        	                 	                	
        	                 	                	if (!x_final_arrive_mark)
        	                 	                	{
        	                 	                	vx_map_ = (vx_max_ / x_a_) * (req.x - current_x_ - 0.02);
        	                 	                	  if(fabs(req.x - current_x_)<0.03)  x_first_arrive_mark=true;
        	                 	                	  
        	                 	                	}
        	                 	                	else vx_map_= 0;
        	                 	                }
        	                 	           	   }
        	                 	           	   else
        	                 	           	   {
        	                 	           		 if (error_x_vector_.size() == 0)
        	                 	           		 {
        	                 	           		   error_x_vector_.push_back(error_x);
        	                 	           		   error_x_mark= req.x - current_x_;
        	                 	           		 }
        	                 	           		 if(!x_final_arrive_mark)
        	                 	           		 {
        	               	           			   vx_map_ = (2.0 * ax_max_ * Tx) * (req.x - current_x_) / error_x_mark;
        	               	           			  if(fabs(req.x - current_x_)<0.03)  x_first_arrive_mark=true;
        	                 	           		 }  
        	                 	           		 else vx_map_= 0;
        	                 	           	   }
        	         								 
        	                 	           	 }
        	                                                                        if(time < Ty)
        	                                      	                 	           	 {
        	                                      	                 	           	    vy_map_=(ay_max_/(2.0*Ty)) *(time * time);
        	                                      	                 	           		         
        	                                      	                 	           	 }
        	                                      	                 	           	else if(time<(2.0*Ty))
        	                                      	                 	           	 {
        	                                      	                 	           	    vy_map_=ay_max_*(time-Ty/2.0);
        	                                      	                 	           	 }
        	                                      	                 	           	 else if(time<(3.0*Ty))
        	                                      	                 	           	 {				  
        	                                      	                 	           	    vy_map_=ay_max_*(-(time *time)/(2.0*Ty) + 3.0*time - 2.5*Ty);
        	                                      	                 	           	 }
        	                                      	                 	           	 else
        	                                      	                 	           	 { 
        	                                      	                 	           	  if(IS_LONG_WAY_y)
        	                                      	                 	           	  {
        	                                      	                 	                if((fabs(req.y - current_y_ -y_a_)>0.02) && (!y_mark))  // error limit need change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
        	                                      	                 	           	    {       	 
        	                                      	                 	                	double testy;
        	                                      	                 	           	        testy=req.y-current_y_-y_a_;
        	                                      	                 	                    ROS_INFO("testy %f ",
        	                                      	                 	                	      testy);
        	                                      	                 	                	vy_map_= vy_max_;        	           	                      	
        	                                      	                 	           	    }
        	                                      	                 	                else 
        	                                      	                 	                {   
        	                                      	                 	                	y_mark=true;
        	                                      	                 	                	if(t3_vector_.size() == 0)
        	                                      	                 	                    {
        	                                      	                 	                	   t3_vector_.push_back(time);
        	                                      	                 	                 	   t3= time;  
        	                                      	                 	                    }	
        	                                      	                 	                    
//        	                                      	                 	                	if (time < (t2+Tx))
//        	                                      	                 	                	{  
//        	                                      	                 	                		vx_map_=ax_max_*(-((t2 + 3*Tx-time) *(t2+3*Tx-time))/(2.0*Tx) + 3.0*(t2+3*Tx-time) - 2.5*Tx);  	                		
//        	                                      	                 	                	}
//        	                                      	                 	                	else if (time < (t2+2*Tx))
//        	                                      	                 	                	{
//        	                                      	                 	                		vx_map_=ax_max_*(t2 + 3*Tx-time-Tx/2.0);
//        	                                      	                 	                	}
//        	                                      	                 	                	else if (time < (t2+3*Tx))
//        	                                      	                 	                	{
//        	                                      	                 	                		vx_map_=(ax_max_/(2.0*Tx)) *((t2+3*Tx-time)  * (t2+3*Tx-time) );
//        	                                      	                 	                	}
//        	                                      	                 	                	else vx_map_=0;
        	                                      	                 	                	
        	                                      	                 	                	if (!y_final_arrive_mark)
        	                                      	                 	                	{
        	                                      	                 	                	vy_map_ = (vy_max_ / y_a_) * (req.y - current_y_ - 0.02);
        	                                      	                 	                	  if(fabs(req.y - current_y_)<0.03)  y_first_arrive_mark=true;
        	                                      	                 	                	  
        	                                      	                 	                	}
        	                                      	                 	                	else vy_map_= 0;
        	                                      	                 	                }
        	                                      	                 	           	  }
        	                                      	                 	              else
        	                                      	                 	              {
        	                                      	                 	            	if (error_y_vector_.size() == 0)
        	                                      	                 	            	{
        	                                      	                 	            	  error_y_vector_.push_back(error_y);
        	                                      	                 	            	  error_y_mark= req.y - current_y_;
        	                                      	                 	            	}
        	                                      	                 	            	if(!y_final_arrive_mark)
        	                                      	                 	            	{
        	                                      	                 	            	 vy_map_ = (2.0 * ay_max_ * Ty) * (req.y - current_y_) / error_y_mark;
        	                                      	                 	            	  if(fabs(req.y - current_y_)<0.03)  y_first_arrive_mark=true;
        	                                      	                 	            	}  
        	                                      	                 	            	else vy_map_= 0;  
        	                                      	                 	              }
        	                                      	                 	           	 }
        	             if((theta_first_arrive_mark == true) && (x_first_arrive_mark == true) && (y_first_arrive_mark == true))
        	             {  
        	            	 theta_final_arrive_mark = true;
        	            	 x_final_arrive_mark = true;
        	            	 y_final_arrive_mark = true;
        	             }
        	            
        	             ROS_INFO("rotation speed :  vx_map_ speed : vy_map_speed: %f %f %f",
        	           			      w, vx_map_, vy_map_);
        	           //velocity.linear.x = vx_map_ * (k * sin(current_theta_) + cos(current_theta_));
        	           //velocity.linear.y = vx_map_ * (k * cos(current_theta_) - sin(current_theta_));   
        	             velocity.linear.x = vy_map_ * sin(current_theta_) + vx_map_ * cos(current_theta_);
        	             velocity.linear.y = vy_map_ * cos(current_theta_) - vx_map_ * sin(current_theta_);
        	            //  velocity.linear.x= 0;
        	           //   velocity.linear.y =0;
        	                  	//      ROS_INFO("velocity.linear.x : velocity.linear.y:  %f %f",
        	                  	 //                 	            		 velocity.linear.x, velocity.linear.y);
        	             velocity.angular.z= 0;
        	           	 cmd_vel_pub.publish(velocity);
        	            
        	             r.sleep();
        }
        cmd_vel_pub.publish(geometry_msgs::Twist());
        ros::Duration(1).sleep();
        res.mark = 1;               //the position is reached!!
        return true;
    }
