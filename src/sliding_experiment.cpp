#include <ros/ros.h>
#include <bezier_curve/bezier_curve.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
// #include <kdl_conversions/kdl_msg.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <eigen3/Eigen/Geometry>
#include <lwr_controllers/PoseRPY.h>
#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>

void compute_orientation_from_vector(const Eigen::Vector3d& x_new, Eigen::Quaterniond& res)
{
    // double x_norm = x_new.Norm();
    double x_norm = x_new.dot(x_new);
    if (x_norm < 1e-16)
    {
        ROS_WARN_STREAM("The norm of the required ax is close to zero: not performing the rotation");
        res.setIdentity();
        return;
    }
    
    Eigen::Vector3d x_ax(1,0,0);
    res.setFromTwoVectors(x_ax,x_new);
    
//     std::cout << __func__ << " eigen setFromTwoVectors : xyz=[" << res.x() << " " << res.y() << " " << res.z() << "], w=" << res.w() << std::endl;
//     Eigen::Vector3d x_old_eig = res.inverse()*x_new_eig;
//     std::cout << __func__ << " new vector Eigen : " << x_new_eig[0] << " " << x_new_eig[1] << " " << x_new_eig[2] << " " << std::endl;
//     std::cout << __func__ << " old vector with Eigen : " << x_old_eig[0] << " " << x_old_eig[1] << " " << x_old_eig[2] << " " << std::endl << std::endl;
}

void printStuff(BezierCurve::PointVector& cp1, BezierCurve& bc, rviz_visual_tools::RvizVisualToolsPtr visual_tools_, const ros::Publisher& controller_pub, int num_samples)
{
    BezierCurve::PointVector cp1b;
    cp1b = cp1;
    bc.init_curve(cp1b);
	
	ros::Duration ts(3.0);
	
    for(int i=0; i<cp1.cols(); ++i)
        visual_tools_->publishSphere(cp1.col(i),rviz_visual_tools::RED,rviz_visual_tools::xxLARGE);
	
	ros::spinOnce();
	
    for(double t=0.99; t>=0.01; t-=0.98/num_samples)
    {
        // compute and publish a point on the curve
        BezierCurve::Point res = bc.compute_point(t);
        visual_tools_->publishSphere(res,rviz_visual_tools::CYAN,rviz_visual_tools::LARGE);
        
        // compute and publish the derivative along the curve
		BezierCurve::Point dres = bc.compute_derivative(t);
		Eigen::Quaterniond rot;
		compute_orientation_from_vector(dres,rot);
		Eigen::Affine3d pose;
		pose = Eigen::Translation3d(res)*rot;
		visual_tools_->publishArrow(pose);
		
// 		lwr_controllers::PoseRPY des_pose;
// 		des_pose.id = 0;
// 		Eigen::Vector3d rpy;
// 		rpy = pose.rotation().eulerAngles(0, 1, 2);
// 		des_pose.orientation.roll= rpy(0);
// 		des_pose.orientation.pitch = rpy(1);
// 		des_pose.orientation.yaw = rpy(2);
// 		des_pose.position.x = pose.translation()(0);
// 		des_pose.position.y = pose.translation()(1);
// 		des_pose.position.z = pose.translation()(2);
// 		controller_pub.publish(des_pose);
// 		ros::spinOnce();
// 		ts.sleep();
// 		tf::Pose pose_tf;
// 		KDL::Frame pose_kdl;
// 		tf::poseEigenToTF(pose,pose_tf);
// 		tf::poseTFToKDL(pose_tf,pose_kdl);
// 		std::cout << "t = " << t << std::endl;
// 		std::cout << "rpy = " << rpy << std::endl;
		
		geometry_msgs::PoseStamped des_pose_s;
		des_pose_s.header.seq = 0;
		des_pose_s.header.frame_id = "0";
		des_pose_s.header.stamp = ros::Time::now();
		geometry_msgs::Pose& des_pose(des_pose_s.pose);
		des_pose.position.x = res(0);
		des_pose.position.y = res(1);
		des_pose.position.z = res(2);
		des_pose.orientation.x = rot.x();
		des_pose.orientation.y = rot.y();
		des_pose.orientation.z = rot.z();
		des_pose.orientation.w = rot.w();
		
		controller_pub.publish(des_pose_s);
		ros::spinOnce();
		ts.sleep();
// 		std::cout << "t = " << t << std::endl;
// 		std::cout << des_pose << std::endl << std::endl;
    }
}

int main(int argc, char** argv)
{
    
    
//     Eigen::Vector3d v1_eig(1,0,0);
//     Eigen::Vector3d v2_eig(0,0,1);
//     
//     Eigen::Quaterniond q_eig;
//     q_eig.setFromTwoVectors(v1_eig,v2_eig);
//     
//     std::cout << __func__ << " eigen setFromTwoVectors : xyz=[" << q_eig.x() << " " << q_eig.y() << " " << q_eig.z() << "], w=" << q_eig.w() << std::endl;
//     Eigen::Vector3d v3_eig = q_eig.inverse()*v2_eig;
//     std::cout << __func__ << " old vector with Eigen : " << v3_eig[0] << " " << v3_eig[1] << " " << v3_eig[2] << " " << std::endl;
//     
//     
//     
//     return 0;
    
    while(!ros::isInitialized())
    {
        ros::init(argc,argv,"sliding_experiment");
    }
    ros::NodeHandle nh;
    ros::AsyncSpinner aspin(1);
    aspin.start();
    
    // For visualizing things in rviz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers"));
//     visual_tools_->enableBatchPublishing();
	
// 	ros::Publisher controller_pub = nh.advertise<lwr_controllers::PoseRPY>("/lwr/one_task_inverse_kinematics/command",100,false);
	ros::Publisher controller_pub = nh.advertise<geometry_msgs::PoseStamped>("/lwr/one_task_inverse_kinematics/command",100,false);
    
    sleep(2);
    
    // construct bezier object here
    BezierCurve bc;
    
    std::cout << "sliding_experiment running!!!" << std::endl;
    
	// bring the arm to a known position
// 	lwr_controllers::PoseRPY init_pose;
	geometry_msgs::PoseStamped init_pose;
	init_pose.header.seq = 0;
	init_pose.header.frame_id = "0";
	init_pose.header.stamp = ros::Time::now();
	init_pose.pose.position.x = -0.4;
	init_pose.pose.position.y = 0.5;
	init_pose.pose.position.z = 0.6;
	init_pose.pose.orientation.w = 1.0;
	
	controller_pub.publish(init_pose);
	sleep(2);
	
    // test 1
    BezierCurve::PointVector cp1;
    cp1.resize(cp1.RowsAtCompileTime,4);
    cp1 << -0.4,-0.3,-0.3,-0.4,
           0.5, 0.5, 0.3, 0.3,
           0.6, 0.6, 0.6, 0.6;
// 	cp1 << -0.4,-0.3,-0.3,-0.4,
//             0.3, 0.3, 0.5, 0.5,
//             0.6, 0.6, 0.6, 0.6;
	
// 	std::cout << cp1 << std::endl;
	printStuff(cp1,bc,visual_tools_,controller_pub,20);
    
//     // test 2
//     cp1 << 0, 2,-1, 1,
//            0, 1, 1, 0,
//            0, 1, 1, 0;
//     printStuff(cp1,bc,visual_tools_,controller_pub,20);
//     
//     // test 3
//     cp1 << 0, 0, 1, 1,
//            0, 1, 1, 0,
//            1, 1, 1, 1;
//            printStuff(cp1,bc,visual_tools_,controller_pub,20);
//            
//     // test 4
//     cp1.resize(cp1.RowsAtCompileTime,6);
//     cp1 << 0, 1, 2, 2, 1, 0,
//            0, 1, 1, 0, 0, 1,
//            2, 3, 4, 4, 3, 2;
//     printStuff(cp1,bc,visual_tools_,controller_pub,20);
//     
//     if(!visual_tools_->triggerBatchPublish())
//     {
//         ROS_WARN_STREAM("Unable to publish batch markers!");
//     }
//     else
//     {
//         ROS_INFO_STREAM("Done publishing!");
//     }
    
    ros::Rate rate(0.1);
    int counter = 0;
    while(ros::ok())
    {
        rate.sleep();
        if(++counter > 10)
            break;
    }
    aspin.stop();
    ros::shutdown();
    
    return 0;
}