#include <ros/ros.h>
#include <bezier_curve/bezier_curve.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_conversions/kdl_msg.h>
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
}

inline void waitForInput()
{
    char y;
    std::cout << "waiting for input" << std::endl;
    std::cin >> y;
}

void printStuff(BezierCurve::PointVector& cp1, BezierCurve& bc, rviz_visual_tools::RvizVisualToolsPtr visual_tools_, const ros::Publisher& controller_pub, int num_samples)
{
    BezierCurve::PointVector cp1b;
    cp1b = cp1;
    bc.init_curve(cp1b);
    
    ros::Duration ts(0.05);
    
    for(int i=0; i<cp1.cols(); ++i)
        visual_tools_->publishSphere(cp1.col(i),rviz_visual_tools::RED,rviz_visual_tools::xxLARGE);
    
    ros::spinOnce();
    waitForInput();

    for(double t=0.0; t<=1.0; t+=1.0/num_samples)
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
        
        pose = pose*Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitX()) ;
        visual_tools_->publishAxis(pose);

        geometry_msgs::PoseStamped des_pose_s;
        des_pose_s.header.seq = 0;
        des_pose_s.header.frame_id = "0";
        des_pose_s.header.stamp = ros::Time::now();
        tf::Pose des_tf;
        tf::poseEigenToTF(pose,des_tf);
        tf::poseTFToMsg(des_tf,des_pose_s.pose);
        
        controller_pub.publish(des_pose_s);
        ros::spinOnce();
        ts.sleep();
        waitForInput();
    }
}

int main(int argc, char** argv)
{
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

    ros::Publisher controller_pub = nh.advertise<geometry_msgs::PoseStamped>("/right_arm/one_task_inverse_kinematics/command",100,false);
    
    sleep(2);
    
    // construct bezier object here
    BezierCurve bc;
    
//     // test 1 - working in mid air
//     BezierCurve::PointVector cp1;
//     cp1.resize(cp1.RowsAtCompileTime,4);
//     cp1 << -0.8,-1.0,-1.0,-0.8,
//            0.1, 0.1, 0.3, 0.3,
//            0.35, 0.35, 0.35, 0.35;
    
    // test 2
    BezierCurve::PointVector cp1;
    cp1.resize(cp1.RowsAtCompileTime,4);
    cp1 << -0.8,-1.0,-1.0,-0.8,
            0.1, 0.1, 0.3, 0.3,
            0.2, 0.2, 0.2, 0.2;
 
    std::cout << "sliding_experiment running!!!" << std::endl;
    
    // bring the arm to a known position
    geometry_msgs::PoseStamped init_pose;
    init_pose.header.seq = 0;
    init_pose.header.frame_id = "0";
    init_pose.header.stamp = ros::Time::now();
    // use the same position as the first Bezier control point
    init_pose.pose.position.x = cp1(0,0);
    init_pose.pose.position.y = cp1(1,0);
    init_pose.pose.position.z = cp1(2,0);
    // orient the hand pointing forward
    init_pose.pose.orientation.w = 0.0;
    init_pose.pose.orientation.x = 0.0;
    init_pose.pose.orientation.y = 0.0;
    init_pose.pose.orientation.z = 1.0;
    
    // apply transformations due to the hand frame
    KDL::Frame KDL_init_position;
    tf::poseMsgToKDL(init_pose.pose, KDL_init_position);
    KDL_init_position.M.DoRotZ(-0.5*M_PI);
    KDL_init_position.M.DoRotX(-0.5*M_PI);
    tf::poseKDLToMsg(KDL_init_position, init_pose.pose);
    
    controller_pub.publish(init_pose);
    sleep(2);

    printStuff(cp1,bc,visual_tools_,controller_pub,8);
    
    std::cout << "Trajectory performed" << std::endl;
    
    ros::Rate rate(1);
    int counter = 0;
    while(ros::ok())
    {
        rate.sleep();
        if(++counter > 2)
            break;
    }
    aspin.stop();
    ros::shutdown();
    
    return 0;
}