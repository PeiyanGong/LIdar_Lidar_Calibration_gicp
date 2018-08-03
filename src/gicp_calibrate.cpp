#include "gicp_calibrate.h"

#define PI 3.14159265

void callback_lidar_sync(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_1, 
                        const sensor_msgs::PointCloud2ConstPtr& cloud_msg_2){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*cloud_msg_1,*ptrCloud_1);
    pcl::fromROSMsg(*cloud_msg_2,*ptrCloud_2);

    //Crop down ptrCloud_1
    // pcl::PassThrough<pcl::PointXYZ> pass_1;
    // pass_1.setInputCloud (ptrCloud_1);
    // pass_1.setFilterFieldName ("y");
    // pass_1.setFilterLimits (3.5, 10.0);
    // pass_1.setFilterFieldName ("z");
    // pass_1.setFilterLimits (-2.0, 0.0);
    //pass.setFilterLimitsNegative (true);
    // pass_1.filter (*cloud_filtered_1);
    pcl::CropBox<pcl::PointXYZ> boxFilter_1;
    // boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    // boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter_1.setMin(Eigen::Vector4f(-2.0, 4.0, -2.0, 1.0));
    boxFilter_1.setMax(Eigen::Vector4f(10.0, 20.0, 0.0, 1.0));
    boxFilter_1.setInputCloud(ptrCloud_1);
    boxFilter_1.filter(*cloud_filtered_1);


    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 _output;

    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity ();

    Eigen::Matrix4f guess_ns1 = Eigen::Matrix4f::Identity (); // -90 on z axis for ns1, -60 on z axis for ns2
    guess_ns1 <<  -0.0705702,   0.970619,   -0.23004,   0.849821,
     -0.967564,  -0.122688,  -0.220838,    2.29104,
     -0.242571,   0.206993,   0.947793,   -1.23452,
            0,         0,         0,         1;
    /*double theta_1 = -60.0*3.14159265/180.0;
    guess_ns1 << cos(theta_1),-sin(theta_1),0.0,0.0,
                 sin(theta_1), cos(theta_1),0.0,0.0,
                 0.0  ,0.0   ,1.0,0.0,
                 0.0  ,0.0   ,0.0,1.0;*/
    // for ns03 
    // guess_ns1 <<     0.954763,  0.00484971,    0.297328,    0.480585,
    //             -0.00670342,    0.999964,  0.00521546,  0.00633954,
    //               -0.297292, -0.00697265,    0.954761,   0.0019191, 
    //               0.0,           0.0,          0.0,           1.0; 
    /*guess_ns1 <<  -0.0619962,   0.971839,  -0.227346,   0.878283, // For ns1
                 -0.968364,  -0.113736,  -0.222119,    2.29353,
                 -0.241719,   0.206383,   0.948144,   -1.22764,
                         0,          0,          0,          1; */
    /*
    ns1 final result
    Final result: 
     -0.07775  0.971848 -0.222408  0.813054
    -0.966705 -0.128043 -0.221559   2.22258
    -0.243798  0.197776  0.949445  -1.24362
            0         0         0         1

    */

    /*
    ns2 final result
    has converged:1 score: 0.0414148
     Final result: 
     0.412873  0.878207  0.241429 -0.787354
    -0.871087   0.45816 -0.176906   2.20536
    -0.265974 -0.137267  0.954158  -1.25976
        0         0         0         1

    */
//  ns3 on Denso Data
//  Final result: 
//   0.944936  0.0217603  -0.326531  -0.455621
// -0.0138125   0.999549  0.0266392  0.0180255
//   0.326963 -0.0206622   0.944811 -0.0503514
//          0          0          0          1
// ns5 on Denso Data
// Final result: 
//    0.954769     0.00478     0.29731    0.480566
// -0.00665779    0.999964  0.00530383   0.0066282
//   -0.297274 -0.00704337    0.954766  0.00189982
//           0           0           0           1


    /*
    ns3 final result
     0.940413   0.0348746   -0.338241   -0.473188
    -0.031771    0.999387   0.0147095   0.0110943
     0.338547 -0.00308675    0.940944   -0.114471
            0           0           0           1

    */

    /*
    ns5 final result
    0.958359  0.0174144   0.285036   0.471749
    -0.0100379   0.999576 -0.0273196 -0.0179415
    -0.285391  0.0233208   0.958127 -0.0886846
         0          0          0          1

    */
    pcl::toROSMsg(*cloud_filtered_1, _output);
    pub_velo_wo_speed.publish(_output);

    pcl::transformPointCloud (*ptrCloud_2, *cloud_filtered_2, guess_ns1); // Transform initial guess manually ** Works better

    // pcl::PassThrough<pcl::PointXYZ> pass_2;               // For fine matching
    // pass_2.setInputCloud (cloud_filtered_2);
    // pass_2.setFilterFieldName ("y");
    // pass_2.setFilterLimits (3.5, 10.0);
    // pass_2.setFilterFieldName ("z");
    // pass_2.setFilterLimits (-2.0, 0.0);
    // //pass.setFilterLimitsNegative (true);
    // pass_2.filter (*cloud_filtered_2);
    pcl::CropBox<pcl::PointXYZ> boxFilter_2;
    // boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    // boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter_2.setMin(Eigen::Vector4f(-2.0, 4.0, -2.0, 1.0));
    boxFilter_2.setMax(Eigen::Vector4f(10.0, 20.0, 0.0, 1.0));
    boxFilter_2.setInputCloud(cloud_filtered_2);
    boxFilter_2.filter(*cloud_filtered_2);

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //pcl::registration::DefaultConvergenceCriteria<float>::Ptr criteria = icp.getConvergeCriteria();
    //icp.convergence_criteria_->setRotationThreshold(0.1);
    //icp.convergence_criteria_->setTranslationThreshold(1e-08);
    //icp.convergence_criteria_->setRelativeMSE(1e-12);
    //icp.convergence_criteria_->setAbsoluteMSE(1e-14);
    std::cout << icp.convergence_criteria_->getRotationThreshold()<< std::endl;
    std::cout << icp.convergence_criteria_->getTranslationThreshold()<< std::endl; 
    std::cout << icp.convergence_criteria_->getRelativeMSE()<< std::endl; 
    std::cout << icp.convergence_criteria_->getAbsoluteMSE()<< std::endl;  
    icp.setRANSACIterations(20);
    icp.setInputSource(cloud_filtered_2);
    icp.setInputTarget(cloud_filtered_1);
    icp.setMaxCorrespondenceDistance (3.0);
    icp.setMaximumOptimizerIterations(500);
    std::cout << "start aligning "<< std::endl;
    //icp.align(*Final, guess_ns1); // For some reason, the pointcloud behind ptr Final is the same with Inputsource pcl
    icp.align(*Final);
    transformation_matrix = icp.getFinalTransformation().cast<float>()*guess_ns1;

    
    
    pcl::transformPointCloud (*ptrCloud_2, *Final, transformation_matrix); // Do transformation in order to viz result.
    //std::cout << "Convergence state: " << icp.convergence_criteria_->getConvergenceState() <<std::endl;
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << "Final result: " <<std::endl<< transformation_matrix << std::endl;
    //std::cout << "Last iter: " << icp.getLastIncrementalTransformation() << std::endl;
    std::cout << "Publishing "<< std::endl;
    
    pcl::toROSMsg(*Final, output);
    pub_velo.publish(output);
    std::cout << "MaxOptimizerIter:" << icp.getMaximumOptimizerIterations() << std::endl;
    std::cout << "RANSACIter:" << icp.getRANSACIterations() << std::endl;
    std::cout << "getMaxCorrespondenceDistance: " << icp.getMaxCorrespondenceDistance() << std::endl;
    while(1);
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "velodyne_sub");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_1(nh, "ns4/velodyne_points", 10);  // Should be unchanged. ns4 is the center one
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_2(nh, "ns1/velodyne_points", 10);
 
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_1, sub_2);
  sync.registerCallback(boost::bind(&callback_lidar_sync, _1, _2));
  
  pub_velo = nh.advertise<sensor_msgs::PointCloud2>("/vlp_fuse", 1);
  pub_velo_wo_speed = nh.advertise<sensor_msgs::PointCloud2>("/vlp_fuse_wo_speed", 1);

  ros::spin();
  // Spin
}
