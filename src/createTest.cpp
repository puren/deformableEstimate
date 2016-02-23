#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>  
#include <iostream>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>


using namespace std;


typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

int main(int argc, char** argv)
{
	if(argc<3)
	{
		std::cout<<"createTest [path-to-pcd1] [path-to-pcd2]"<<std::endl;
		return -1;
	}

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  
  std::string path1(argv[1]);
  std::string path2(argv[2]);

  //"/media/Raid/home/puren/data/Dataset_deformability2/Dataset_def_points_Foam1_1/ID_initial_prec_020_.pcd"-->less deformable
  //"/media/Raid/home/puren/data/Dataset_deformability/Dataset_1_foam_1/ID_01_initial.pcd"-->more deformable
  if(pcl::io::loadPCDFile<pcl::PointXYZ> (path1.c_str(), *cloud1) == -1)
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud1->width * cloud1->height
            << " data points from initial.pcd"
            << std::endl;
                  
   if (pcl::io::loadPCDFile<pcl::PointXYZ> (path2.c_str(), *cloud2) == -1)
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  
  std::cout << "Loaded "
            << cloud2->width * cloud2->height
            << " data points from final.pcd  "
            << std::endl;
           
  
              
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  
  ros::Publisher pub1 = nh.advertise<PointCloud> ("points_init", 1);
  ros::Publisher pub2 = nh.advertise<PointCloud> ("points_def", 1);
  
  vector<Eigen::Vector3d> vInit;
  vector<Eigen::Vector3d> vFinal;
  
  Eigen::Vector3d pos1;
  Eigen::Vector3d pos2;
  
  int size = cloud1->size();
  int width = sqrt(size);
  int height = width;
  
  std::cout<<"size: "<<size<<std::endl;
  
  //find pushed particle and boundaries 	
  float min_x=10000, max_x=0.0, min_y=10000, max_y=0;
  vector<float> v_push;
  
   
  PointCloud::Ptr msg_init (new PointCloud);
  msg_init->header.frame_id = "/map";
  
  PointCloud::Ptr msg_def (new PointCloud);
  msg_def->header.frame_id = "/map";
  
  msg_init->is_dense = false;
  msg_init->points.resize (width*height);  
  msg_init->width    = width*height; 
  msg_init->height   = 1;
  
  msg_def->is_dense = false;
  msg_def->points.resize (width*height);  
  msg_def->width    = width*height; 
  msg_def->height   = 1;
  	
  int p=0;
  for (size_t k = 0; k < size; k++, p++) {
  /*for (size_t i = 0; i < height;i++) 
    for (size_t j = 0; j < width; j++, p++) 
    { 
      	int k=i*23 + j;*/
      
      	pos1(0) = cloud1->points[k].x; 
      	pos1(1) = cloud1->points[k].z; 
      	pos1(2) = cloud1->points[k].y;
        //vInit.push_back(pos1);
        	
        pos2(0) = cloud2->points[k].x; 
        pos2(1) = cloud2->points[k].z; 
        pos2(2) = cloud2->points[k].y;
        vFinal.push_back(pos2);
        	
        std::cout<<k<<" "<< pos1(0)<<" "<< pos1(1)<<" " <<pos1(2)<<std::endl;
        
        v_push.push_back(pos1(2) - pos2(2));
	  
	  	if(pos1(0)>max_x)
	  	{
	  		max_x = pos1(0);
	  	}
	  	if(pos1(0)<min_x)
	  	{
	  		min_x = pos1(0);
	  	}
	  
	  	if(pos1(1)>max_y)
	  	{
	  		max_y = pos1(1);
	  	}
	  	if(pos1(1)<min_y)
	  	{
	  		min_y = pos1(1);
	  	}
        
      	msg_init->points[p].x = pos1(0);
   	  	msg_init->points[p].y = pos1(1);
      	msg_init->points[p].z = pos1(2);
    	
      	msg_def->points[p].x = pos2(0);
      	msg_def->points[p].y = pos2(1);
   	  	msg_def->points[p].z = pos2(2);
    } 
      
	float index_pushed = distance(v_push.begin(), max_element(v_push.begin(), v_push.end()));
	float pos_pushed = vFinal.at(index_pushed)(2);
	
	std:cout<<"pushed index "<<index_pushed<<" pos pushed:"<<pos_pushed<<std::endl;
  
  	ros::Publisher pubPushed = nh.advertise<std_msgs::Float32MultiArray> ("pushed", 1);
  	std_msgs::Float32MultiArray pushedPart;
  	pushedPart.data.push_back(index_pushed);
  	pushedPart.data.push_back(pos_pushed);
  	pushedPart.data.push_back(min_x);
  	pushedPart.data.push_back(max_x);
  	pushedPart.data.push_back(min_y);
  	pushedPart.data.push_back(max_y);
  	pushedPart.data.push_back(height);
  	pushedPart.data.push_back(width);
  	
  	ros::Rate loop_rate(4);
  	while (nh.ok())
  	{
		msg_init->header.stamp = ros::Time::now().toNSec();
		pub1.publish (msg_init);
		
		msg_def->header.stamp = ros::Time::now().toNSec();
		pub2.publish (msg_def);
		
		pubPushed.publish(pushedPart);
		
		ros::spinOnce ();
		loop_rate.sleep ();
  	}
}
