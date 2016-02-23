#include <iostream>
#include <fstream>
#include <string.h>

#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <pcl_ros/point_cloud.h>

using namespace sensor_msgs;
using namespace std_msgs;

pcl::PointCloud<pcl::PointXYZI> cloudInitial;
pcl::PointCloud<pcl::PointXYZI> cloudFinal;


ros::Publisher pubBeta;

int numPoints;
int final_numPoints;
bool isInitPC = false;
bool isFinalPC = false;

int idPC=-1;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

void initSim()
{
    /***************************************
	 * initialise simulation
	 ***************************************/
}
 
 
void simulateEstimate()
{	
	/***************************************
	 * Simulate
	 * Estimate
	 ***************************************/
	 
	PointCloud::Ptr msg (new PointCloud);
  	msg->header.frame_id = "/map";
	
	msg->is_dense = false;
  	msg->points.resize (numPoints);  
  	msg->width    = msg->points.size (); 
  	msg->height   = 1;
 
 	float sum = 0.0;
 	for (int i = 0; i < numPoints; i++) {
        float x1 = cloudInitial.points[i].x;
        float y1 = cloudInitial.points[i].y;
        float z1 = cloudInitial.points[i].z;
        
        float x2 = cloudFinal.points[i].x;
        float y2 = cloudFinal.points[i].y;
        float z2 = cloudFinal.points[i].z;
        
        sum += sqrt(pow((x1-x2), 2) + pow((y1-y2), 2) + pow((z1-z2), 2));
        
    }
	
	sum = sum/numPoints;
	
	float beta_est = ((double) std::rand() / (RAND_MAX));	
		
	Float32MultiArray betaPub;
	betaPub.data.push_back(beta_est);
	betaPub.data.push_back(sum);
  	betaPub.data.push_back(idPC);
  	pubBeta.publish(betaPub);
}

//get the initial point cloud. Initialise the simulation
void cloud_cb_initial (const PointCloud2ConstPtr& input)
{
	//point cloud from ROS is obtained, turned to pcl point cloud
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::fromROSMsg (*input, cloud);
	cloudInitial = cloud;

	//id of the point cloud is obtained
	idPC = cloudInitial.points[0].data[3];

	numPoints = cloudInitial.size();

	isInitPC = true;

	std::string f_cur="";
	std::stringstream fPath;
	fPath.str("");
	fPath << "./initConfig.txt";
	f_cur = fPath.str();
	std::ofstream f;
	f.open(f_cur.c_str(), std::ofstream::out );

	f<<0<<" "<<0<<" "<<0<<"\n";
	f<<25<<" "<<25<<" "<<25<<"\n";
	f<<numPoints<<"\n";

	//create the initial configuration file	
	for(int i=0; i<numPoints; i++)
	{
		f <<cloudInitial.points[i].x<<" "<<cloudInitial.points[i].y<<" "<<cloudInitial.points[i].z<<" 0.100000\n";
	}
	f<<"0.01000"<<"\n"; //time step
	f<<"0.000000 0.000000 -9.870000\n";
	f<<"1.000000\n";
	f<<"0.000000\n";
	f<<"0\n";
	f<<"1\n";
	f<<"0\n";
	f<<"1\n";
	f<<"0.010000";
	
	f.close();

}

void cloud_cb_final ( const PointCloud2ConstPtr& input)
{	
	//point cloud from ROS is obtained, turned to pcl point cloud
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::fromROSMsg (*input, cloud);
	cloudFinal = cloud;

	//id of the point cloud is obtained
	int idPC = cloud.points[0].data[3];

	
	int final_numPoints = cloudFinal.size();

	isFinalPC = true;

	std::cout<<isInitPC<<" "<<isFinalPC<<" "<<numPoints<<" "<<final_numPoints<<std::endl;
	if(isInitPC && isFinalPC && (numPoints==final_numPoints)) 
	{
		std::cout<< "Final PC is obtained. NumPoints " << final_numPoints<<std::endl;
		simulateEstimate();
	}
}


int main (int argc, char** argv)
{
	ros::init (argc, argv, "send_beta");
	ros::NodeHandle nh;
	ROS_INFO("Starting ");
                
	ros::Subscriber sub1 = nh.subscribe ("points_init", 1, cloud_cb_initial);	
	ros::Subscriber sub2 = nh.subscribe ("points_def", 1, cloud_cb_final);
	
	//pubSim = nh.advertise<PointCloud> ("sim", 1);  
	pubBeta = nh.advertise<	Float32MultiArray> ("beta", 1);  
	
    // Spin
	ros::spin();
	
    return 0;
}
