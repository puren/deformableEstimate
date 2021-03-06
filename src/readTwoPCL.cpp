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

#include "opencv2/opencv.hpp"
#include <cv.h>
#include "opencv2/highgui/highgui.hpp"

//SM simulation
#include "deformable.h"
#include <time.h>


using namespace sensor_msgs;
using namespace std_msgs;
using namespace std;

pcl::PointCloud<pcl::PointXYZI> cloudInitial;
pcl::PointCloud<pcl::PointXYZI> cloudFinal;

bool isInitPC = false; //true if initial point cloud arrives
bool isFinalPC = false; //true if final point cloud arrives
bool isInSim = false; //true if both  point clouds are obtained and simulation starts
bool isFirst = true; //true if new initial point cloud arrives
int count = 0;

int initID = 0;
int finalID = 0;

int numPoints=-1;

Deformable *mDeformable;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

int width = 0;
int height = 0;
int depth = 0;
;
double worldToScreen = 600; //1.0;
double screenToWorld = 1.0/worldToScreen;

ros::Publisher pubSim;
ros::Publisher pubBeta;

Eigen::Vector3d posF1(0, 0, 0);
int idF1 = 0;
float pushed_z=0;

/********set fixed boundary**************/
double bt_x=0, bt_y=0, bt_z=0.0, bb_x=0, bb_y=0, bb_z=0;
int m, n;
int idPC=-1;

float max_y;
float min_y;
float max_x;
float min_x;
float pos_pushed;
float index_pushed;
bool isPushInfo = false;

//simulation setting
int isCluster;
int allowFlip;
int volConv;
int quadratic;
float timeStep;
int w;
Eigen::Vector3i c(0, 0, 0);

float Arr[10];

void initSim()
{  
	std::cout<<"m: "<<m<<"n: "<<n<<std::endl;
	
	if(isPushInfo)
	{
		
		
		mDeformable->initState();
		for (int i=0; i<numPoints; i++) {
		    Eigen::Vector3d pos;
		   	pos = mDeformable->getOriginalVertexPos(i);
		    //pos(2) = cloudInitial.points[i].z;
			if( (fabs(pos(0)-bt_x)<1e-3 || fabs(pos(0)-bb_x)<1e-3) || (fabs(pos(1)-bt_y)<1e-3 || fabs(pos(1)-bb_y)<1e-3) )
			{
				mDeformable->fixVertex(i, pos);
				
				//std::cout<<"***"<<pos(0)<< " "<<pos(1)<<std::endl;
			}
			
			
		}         
	
		posF1 = mDeformable->getOriginalVertexPos(idF1);
		posF1(2) = pushed_z;		
		
		
		
		
    }	
}
 
void simulateEstimate()
{
	isInSim = true;
    
    std::vector<double> dist;
    std::vector<double> beta;
    double range = 0.1;
    double x, y, z, px, py, pz, min_err = 0.0, min_beta = 0.0, resToStop = 0.01, max_beta = 0.99, beta_est;
	int  min_ind= 0, finish=0;
	
	mDeformable->params.beta=0.0;

	Eigen::Vector3d pos;
	
    mDeformable->params.bounds.min(2) = mDeformable->getOriginalVertexPos(0)(2);
   	height = mDeformable->params.bounds.max(1)*worldToScreen;
   	width = mDeformable->params.bounds.max(0)*worldToScreen;
   	depth = mDeformable->params.bounds.max(2)*worldToScreen;
   	
   	cv::namedWindow( "View1", CV_GUI_EXPANDED );
	cv::namedWindow( "View2", CV_GUI_EXPANDED );
    cv::Mat image(cv::Size(width,height),CV_8UC3);
    cv::Mat image2(cv::Size(width,depth+100),CV_8UC3);
    
   	
   	std::cout<<"ground boundary: "<<mDeformable->params.bounds.min(2)<<std::endl;
   	
   	clock_t t1,t2;
   	
   	t1=clock();

   	
	while (std::abs(range)>0.001) 
	{
	   	while(mDeformable->params.beta<=max_beta && mDeformable->params.beta>=min_beta)
		{
            
        	int k=0;
        
        	initSim();
        
       	 	double mMassToRadius = 0.0001;
        
        	finish=0;
        
        	float z1 = 0.0;
        	float z2 = 0.01;
		       
		      
	    	while(1)
	    	{
	        	image = cv::Mat::zeros(height,width,CV_8UC3);
	        	image2 = cv::Mat::zeros(depth+100, width,CV_8UC3);
	            cv::line( image2,
	                     cv::Point(0,depth-(mDeformable->params.bounds.min(2)*worldToScreen)),cv::Point(width,depth-(mDeformable->params.bounds.min(2)*worldToScreen)),
	                   cv::Scalar( 255,255 , 255 ),1);
	                   
	            double res=0.0;
	            
	            if(k>2)
				{
					int step = mDeformable->params.n;
					//int w = 1;
					mDeformable->fixVertex(idF1, posF1);
					y=idF1%step;
					x=idF1/step;
				
					for(int i=x-w; i>=0 && i<=x+w && i<numPoints; i+=w)
					{
						for(int j=y-w; j>=0 && j<=y+w && j<numPoints; j+=w)
						{
							int idx = i*step+j;
							pos = mDeformable->getOriginalVertexPos(idx);
							pos(2) =posF1(2);
							mDeformable->fixVertex(idx, pos);
						}
					}
				}
				
		          
	            for (int i = 0; i < numPoints; i++) {
	                pos = mDeformable->getVel(i);
	                res += sqrt(pow(pos(0),2) + pow(pos(1),2) + + pow(pos(2),2));
	            }
	            res=res/numPoints;
	            
	            if(res<resToStop)
	            {
	                finish++;
	            }
	            else
	            {
	                finish=0;
	            }
		            
	            for( int i = 0; i<numPoints; i++)
	            {
	                pos = mDeformable->getVertexPos(i);
	                
	                double r = sqrtf(mDeformable->getMass(i) * mMassToRadius) * worldToScreen;
	                x = pos(0)*worldToScreen;
	                y = pos(1)*worldToScreen;
	                z = pos(2)*worldToScreen;
	                
	                y = height - y;
	                z = depth - z;
	                
	                circle(image2, cv::Point(x-r, z-r), r, cv::Scalar(255, 255, 255), 1);
	                circle(image, cv::Point(x-r, y-r), r, cv::Scalar(255, 255, 255), -1);
	                	
	            	double x1=cloudFinal.points[i].x*worldToScreen;
	            	double y1=cloudFinal.points[i].y*worldToScreen;
	            	double z1=cloudFinal.points[i].z*worldToScreen;
	                
	            	z1=depth-z1;
	            	y1=height-y1;
	            	circle(image2, cv::Point(x1-r, z1-r), r, cv::Scalar(0, 0, 255), 1);
	            	
	            	if(mDeformable->isFixed(i))
			        {
			        	circle(image, cv::Point(x-r, y-r), r, cv::Scalar(0, 255, 0), -1);
			        }
	            }
		                
	            
		       
		        imshow( "View1", image);
		        imshow( "View2", image2);
		                
		        int key=cvWaitKey(1);
                
		        switch(key){
		        	case 'q':
		        		exit(1);
		        }
            
		        if((finish>10 && k>3))
		        {
                	break;
            	}
            
            	mDeformable->timeStep();
            
            	k++;
        	}            
           
		    double err = 0.0, err_z = 0.0;
		    for (int i = 0; i < numPoints; i++) {
		        
		        x = cloudFinal.points[i].x;
		        y = cloudFinal.points[i].y;
		        z = cloudFinal.points[i].z;
		        
		        pos = mDeformable->getVertexPos(i);
		        px = pos(0); py = pos(1); pz = pos(2);
		        
		        err += sqrt(pow((x-px), 2) + pow((y-py), 2) + pow((z-pz), 2));
		        err_z += sqrt(pow((z-pz), 2));
            
        	}
        	err = err / numPoints;
        	err_z = err_z / numPoints;
            
        	dist.push_back(err);
        	beta.push_back(mDeformable->params.beta);      
         
        	std::cout<<"beta:"<<mDeformable->params.beta<<" error:"<<err<<" error in z: "<<err_z<<std::endl;
        
        	mDeformable->params.beta += range;
        	
        	PointCloud::Ptr msg (new PointCloud);
  	msg->header.frame_id = "/map";
	
	msg->is_dense = false;
  	msg->points.resize (numPoints);  
  	msg->width    = msg->points.size (); 
  	msg->height   = 1;
 
 	for (int i = 0; i < numPoints; i++) {
        pos = mDeformable->getVertexPos(i);
        msg->points[i].x = pos(0);
        msg->points[i].y = pos(1);
        msg->points[i].z = pos(2);
    }
    
    pubSim.publish (msg);
        
    	}		
   		
		std::vector<double>::iterator result = std::min_element(dist.begin(), dist.end());
		min_ind =  std::distance(dist.begin(), result);
		beta_est = beta.at(min_ind);
		min_err = dist.at(min_ind);
	 
		int N = dist.size();
		if (min_ind<N-1 && min_ind>0) {

			if (std::abs(dist.at(min_ind)-dist.at(min_ind-1)) < std::abs(dist.at(min_ind+1)-dist.at(min_ind))) {
				range = -range/10;
				min_beta = beta.at(min_ind-1);
				max_beta = beta.at(min_ind+1);
				
			}
			else if(std::abs(dist.at(min_ind)-dist.at(min_ind-1)) > std::abs(dist.at(min_ind+1)-dist.at(min_ind)))
			{
				range = range/10;
				min_beta = beta.at(min_ind-1);
				max_beta = beta.at(min_ind+1);
			   
			}
		
		}
		else if (min_ind == N-1)
		{
			range = -range/10;
			min_beta = beta.at(N-2);
			max_beta = beta.at(N-1);
		
		}
		else if (min_ind == 0)
		{
			range = range/10;
			min_beta = beta.at(0);
			max_beta = beta.at(1);
		   
		}
		dist.clear();
		beta.clear();
		mDeformable->params.beta = beta_est;
	
		//std::cout<<"the estimated beta is "<<beta_est<<" range  "<<range<<std::endl;
	}
    
    t2=clock();
    float diff ((float)t2-(float)t1);
    cout<<"time for estimaton: "<<diff/CLOCKS_PER_SEC<<endl;

    
    std::cout<<"\nEstimatin is finished\n";
       
    
    
    std::cout<<"\nSimulated point cloud is published\n";
	
	Float32 betaPub;
	betaPub.data =beta_est;
	
	pubBeta.publish(betaPub);
	
	std::cout<<"\nBeta is published "<<beta_est<<"\n";
}

void cb_pushInfo(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	int i=0;
	
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Arr[i] = *it;
		i++;
	}
	
	idF1 = Arr[0];
	pushed_z = Arr[1];
	bt_x=Arr[2], bt_y=Arr[4], bb_x=Arr[3], bb_y=Arr[5];
	m = Arr[6];
	n = Arr[7];
	
		
	
	isPushInfo = true;
}

//get the initial point cloud. Initialise the simulation
void cloud_cb_initial (const PointCloud2ConstPtr& input)
{
  	if(isPushInfo)
  	{
		//point cloud from ROS is obtained, turned to pcl point cloud
		pcl::PointCloud<pcl::PointXYZI> cloud;
		pcl::fromROSMsg (*input, cloud);
		cloudInitial = cloud;
	
		//id of the point cloud is obtained
		idPC = cloudInitial.points[0].data[3];
	
		//set the initial point cloud ID
		initID = idPC;
	
		numPoints = cloudInitial.size();
	
		isInitPC = true;
	
		std::string f_cur="";
		std::stringstream fPath;
		fPath.str("initConfig.txt");
		//fPath << "/media/Raid/home/puren/catkin_ws/src/deformablePCL/data/initConfig.txt";
		f_cur = fPath.str();
		std::ofstream f;
		f.open(f_cur.c_str(), std::ofstream::out );
	
		f<<0<<" "<<0<<" "<<0<<"\n";
		f<<1<<" "<<1<<" "<<1<<"\n";
		f<<numPoints<<"\n";
	
		//create the initial configuration file	
		for(int i=0; i<cloudInitial.size(); i++)
		{
			f <<cloudInitial.points[i].x<<" "<<cloudInitial.points[i].y<<" "<<cloudInitial.points[i].z<<" 0.100000\n";
		}
		
		/*for(int i=1; i<4; i++)
		{
			f <<cloudInitial.points[i].x<<" "<<cloudInitial.points[i].y<<" "<<cloudInitial.points[i].z-(0.01*i)<<" 0.100000\n";
			numPoints++;
		}*/
		f<<std::setprecision(6)<<timeStep<<"\n";
		f<<"0.000000 0.000000 -9.870000\n";
		f<<"1.0\n";
		f<<"0.000000\n";
		f<< quadratic <<"\n";
		f<< volConv <<"\n";
		f<< allowFlip <<"\n";
		f<< isCluster <<"\n";
		
		/*f<<"0.002000"<<"\n"; //time step
		f<<"0.000000 0.000000 -9.870000\n";
		f<<"0.990000\n";
		f<<"0.000000\n";
		f<<"0\n";
		f<<"0\n";
		f<<"1\n";
		f<<"1\n";
		f<<"0.010000";*/
		
		f.close();

		mDeformable = new Deformable();
		mDeformable->params.m = m;
		mDeformable->params.n = n;
		mDeformable->params.p = 1;
		
		mDeformable->params.cluster_size(0) = c(0);
		mDeformable->params.cluster_size(1) = c(1);
		mDeformable->params.cluster_size(2) = c(2);
		
		mDeformable->loadFromFile("initConfig.txt");
	
	}
}

void cloud_cb_final ( const PointCloud2ConstPtr& input)
{	
	//point cloud from ROS is obtained, turned to pcl point cloud
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::fromROSMsg (*input, cloud);
	cloudFinal = cloud;

	//id of the point cloud is obtained
	int idPC = cloud.points[0].data[3];

	//set the final point cloud ID
	finalID = idPC;

	int final_numPoints = cloudFinal.size();

	isFinalPC = true;

	std::cout<<"First Pc: "<<isInitPC<<" Last PC: "<<isFinalPC<<" Num of points: "<<numPoints<<" "<<"Final num of points: "<<final_numPoints<<std::endl;
	if(isInitPC && isFinalPC && (numPoints==final_numPoints) && isPushInfo)  
	{
		std::cout<< "Final PC is obtained. NumPoints " << final_numPoints<<std::endl;
		simulateEstimate();
	}
}

int main (int argc, char** argv)
{
	if(argc<10)
	{	
		std::cout<<"readTwoPCL [isCluster] [allowFlip] [volConv] [quadratic] [timeStep] [w] [cluster-x] [cluster-y] [cluster-z]"<<std::endl;
		return -1;
	}
	
	isCluster = atoi(argv[1]);
	allowFlip = atoi(argv[2]);
	volConv = atoi(argv[3]);
	quadratic = atoi(argv[4]);
	timeStep = atof(argv[5]);
	w = atoi(argv[6]);
	c(0) = atoi(argv[7]);
	c(1) = atoi(argv[8]);
	c(2) = atoi(argv[9]);
	
	ros::init (argc, argv, "deformablePCL");
	ros::NodeHandle nh;
	ROS_INFO("Starting ");
                
	ros::Subscriber sub3 = nh.subscribe ("pushed", 1, cb_pushInfo);

	ros::Subscriber sub1 = nh.subscribe ("points_init", 1, cloud_cb_initial);	
	ros::Subscriber sub2 = nh.subscribe ("points_def", 1, cloud_cb_final);
		
	pubSim = nh.advertise<PointCloud> ("sim", 1);  
	pubBeta = nh.advertise<Float32> ("beta", 1);
	  
    // Spin
	ros::spin();
	
    return 0;
}
