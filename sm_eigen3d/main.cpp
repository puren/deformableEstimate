//Simulation
#include <iostream>
#include <fstream>
#include <string.h>

#include "include/deformable.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <vector>

Deformable *mDeformable;
/*int width = 656;
 int height = 710;
 int owidth = 256;
 int oheight = 248;
 double worldToScreen = owidth;*/

/*int owidth=1920;
int oheight = 810;
int odepth = 1400;
int width=1920;
int height = 810;
int depth = 1400;*/
int width = 656;
int height = 710;
int depth = 656;
int owidth = 256;
int oheight = 248;
int odepth = 256;
double worldToScreen = owidth;
double screenToWorld = 1.0f/worldToScreen;
int mouseVertexNr;
double rad;
//const float alphas[8] = { 0.00f, 0.01f, 0.02f, 0.05f, 0.10f, 0.20f, 0.50f, 1.00f };
//const float betas[5] = { 0.00f, 0.10f, 0.20f, 0.50f, 0.9f };

int indxAlpha=0;
int indxBeta=0;
bool isFixed=false;
bool isWrite=false;

int k=0;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
        Eigen::Vector3d pos;
        mouseVertexNr = -1;
        double minR2 = 0.0;
        int num = mDeformable->getNumVertices();
        
        for(int i=0; i<num; i++)
        {
            pos=mDeformable->getVertexPos(i)*worldToScreen;
            pos(1) = height - pos(1);
            float r2 = (x-pos(0))*(x-pos(0))+(y-pos(1))*(y-pos(1));
            if(i==0 || r2<minR2)
            {
                mouseVertexNr = i;
                minR2 = r2;
            }
        }
        std::cout<<"the vertex is fixed:"<<mouseVertexNr<<std::endl;
        
    }
    else if  ( event == cv::EVENT_RBUTTONDOWN )
    {
        //std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if  ( event == cv::EVENT_MBUTTONDOWN )
    {
        //std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if ( event == cv::EVENT_MOUSEMOVE )
    {
        //std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;
        if(mouseVertexNr<0) return;
        Eigen::Vector3d pos;
        //pos = mDeformable->getGoalVertexPos(mouseVertexNr);
        pos(0)=x*screenToWorld;
        pos(1)=(height-y)*screenToWorld;
        mDeformable->fixVertex(mouseVertexNr, pos);
        std::cout<<mouseVertexNr<<" "<<pos(0)<<" "<<pos(1)<<std::endl;
    }
    else if ( event == cv::EVENT_LBUTTONUP )
    {
        //std::cout << "Left button of the mouse is released - position - position (" << x << ", " << y << ")" << std::endl;
        if (mouseVertexNr >= 0)
        {
            if(isFixed && mouseVertexNr<8)
                return;
            mDeformable->releaseVertex(mouseVertexNr);
            std::cout<<"the vertex is released:"<<mouseVertexNr<<std::endl;
        }
        mouseVertexNr = -1;  
    }
}

int main(int argc, char **argv)
{
    if(argc<3)
    {
        std::cout<<"./def_sim [path_initconfig] [data0x] [res_toStop] [pushed_particle] [pos_x] [pos_y]"<<std::endl;
        return 0;
    }
    
    Eigen::Vector3d pos;
    
    mDeformable = new Deformable();
    //char fileN[] = "cube8.txt";
    

    std::string path(argv[1]);
    mDeformable->loadFromFile(path);
    std::cout<<"SIZE ";
    std::cout<<mDeformable->getNumVertices()<<std::endl;
    
    int m = mDeformable->params.m;
    int n = mDeformable->params.n;
    int p = mDeformable->params.p;

    double iRes;
    std::vector<int> partNos;
    std::vector<Eigen::Vector3d> pos_fixeds;
    
    if (argc>3) {
        
        iRes = atof(argv[3]);

        for (int i=4; i<argc; i++) {
            int partNo=atoi(argv[i]);
            partNos.push_back(partNo);
            Eigen::Vector3d pos_fixed;
            pos_fixed = mDeformable->getOriginalVertexPos(partNo);
            if (partNo>=0) {
                pos_fixed(0)=atof(argv[++i]);
                pos_fixed(1)=atof(argv[++i]);
                pos_fixed(2)=atof(argv[++i]);
            }
            std::cout<<"frame: "<<k<<std::endl;
            pos_fixeds.push_back(pos_fixed);
        }
    }
    
    int numVer;
    
    mDeformable->params.bounds.max(0) = width * screenToWorld;
    mDeformable->params.bounds.max(1) = height * screenToWorld;
    mDeformable->params.bounds.max(2) = depth * screenToWorld;
    
    cv::namedWindow( "View", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED );
    
    //cv::createButton(NULL,callbackButton);
    
    cv::Mat image(cv::Size(width,height),CV_8UC3);
    
    //double mMassToRadius=0.01;//mDeformable->params.massToRadius;
    //std::cout<<mMassToRadius<<std::endl;
    /*rad = sqrt(mDeformable->getMass(0) * mMassToRadius) * worldToScreen;**/
    
    std::ofstream myfile;
    char s [50];
    int i =0;
    int j;
    
    MatrixDef pos_prev;
    pos_prev=mDeformable->mOriginalPos;
    
    numVer=mDeformable->getNumVertices();
    while (mDeformable->params.beta<1.0)
    {
        std::cout<<"frame: "<<k<<std::endl;
            
        int k=0;
        mDeformable->initState();
        isFixed = false;
        int finish = 0;
        
        while(1)
        {
            double res=0.0;
            //std::cout<<"frame: "<<k<<std::endl;
            
            //<---------
            
            if(k==0 && partNos.size()>0)
            {
                std::cout<<"FIXED"<<std::endl;
                for (int k=0; k<p; k++) {
                    for (int i=0; i<m; i++) {
                        for (int j=0; j<n; j++) {
                            int idx = k*m*n + j*m + i;
                            Eigen::Vector3d pos;
                            pos = mDeformable->getOriginalVertexPos(idx);
                            
                            if((i==0 || i==m-1) || (j==0 || j==n-1) )
                            {
                                std::cout<<idx<<" "<<pos(0)<<" "<<pos(1)<<" "<<pos(2)<<std::endl;
                                mDeformable->fixVertex(idx, pos);
                            }
                            mDeformable->setVertexPos(idx, pos);
                        }
                    }
                }
                for (i=0; i<partNos.size(); i++) {
                    mDeformable->fixVertex(partNos.at(i), pos_fixeds.at(i));
                }
                
                /*for (int z=1; z<mDeformable->params.p; z++) {
                    for (int y=0; y<mDeformable->params.m; y++) {
                        for (int x=0; x<mDeformable->params.n; x++) {
                            int indx = z*mDeformable->params.m*mDeformable->params.n+
                                       y*mDeformable->params.n + x;
                            std::cout<<indx<<std::endl;
                            Eigen::Vector3d pos;
                            pos = mDeformable->getOriginalVertexPos(indx);
                            mDeformable->fixVertex(indx, pos);

                        }
                    }
                }*/
                
                
            }//<-------------
            if(partNos.size()<=0)
            {
                cv::setMouseCallback("View", CallBackFunc, NULL);
            }
            
            mDeformable->timeStep();
            image = cv::Mat::zeros(height+20, width,CV_8UC3);
            cv::line( image,
                     cv::Point(0,height),cv::Point(width-1,height),
                     cv::Scalar( 255,255 , 255 ),1);
            
            
            if (mDeformable->params.isCluster) {
                
                cv::RNG rng(12345);
                std::vector<cv::Scalar> colors;
                
                for ( i=0; i<mDeformable->mNumClusters; i++) {
                    cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
                    colors.push_back(color);
                }
                
                for( i = 0; i<mDeformable->mNumClusters; i++)
                {
                    std::vector<int> cluster = mDeformable->mClusters.at(i);
                    for (j=0; j<cluster.size(); j++) {
                        int v_indx = cluster.at(j);
                        pos = mDeformable->getVertexPos(v_indx);
                        double r = 1;//sqrt(mDeformable->getMass(i) * mMassToRadius) * worldToScreen;
                        double x = pos(0)*worldToScreen;
                        double z = pos(2)*worldToScreen;
                        double y = pos(1)*worldToScreen;
                        z = depth - z;
                        y = height - y;
                        //double z = pos(2)*worldToScreen;
                        //z = depth - z;
                        r=3;
                        circle(image, cv::Point(x-r, z-r), r, colors.at(i),1);
                        //std::cout<<x<<" "<<y<<std::endl;
                    }
                    /*if (k==0) {
                     imshow( "View", image );
                     cvWaitKey(1000);
                     
                     }
                     else if(k==1) cvWaitKey(10000);*/
                    /*if (mDeformable->isFixed(i))
                     {
                     std::cout<<i<<" "<<pos(0) <<","<< pos(1)<<std::endl;
                     continue;
                     }*/
                    
                    
                    //&y = height - y;
                    
                    /*if(x>=width) x=width-1;
                     else if(x<0) x = 0;
                     if(y>=height) y=height-1;
                     else if(y<0) y = 0;*/
                    
                    
                    
                    //image.at<cv::Vec3b>(y, x) = cv::Vec3b(255,255, 255);
                    
                }
            }
            else
            {
                
                for( i = 0; i<numVer; i++)
                {
                    
                    pos = mDeformable->getVertexPos(i);
                    
                    /*if (mDeformable->isFixed(i))
                     {
                     std::cout<<i<<" "<<pos(0) <<","<< pos(1)<<std::endl;
                     continue;
                     }*/
                    
                    double r = 5;
                    double x = pos(0)*worldToScreen;
                    double y = pos(1)*worldToScreen;
                    y = height - y;
                    //y = height - y;
                    circle(image, cv::Point(x-r, y-r), r, cv::Scalar(255, 255, 255), 1);
                    //std::cout<<i<<" "<<pos(0) <<","<< pos(1)<<std::endl;
                }
            }
            
            
            //std::cout<<"beta: "<<mDeformable->params.beta<<std::endl;
            
            for ( i = 0; i < numVer; i++) {
                pos = mDeformable->getVel(i);
                res += sqrt(pow(pos(0),2) + pow(pos(1),2));
                //sqrt(pow((pos_prev(i,0)-pos(0)),2) + pow((pos_prev(i,1)-pos(1)),2));
                //std::cout<<"before:"<<pos(0)<<" "<<pos(1)<<std::endl;
            }
            res=res/numVer;
            
            if(res<iRes)
            {
                finish++;
            }
            else
            {
                finish=0;
            }
            std::cout<<"res: "<<res<<std::endl;
            std::cout<<"beta: "<<mDeformable->params.beta<<std::endl;
            std::cout<<"finish count: "<<finish<<std::endl;
            
            pos_prev=mDeformable->mPos;
            if((finish>3 && k>10 && partNos.size()>0) || isWrite)
            {
                sprintf(s, "%s/sim%f.txt",argv[2], mDeformable->params.beta);
                FILE *f = fopen(s, "w");
                if (!f)
                {
                    std::cout<<"not open"<<std::endl;
                    return 0;
                }
                
                fprintf(f, "Pos\n");
                for ( i = 0; i < numVer; i++) {
                    pos = mDeformable->getVertexPos(i);
                    fprintf(f, "%lf %lf %lf\n", pos(0), pos(1), pos(2));
                }
                fprintf(f, "\nVel\n");
                for ( i = 0; i < numVer; i++) {
                    pos = mDeformable->getVel(i);
                    fprintf(f, "%lf %lf %lf\n", pos(0), pos(1), pos(2));
                    //std::cout<<"after:"<<pos(0)<<" "<<pos(1)<<std::endl;
                }
                std::cout<<std::endl;
                
                fclose(f);
                
                //cvWaitKey();
                
                //return 0;
                break;
            }
            
            cv::Size size(width/2,height/2);
            cv::Mat dst;//src image
            //resize(image,dst,size);//resize image
            imshow( "View", image);
            //sprintf(s, "./result/sim_%07d.jpg", k);
            //imwrite(s, dst );
            /*if(k>=100)
             cv::waitKey(1000);*/
            
            //update interface
            int key=cvWaitKey(1);
            //int indx_tmp=0;
            if(key==27) break;
            
            
            switch(key){
                case 'f':
                    
                    isWrite=true;
                    // mDeformable->params.beta += 0.1;
                    //return 0;
                    break;
                case 'w':
                    std::cout<<"write"<<std::endl;
                    isWrite = true;
                    break;
                case 's':
                    std::cout<<"write ends"<<std::endl;
                    isWrite = false;
                    break;
                case 'r':
                    std::cout<<"init"<<std::endl;
                    isFixed = false;
                    isWrite = false;
                    k=0;
                    mDeformable->initState();
                    break;
            }
            k++;
        }
        mDeformable->params.beta += 0.1;
    }
    
}
