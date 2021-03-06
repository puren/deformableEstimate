#ifndef deformableH
#define deformableH

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/SVD>


typedef std::vector<Eigen::Vector3d > MatrixDef;


struct Bound{
    Eigen::Vector3d min;
    Eigen::Vector3d max;
};

struct DeformableParameters
{
    DeformableParameters() { setDefaults(); }
    double timeStep;
    Eigen::Vector3d gravity;
    Bound bounds;
    
    double alpha;
    double beta;
    std::vector<double> lbeta;
    bool quadraticMatch;

    bool volumeConservation;
    
    bool allowFlip;
    
    void setDefaults();
    
    bool isCluster;
    
    double massToRadius;
    double offset;
    
    int m,n,p;
    
    Eigen::Vector3i cluster_size;
    
};

class Deformable
{
public:
    Deformable();
    ~Deformable();
    
    void reset();
    void addVertex(const Eigen::Vector3d pos, double mass);
    
    void externalForces();
    void projectPositions();
    void integrate();
    
    void timeStep();
    
    DeformableParameters params;
    
    int  getNumVertices() const { return mNumVertices; }
    const Eigen::Vector3d getVertexPos(int nr) { return mPos.at(nr); }
    const Eigen::Vector3d getNewPos(int nr) { return mNewPos.at(nr); }
    void setVertexPos(int nr, const Eigen::Vector3d pos);
    const Eigen::Vector3d getOriginalVertexPos(int nr) { return mOriginalPos.at(nr); }
    const Eigen::Vector3d getGoalVertexPos(int nr) { return mGoalPos.at(nr);}
    const float getMass(int nr) { return mMasses.at(nr); }
    
    const Eigen::Vector3d getVel(int nr) { return mVelocities.at(nr);}
    void setVel(int nr, const Eigen::Vector3d vel);
    
    void fixVertex(int nr, const Eigen::Vector3d pos);
    bool isFixed(int nr) { return mFixed.at(nr); }
    void releaseVertex(int nr);
    
    //void saveToFile(char *filename);
    void loadFromFile(std::string filename);
    
    void clusterVertex();
    void projectPositionsCluster(std::vector<int> cluster, int indx_cluster);
    
    void loadFromArray(MatrixDef l_vertices);
    
    int isThere(const Eigen::Vector3d pos);
    
//private:
    void initState();
    
    int mNumVertices;
    int mNumClusters;
    MatrixDef mOriginalPos;
    MatrixDef mPos;
    MatrixDef mNewPos;
    MatrixDef mGoalPos;
    MatrixDef mGoalPos_sum;
    MatrixDef mVelocities;
    std::vector<double> mFixed;
    std::vector<double> mMasses;
    
    MatrixDef v_pos;
    std::vector<double> v_mass;
    std::vector< std::vector<int> > mClusters;
    std::vector<double> indxCount;
    std::vector<std::vector<int> > vert_cluster;
};

#endif
