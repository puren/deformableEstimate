#include "deformable.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <unsupported/Eigen/NonLinearOptimization>

void DeformableParameters::setDefaults()
{
    timeStep = 0.01;
    gravity(0)= 0.0;
    gravity(1)= -9.81;
    
    
    bounds.min(0)=0.0;
    bounds.min(1)=0.0;
    bounds.max(0)=1.0;
    bounds.max(1)=1.0;
    
    alpha = 1.0;
    beta = 0.0;
    lbeta.clear();
    
    quadraticMatch = false;
    volumeConservation = false;
    
    allowFlip = false;
    isCluster = false;
}

Deformable::Deformable()
{
    reset();
}

Deformable::~Deformable()
{
}

void Deformable::reset()
{
    //std::cout<<"reset"<<std::endl;
    mNumVertices = 0;
    mNumClusters = 0;
    mOriginalPos.setZero();
    mPos.setZero();
    mNewPos.setZero();
    mGoalPos.setZero();
    mGoalPos_sum.setZero();
    mMasses.setZero();
    mVelocities.setZero();
    mFixed.setZero();
    v_pos.clear();
    v_mass.clear();
    indxCount.clear();
}

void Deformable::initState()
{
     //std::cout<<"init state"<<std::endl;
    //std::cout<<mNumVertices<<std::endl;
    mOriginalPos.resize(mNumVertices, Eigen::NoChange);
    mPos.resize(mNumVertices, Eigen::NoChange);
    mNewPos.resize(mNumVertices, Eigen::NoChange);
    mGoalPos.resize(mNumVertices, Eigen::NoChange);
    mGoalPos_sum.resize(mNumVertices, Eigen::NoChange);
    mVelocities.resize(mNumVertices, Eigen::NoChange);
    mFixed.resize(mNumVertices, Eigen::NoChange);
    mMasses.resize(mNumVertices, Eigen::NoChange);
    
    for (int i = 0; i < mNumVertices; i++) {
        Eigen::Vector2d row=v_pos.at(i);
        mOriginalPos.row(i) = row;
        mPos.row(i) = row;
        mNewPos.row(i) = row;
        mGoalPos.row(i) = row;
        mVelocities.row(i).setZero();
        mFixed(i,0) = false;
        mMasses(i,0) = v_mass.at(i);
    }
}
void Deformable::clusterVertex()
{
    int l_side=floor(sqrt(mNumVertices));
    int count_v=mNumVertices;
    for(int i=0; i<l_side-1; i++)
    {
        for(int j=0; j<l_side-1; j++)
        {
            std::vector<int> row;
            //get the vertices in the cluster
            for(int k=0; k<2; k++)
            {
                for(int l=0; l<2; l++)
                {
                    int p=(i+k)*l_side + (j+l);
                    if (p<mNumVertices) {
                        row.push_back(p);
                        std::cout<<p<<std::endl;
                        count_v--;
                    }
                }
            }
            std::cout<<std::endl;
            mClusters.push_back(row);//put the cluster
            params.lbeta.push_back(params.beta);
            mNumClusters++;
        }
    }
    if(count_v>0)
    {
        std::vector<int> row;
        for(int i=count_v; i>=0; i--)
        {
            int p=(mNumVertices-1)-i;
            row.push_back(p);
            std::cout<<p<<std::endl;
        }
        mClusters.push_back(row);
        mNumClusters++;
    }
    
}

void Deformable::addVertex(const Eigen::Vector2d pos, double mass)
{
    v_pos.push_back(pos);
    v_mass.push_back(mass);
    indxCount.push_back(0);
    mNumVertices++;
}

void Deformable::externalForces()
{
    
    int i;
    
    for (i = 0; i < mNumVertices; i++) {
        if (mFixed(i,0))
        {
            //std::cout<<"fixed"<<std::endl;
            continue;
        }
        mVelocities.row(i) += params.gravity * params.timeStep;
        mNewPos.row(i) = mPos.row(i) + mVelocities.row(i) * params.timeStep;
        mGoalPos.row(i) = mOriginalPos.row(i);
    }
    
    // boundaries
    double restitution = 0.9;
    for (i = 0; i < mNumVertices; i++) {
        
        if (mNewPos(i,0) < params.bounds.min(0) || mNewPos(i,0) > params.bounds.max(0)) {
            mNewPos(i,0) = mPos(i,0) - mVelocities(i,0) * params.timeStep * restitution;
            mNewPos(i,1) = mPos(i,1);
        }
        if (mNewPos(i,1) < params.bounds.min(1) || mNewPos(i,1) > params.bounds.max(1)) {
            mNewPos(i,1) = mPos(i,1) - mVelocities(i,1) * params.timeStep * restitution;
            mNewPos(i,0) = mPos(i,0);
        }
        // params.bounds.clamp(mNewPos[i]);
        //clamp
        float offset = 0.0;
        if (mNewPos(i,0) < params.bounds.min(0) + offset)
        {
            mNewPos(i,0) = params.bounds.min(0) + offset;
        }
        if (mNewPos(i,0) > params.bounds.max(0) - offset)
        {
            mNewPos(i,0) = params.bounds.max(0) - offset;
        }
        if (mNewPos(i,1) < params.bounds.min(1) + offset)
        {
            mNewPos(i,1) = params.bounds.min(1) + offset;
        }
        if (mNewPos(i,1) > params.bounds.max(1) - offset)
        {
            mNewPos(i,1) = params.bounds.max(1) - offset;
        }
        

    }
    
}



void Deformable::projectPositions()
{
    if (mNumVertices <= 1) return;
    int i;//,j,k;
    
    // center of mass
    Eigen::Vector2d cm, originalCm;
    cm.setZero(); originalCm.setZero();
    double mass = 0.0;
    
    for (i = 0; i < mNumVertices; i++) {
        double m = mMasses(i,0);
        if (mFixed(i,0)) m *= 1000.0;
        mass += m;
        cm += mNewPos.row(i) * m;
        originalCm += mOriginalPos.row(i) * m;
        
    }
    //std::cout<<std::endl;
    cm /= mass;
    
    originalCm /= mass;
    
    Eigen::Matrix2d Apq;
    Eigen::Matrix2d Aqq;
    Eigen::Vector2d p;
    Eigen::Vector2d q;
    Apq.setZero();
    Aqq.setZero();
    
    for (i = 0; i < mNumVertices; i++) {
        p(0) = mNewPos(i,0) - cm(0);
        p(1) = mNewPos(i,1) - cm(1);
        
        q(0) = mOriginalPos(i,0) - originalCm(0);
        q(1) = mOriginalPos(i,1) - originalCm(1);
        double m = mMasses(i,0);
        Apq += m*p*q.transpose();
        Aqq += m*q*q.transpose();
    }
  
    /*if (!params.allowFlip && Apq.determinant() < 0.0f) {  	// prevent from flipping
        Apq(0,1) = -Apq(0,1);
        Apq(1,1) = -Apq(1,1);
    }*/
    
    //std::cout<<Apq<<std::endl;
    
    Eigen::Matrix2d R;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Apq, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();
    R = U*V.transpose();
    if (!params.quadraticMatch) {	// --------- linear match
        
        
        Eigen::Matrix2d A = Aqq;
        

        Eigen::Matrix2d A_=A.inverse();
        /*std::cout<<A_.row(0)<<std::endl;
        std::cout<<A_.row(1)<<std::endl;*/
        A = Apq*A_;
        
        if (params.volumeConservation) {
            double det = A.determinant();
            if(det<0) det=-det;
            if (det != 0.0) {
                det = 1.0 / sqrt(fabs(det));
                if (det > 2.0) det = 2.0;
                A *= det;
            }
        }
        float one_beta =1.0 - params.beta;
        Eigen::Matrix2d T = R * one_beta;
        
        
        A=A*params.beta;
        T = T + A;
        
        
        for (i = 0; i < mNumVertices; i++) {
            if (mFixed(i)) continue;
            q(0) = mOriginalPos(i,0) - originalCm(0);
            q(1) = mOriginalPos(i,1) - originalCm(1);
            Eigen::Vector2d Tq = T*q;
            mGoalPos(i,0) = Tq(0)+cm(0);
            mGoalPos(i,1) = Tq(1)+cm(1);
            //mGoalPos.row(i)=(R * (1.0f - params.beta) + A * params.beta)*q+cm;
            mNewPos.row(i) += (mGoalPos.row(i) - mNewPos.row(i)) * params.alpha;
            
            //std::cout<<T.row(0)<<std::endl;
            //std::cout<<T.row(1)<<std::endl;
        }
    }
}

void Deformable::projectPositionsCluster(std::vector<int> cluster, int cluster_indx)
{
    int numVertices =  cluster.size();
    if (numVertices <= 1) return;
    int i;//,j,k;
    double beta_cluster =params.lbeta.at(cluster_indx);
    
    // center of mass
    Eigen::Vector2d cm, originalCm;
    cm.setZero(); originalCm.setZero();
    double mass = 0.0;
    int indx;
    for (i = 0; i < numVertices; i++) {
        indx= cluster.at(i);
        double m = mMasses(indx,0);
        if (mFixed(indx,0)) m *= 100.0;
        mass += m;
        cm += mNewPos.row(indx) * m;
        originalCm += mOriginalPos.row(indx) * m;
        //std::cout<<"before: "<<mNewPos.row(indx)<<std::endl;
    }
    
    cm /= mass;
    originalCm /= mass;
    
    Eigen::Matrix2d Apq;
    Eigen::Matrix2d Aqq;
    Eigen::Vector2d p;
    Eigen::Vector2d q;
    Apq.setZero();
    Aqq.setZero();
    
    for (i = 0; i < numVertices; i++) {
        indx= cluster.at(i);
        p(0) = mNewPos(indx,0) - cm(0);
        p(1) = mNewPos(indx,1) - cm(1);
        
        q(0) = mOriginalPos(indx,0) - originalCm(0);
        q(1) = mOriginalPos(indx,1) - originalCm(1);
        double m = mMasses(indx,0);
        Apq += m*p*q.transpose();
        Aqq += m*q*q.transpose();
    }
    

    
    if (!params.allowFlip && Apq.determinant() < 0.0f)
    {  	// prevent from flipping
        Apq(0,1) = -Apq(0,1);
        Apq(1,1) = -Apq(1,1);
    }
    
    Eigen::Matrix2d R;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Apq, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();
    R = U*V.transpose();
    
    if (!params.quadraticMatch) {	// --------- linear match
        Eigen::Matrix2d A = Aqq;
        A = Apq*A.inverse();
        
        if (params.volumeConservation) {
            
            double det = A.determinant();
            if (det != 0.0) {
                det = 1.0 / sqrt(fabs(det));
                if (det > 2.0) det = 2.0;
                
                A = A*det;
            }
        }
        
        Eigen::Matrix2d T = R * (1.0 - beta_cluster) + A * beta_cluster;
        std::cout<<"cluster's beta "<<beta_cluster<<std::endl;
        
        for (i = 0; i < numVertices; i++) {
            int indx= cluster.at(i);
            indxCount.at(indx) +=1;
            
            if (mFixed(indx)) continue;
            q(0) = mOriginalPos(indx,0) - originalCm(0);
            q(1) = mOriginalPos(indx,1) - originalCm(1);
            Eigen::Vector2d Tq = T*q;
            
            mGoalPos(indx,0) = Tq(0)+cm(0);
            mGoalPos(indx,1) = Tq(1)+cm(1);
            mNewPos.row(indx) += (mGoalPos.row(indx) - mNewPos.row(indx)) * params.alpha;
            
            mGoalPos_sum.row(indx) += mNewPos.row(indx);
        }
    }
}


void Deformable::integrate()
{
    float dt1 = 1.0f / params.timeStep;
    for (int i = 0; i < mNumVertices; i++) {
        mVelocities.row(i) = (mNewPos.row(i) - mPos.row(i)) * dt1;
        //mVelocities.row(i) = (mGoalPos.row(i) - mPos.row(i)) * dt1;
        mPos.row(i) = mNewPos.row(i);
    }
}

void Deformable::timeStep()
{
    externalForces();
    //projectPositions();
    if( params.isCluster)
    {
        for(int i=0; i<mNumClusters; i++)
        {
            std::vector<int> cluster= mClusters.at(i);
            projectPositionsCluster(cluster, i);
        }
        
        for (int i=0; i<mNumVertices; i++)
        {
            if (mFixed(i)) continue;
            //std::cout<<mGoalPos_sum.row(i)<<" "<<indxCount.at(i)<<std::endl;
            mNewPos.row(i) = mGoalPos_sum.row(i)/indxCount.at(i);
            //std::cout<<"after: "<<mNewPos.row(i)<<" "<<indxCount.at(i)<<std::endl;
            //std::cout<<mNewPos.row(i)<<std::endl;
        }
        
        mGoalPos_sum.setZero();
        std::fill(indxCount.begin(), indxCount.end(), 0);
    }
    else
    {
        projectPositions();
    }
    integrate();
}

void Deformable::setVertexPos(int nr, const Eigen::Vector2d pos)
{
    mPos(nr,0) = pos(0);
    mPos(nr,1) = pos(1);
}

void Deformable::setVel(int nr, const Eigen::Vector2d vel)
{
    mVelocities(nr,0) = vel(0);
    mVelocities(nr,1) = vel(1);
}

void Deformable::fixVertex(int nr, const Eigen::Vector2d pos)
{
    std::cout<<"in fixVertex: "<<nr<<" "<<pos(0)<<" "<<pos(1)<<std::endl;
    mNewPos(nr,0) = pos(0);
    mNewPos(nr,1) = pos(1);
    
    mFixed(nr) = true;
}

void Deformable::releaseVertex(int nr)
{
    mFixed(nr)=false;
}

void Deformable::loadFromArray(std::vector<Eigen::Vector2d> l_vertices)
{
    int numVerts;
    numVerts = int(l_vertices.size());
    int i;
    double mass=0.1;
    Eigen::Vector2d pos;
    for (i = 0; i < numVerts; i++) {
        pos = l_vertices.at(i);
        addVertex(pos, mass);
    }
    initState();
}

void Deformable::saveToFile(char *filename)
{
    FILE *f = fopen(filename, "w");
    if (!f) return;
    
    fprintf(f, "%i\n", mNumVertices);
    for (int i = 0; i < mNumVertices; i++) {
        fprintf(f, "%lf %lf %lf\n", mPos(i,0), mPos(i,1), mMasses(i));
    }
    fprintf(f, "%lf\n", params.timeStep);
    fprintf(f, "%lf %lf\n", params.gravity(0), params.gravity(1));
    
    fprintf(f, "%lf\n", params.alpha);
    fprintf(f, "%lf\n", params.beta);
    std::cout<<params.beta<<std::endl;
    
    fprintf(f, "%i\n", params.quadraticMatch);
    fprintf(f, "%i\n", params.volumeConservation);
    fprintf(f, "%i\n", params.allowFlip);
    fprintf(f, "%i\n", params.isCluster);
    std::cout<<params.quadraticMatch<<std::endl;
    std::cout<<params.volumeConservation<<std::endl;
    std::cout<<params.allowFlip<<std::endl;
    
    fclose(f);
}

void Deformable::loadFromFile(char *filename)
{
    FILE *f = fopen(filename, "r");
    if (!f)
    {
        std::cout<<"no file"<<" "<<filename<<std::endl;
        return;
    }
    const int len = 100;
    char s[100];
    Eigen::Vector2d pos;
    double mass;
    int i;
    double x, y;
    reset();
    
    int original;
    int oheight;
    int owidth;
    int worldToScreen;
    fgets(s, len, f); sscanf(s, "%i", &original);
    //std::cout<<"origial "<<original<<std::endl;
    if(original==1)
    {
        fgets(s, len, f); sscanf(s, "%i", &oheight);
        fgets(s, len, f); sscanf(s, "%i", &owidth);
        //std::cout<<oheight<<" "<<owidth<<std::endl;
        worldToScreen = owidth;
    }
    
    int numVerts;
    fgets(s, len, f); sscanf(s, "%i", &numVerts);
    
    for (i = 0; i < numVerts; i++) {
        
        fgets(s, len, f); sscanf(s, "%lf %lf %lf", &x, &y, &mass);
        if(original==1)
        {
            x=x/worldToScreen;
            y=(oheight-y)/worldToScreen;
            //std::cout<<x<<" "<<y<<std::endl;
        }
        pos(0)=x;
        pos(1)=y;
        //std::cout<<pos(0)<<" "<<pos(1)<<std::endl;
        addVertex(pos, mass);
    }
    fgets(s, len, f); sscanf(s, "%lf", &params.timeStep);
    fgets(s, len, f); sscanf(s, "%lf %lf", &params.gravity(0), &params.gravity(1));
    
    fgets(s, len, f); sscanf(s, "%lf", &params.alpha);
    fgets(s, len, f); sscanf(s, "%lf", &params.beta);
    
    fgets(s, len, f); sscanf(s, "%i", &i); params.quadraticMatch = i;
    fgets(s, len, f); sscanf(s, "%i", &i); params.volumeConservation = i;
    fgets(s, len, f); sscanf(s, "%i", &i); params.allowFlip = i;
    fgets(s, len, f); sscanf(s, "%i", &i); params.isCluster = i;
    
    fclose(f);
    
    initState();
    
    if(params.isCluster)
        clusterVertex();
}