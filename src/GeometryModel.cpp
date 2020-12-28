#include "include/GeometryModel.h"


#define THRESH 0.1f;




// after a perturbation in the warp nodes, we recompute the vertices and the normals
double GeometryModel::RMSE_with_update(Eigen::VectorXd& residual) {

    int tot_matches = 0;
    float error = 0.0f;
    
    DualQuaternion Transfo, point;
    Quaternion pointN = Quaternion(0.f, 0.f, 0.f, 0.f);
    float3 vtx, nmle;
    int2 pt;
    int indx_Depth;
    float dist, prod_scal;
    int nbalert = 0;

    // for visualization 
    vector<Point3f> vertex3d;
    vector<Point3f> tildeVertex3d;

    for (int idx = 0; idx < _Nb_Vertices; idx++) {
        indx_Depth = idx;
        // Warp each vertex with the current state of the warp field
        // Blend the transformations
        Transfo = DualQuaternion(Quaternion(0.0,0.0,0.0,0.0), Quaternion(0.0,0.0,0.0,0.0));
        for (int v = 0; v < 5; v++) {
            Transfo = Transfo +  _Warp[_Indices[(idx)+v]]*_Weights[(idx)+v];
        }
        Transfo = Transfo.Normalize();

        point = DualQuaternion(Quaternion(0.0,0.0,0.0,1.0), Quaternion(_Vertices[3*(idx)], _Vertices[3*(idx)+1], _Vertices[3*(idx)+2], 0.0f));
        point  = Transfo * point * Transfo.DualConjugate2();
        vtx = point.Dual().Vector();
        
        vertex3d.push_back(Point3f(vtx.x,vtx.y,vtx.z));
        tildeVertex3d.push_back(Point3f(_VMap[3*indx_Depth],_VMap[3*indx_Depth+1],_VMap[3*indx_Depth+2]));

        pointN = Quaternion(_Normals[3*(idx)], _Normals[3*(idx)+1], _Normals[3*(idx)+2], 0.0f);
        pointN  = Transfo.Real() * pointN * Transfo.Real().Conjugate();
        nmle = pointN.Vector();
        
        //Compute depth difference at reprojection error (reject outliers, by using normals also)
        indx_Depth = idx;
        prod_scal = nmle.x*(vtx.x - _VMap[3*indx_Depth]) + nmle.y*(vtx.y - _VMap[3*indx_Depth+1]) + nmle.z*(vtx.z - _VMap[3*indx_Depth+2]);
        dist = prod_scal; //*prod_scal;

        
        //Store correspondences into the matches array
        error += fabs(dist);
        residual(idx) = dist;//dist; STEPHANE
    }

    
    // convert the transformed vertices
    viz::Viz3d cloudPointsWindow("Dual quaternion good");

    cloudPointsWindow.showWidget("coordinate", viz::WCoordinateSystem(10)); // default 100
    cloudPointsWindow.showWidget("pointsSphere", viz::WCloud(vertex3d, viz::Color::green()));
    cloudPointsWindow.showWidget("pointsCylind", viz::WCloud(tildeVertex3d, viz::Color::red()));





    cloudPointsWindow.spin();


    return sqrt(error);///float(_Nb_Vertices);
}



