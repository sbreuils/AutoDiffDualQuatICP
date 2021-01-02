#include "include/GeometryModel.h"


#define THRESH 0.1f;




// after a perturbation in the warp nodes, we recompute the vertices and the normals
double GeometryModel::RMSE_with_update(Eigen::VectorXd& residual) {

    int tot_matches = 0;
    float error = 0.0f;

    DualQuaternionScalar<double> Transfo, point;
    QuaternionScalar<double> pointN = QuaternionScalar<double>(0., 0., 0., 0.);
    Scalar3<double> vtx, nmle;
    int indx_Depth;
    double dist, prod_scal;

    // for visualization
    vector<Point3f> vertex3d;
    vector<Point3f> tildeVertex3d;

    for (int idx = 0; idx < _Nb_Vertices; idx++) {
        indx_Depth = idx;
        // Warp each vertex with the current state of the warp field
        // Blend the transformations
        Transfo = DualQuaternionScalar<double>(QuaternionScalar<double>(0.0,0.0,0.0,0.0), QuaternionScalar<double>(0.0,0.0,0.0,0.0));
        for (int v = 0; v < 5; v++) {
            Transfo = Transfo +  _Warp[_Indices[(idx)+v]]*_Weights[(idx)+v];
        }
        Transfo = Transfo.Normalize();

        point = DualQuaternionScalar<double>(QuaternionScalar<double>(0.0,0.0,0.0,1.0), QuaternionScalar<double>(_Vertices[3*(idx)], _Vertices[3*(idx)+1], _Vertices[3*(idx)+2], 0.0f));
        point  = Transfo * point * Transfo.DualConjugate2();
        vtx = point.Dual().Vector();

        vertex3d.push_back(Point3f(vtx.x,vtx.y,vtx.z));
        tildeVertex3d.push_back(Point3f(_VMap[3*indx_Depth],_VMap[3*indx_Depth+1],_VMap[3*indx_Depth+2]));

        pointN = QuaternionScalar<double>(_Normals[3*(idx)], _Normals[3*(idx)+1], _Normals[3*(idx)+2], 0.0f);
        pointN  = Transfo.Real() * pointN * Transfo.Real().Conjugate();
        nmle = pointN.Vector();

        //Compute depth difference at reprojection error (reject outliers, by using normals also)
        indx_Depth = idx;
        prod_scal = nmle.x*(vtx.x - _VMap[3*indx_Depth]) + nmle.y*(vtx.y - _VMap[3*indx_Depth+1]) + nmle.z*(vtx.z - _VMap[3*indx_Depth+2]);
        dist = prod_scal;


        //Store correspondences into the matches array
        error += fabs(dist);
        residual(idx) = dist;//dist; STEPHANE
    }


    // convert the transformed vertices
    viz::Viz3d cloudPointsWindow("Dual quaternion in RMSE with Update");

    cloudPointsWindow.showWidget("coordinate", viz::WCoordinateSystem(10)); // default 100

    cloudPointsWindow.showWidget("pointsSphere", viz::WCloud(vertex3d, viz::Color::green()));
//    cloudPointsWindow.setRenderingProperty( "pointsSphere", cv::viz::POINT_SIZE, 5.4 );
    cloudPointsWindow.showWidget("pointsCylind", viz::WCloud(tildeVertex3d, viz::Color::red()));
    cloudPointsWindow.setRenderingProperty( "pointsCylind", cv::viz::POINT_SIZE, 5.4 );





    cloudPointsWindow.spin();


    return sqrt(error);///float(_Nb_Vertices);
}



