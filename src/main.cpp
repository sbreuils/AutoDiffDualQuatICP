// #include "include/Utilities.h"
#include "include/DualQuaternion.h"
#include "include/CeresFunctors.h"
#include "include/GeometryModel.h"

//using namespace glm;
using namespace std;
using namespace cv;
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CostFunction;
using ceres::DynamicAutoDiffCostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

GeometryModel scene;







int main( int argc, char **argv )
{


    // start with Ceres an example
    google::InitGoogleLogging(argv[0]);


    std::cout << "rigid body parameters" << std::endl;
    std::vector<DualQuaternionScalar<double> > nodeParameters =  scene.Warp();

    std::vector<double> x;

    // extract the parameters
    for(unsigned int i=0 ; i< nodeParameters.size() ; ++i){
        DualQuaternionScalar<double> currParameters = nodeParameters[i];

        Scalar3<double> thetaEul = logOfQuaternion(currParameters.Real());
        Scalar3<double> trans = currParameters.GetTranslation();

        x.push_back(thetaEul.x);
        x.push_back(thetaEul.y);
        x.push_back(thetaEul.z);

        x.push_back(trans.x);
        x.push_back(trans.y);
        x.push_back(trans.z);
    }

    
    // just a copy of the parameters
    std::vector<double> initial_x; 
    std::copy(x.begin(),x.end(),std::back_inserter(initial_x));


    Problem problem;

 
    // with Autodiff and ONLY quaternions
    // for (int i = 0; i < 144; ++i) {
    //    CostFunction* cost_function = new ceres::AutoDiffCostFunction<AutoDiffPbSolving, 3, 6>(new AutoDiffPbSolving(OuterShell.NBVertices(), OuterShell.Vertices(), OuterShell.Normales(), OuterShell.Matches(),i));
    //    problem.AddResidualBlock(cost_function, NULL, &x[6*i]);
    // }

    // with Autodiff and  dualquaternions
    // for (int i = 0; i < 144; ++i) {
    //    CostFunction* cost_function = new ceres::AutoDiffCostFunction<AutoDiffPbSolvingDualQuaternions, 3, 6>(new AutoDiffPbSolvingDualQuaternions(OuterShell.NBVertices(), OuterShell.Vertices(), OuterShell.Normales(), OuterShell.Matches(),i));
    //    problem.AddResidualBlock(cost_function, NULL, &x[6*i]);
    // }


    // with Autodiff, dualquaternions : init the cost function
    for (int idxVer = 0; idxVer < scene.NBVertices(); ++idxVer) {
        // get the set of weights and indices associate
       const int idxNode1 = scene.Indices()[5*(idxVer)+0];
       const int idxNode2 = scene.Indices()[5*(idxVer)+1];
       const int idxNode3 = scene.Indices()[5*(idxVer)+2];
       const int idxNode4 = scene.Indices()[5*(idxVer)+3];
       const int idxNode5 = scene.Indices()[5*(idxVer)+4];

       CostFunction* cost_function = new ceres::AutoDiffCostFunction<AutoDiffPbSolvingDualQuaternionsWithWeights, 3, 6, 6, 6, 6, 6>(new AutoDiffPbSolvingDualQuaternionsWithWeights(scene.NBVertices(), scene.Vertices(), scene.Normales(), scene.Matches(), scene.Weights(), scene.Indices(),idxVer));
       problem.AddResidualBlock(cost_function, NULL, &x[6*idxNode1], &x[6*idxNode2], &x[6*idxNode3], &x[6*idxNode4], &x[6*idxNode5]);
    }



    // Run the solver!
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 70;
    options.use_explicit_schur_complement=true;

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";


    Eigen::VectorXd residuWithourPerturbation(scene.NBVertices());
    std::cout << "before computing RMSE" << std::endl;
    double totalErrorBefore = scene.RMSE_with_update(residuWithourPerturbation);

    cout << "total error without perturbation is = "<<totalErrorBefore<<endl;


    // display the resulting
    std::vector<DualQuaternionScalar<double>> resulting_Warps;
    for(unsigned int i=0;i<scene.NBNodes();++i){
        DualQuaternionScalar<double> currParameters = DualQuaternionScalar<double>(make_Scalar3(x[6*i],x[6*i+1],x[6*i+2]),make_Scalar3(x[6*i+3],x[6*i+4],x[6*i+5]));

        resulting_Warps.push_back(currParameters);
    }

    vector<Point3f> resultVertex3d;
    vector<Point3f> tildeVertex3d;
    
    Scalar3<double> vtx;
    Scalar3<double> nmle;

    vector<double> errorResult;
    double totalError = 0.0;

    QuaternionScalar<double> pointN = QuaternionScalar<double>(0., 0., 0., 0.);

    // version with blending
    for(unsigned int idx=0 ; idx< scene.NBVertices(); ++idx){
        int indx_Depth = idx;

        DualQuaternionScalar<double> point = DualQuaternionScalar<double>(QuaternionScalar<double>(0.0,0.0,0.0,1.0), QuaternionScalar<double>(scene.Vertices()[3*(idx)], scene.Vertices()[3*(idx)+1], scene.Vertices()[3*(idx)+2], 0.0f));

        DualQuaternionScalar<double> Transfo = DualQuaternionScalar<double>(QuaternionScalar<double>(0.0,0.0,0.0,0.0), QuaternionScalar<double>(0.0,0.0,0.0,0.0));
        for(int v= 0; v<5; ++v){
            Transfo = Transfo +  resulting_Warps[scene.Indices()[5*(idx)+v]]*scene.Weights()[5*(idx)+v];
        }
        Transfo = Transfo.Normalize();

        point  = Transfo * point * Transfo.DualConjugate2();
        vtx = point.Dual().Vector();

        pointN = QuaternionScalar<double>(scene.Normales()[3*(idx)], scene.Normales()[3*(idx)+1], scene.Normales()[3*(idx)+2], 0.0f);
        pointN  = Transfo.Real() * pointN * Transfo.Real().Conjugate();
        nmle = pointN.Vector();


        // compute the normal as well
        double scalarProd = nmle.x*(vtx.x - scene.Matches()[3*indx_Depth]) + nmle.y*(vtx.y - scene.Matches()[3*indx_Depth+1]) + nmle.z*(vtx.z - scene.Matches()[3*indx_Depth+2]);
        totalError += std::abs(scalarProd);
        errorResult.push_back(totalError);

        resultVertex3d.push_back(Point3f(vtx.x,vtx.y,vtx.z));
        tildeVertex3d.push_back(Point3f(scene.Matches()[3*indx_Depth],scene.Matches()[3*indx_Depth+1],scene.Matches()[3*indx_Depth+2]));
    }




    std::cout << "total resulting error is "<< totalError << std::endl;

    // display the resulting
    viz::Viz3d cloudPointsWindowResult("Dual quaternion better");

    cloudPointsWindowResult.showWidget("coordinate", viz::WCoordinateSystem(10)); // default 100


    cloudPointsWindowResult.showWidget("pointsSphere", viz::WCloud(resultVertex3d, viz::Color::green()));
    cloudPointsWindowResult.setRenderingProperty( "pointsSphere", cv::viz::POINT_SIZE, 5.4 );
    cloudPointsWindowResult.showWidget("pointsCylind", viz::WCloud(tildeVertex3d, viz::Color::red()));
    cloudPointsWindowResult.setRenderingProperty( "pointsCylind", cv::viz::POINT_SIZE, 5.0 );

    cloudPointsWindowResult.spin();





	return 0 ;
}

