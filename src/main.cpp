// #include "include/Utilities.h"
#include "include/ceresUtilities.h"
#include "include/DualQuaternion.h"

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

GeometryModel OuterShell;







int main( int argc, char **argv )
{


    // start with Ceres an example
    google::InitGoogleLogging(argv[0]);


    std::cout << "rigid body parameters" << std::endl;
    std::vector<DualQuaternion> nodeParameters =  OuterShell.Warp();

    std::vector<double> x;

    // std::cout << "main : 5th node = "<<std::endl;
    // nodeParameters[5].Print();

    for(unsigned int i=0 ; i< nodeParameters.size() ; ++i){
        DualQuaternion currParameters = nodeParameters[i];

        float3 thetaEul = logOfQuaternion(currParameters.Real());
        float3 trans = currParameters.GetTranslation();

        x.push_back(thetaEul.x);
        x.push_back(thetaEul.y);
        x.push_back(thetaEul.z);

        x.push_back(trans.x);
        x.push_back(trans.y);
        x.push_back(trans.z);

    }

    
    
    std::vector<double> initial_x; 
    std::copy(x.begin(),x.end(),std::back_inserter(initial_x)); //&x[0];


    Problem problem;

    // OK , it works
    // CostFunction* cost_function =
    //       new NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL, 1, 1>(
    //       new NumericDiffCostFunctor);
    // problem.AddResidualBlock(cost_function, nullptr, &x);

    // Ok it works
    // with Numerical differentiation, 21 parameters and input
    // CostFunction* cost_function =
    //       new NumericDiffCostFunction<NumericDiffCostSeveralVariablesFunctor, ceres::CENTRAL, 21, 21>(
    //       new NumericDiffCostSeveralVariablesFunctor);
    // problem.AddResidualBlock(cost_function, nullptr, &x[0]);


    // // with Auto differentiation, 21 parameters and input NOT OK
    // CostFunction* cost_function =
    //       new AutoDiffCostFunction<NumericDiffCostSeveralVariablesFunctor, 8, 8>(
    //       new NumericDiffCostSeveralVariablesFunctor);
    // problem.AddResidualBlock(cost_function, nullptr, &x[0]);


    // with Numeric differentiation, 8 parameters and input
    // CostFunction* cost_function =
    //       new NumericDiffCostFunction<NumericDiffCostSeveralVariablesFunctorDualQuat,ceres::CENTRAL, 8, 8>(
    //       new NumericDiffCostSeveralVariablesFunctorDualQuat);
    // problem.AddResidualBlock(cost_function, nullptr, &x[0]);

    // Scalar4<double> testScal;
    // Scalar4<double> testScal2;
    // Scalar4<double> result;
    // testScal.x = 3.2;
    // testScal.y = 2.2;
    // testScal.z = 5.2;
    // testScal.w = -4.2;

    // testScal2.x = 8.1;
    // testScal2.y = 2.3;
    // testScal2.z = 4.9;
    // testScal2.w = 14.9;

    // result = testScal2 + testScal;

    // std::cout << "result double4 ="<<result.x << ", "<<result.y << ", "<<result.z <<", "<<result.w << ". " << std::endl;


    // QuaternionScalar<double> rotation = QuaternionScalar<double>(0.0,0.0,0.0,1.0);
    // rotation.value.w = cos(3.14159/2.0);
    // rotation.value.z = sin(3.14159/2.0);

    // rotation.Print();
    

    // DualQuaternionScalar<double> Transfor = DualQuaternionScalar<double>(rotation,QuaternionScalar<double>(2.0,2.0,0.0,0.0));
    // DualQuaternionScalar<double> pointdd = DualQuaternionScalar<double>(QuaternionScalar<double>(0.0,0.0,0.0,1.0),QuaternionScalar<double>(1.0,0.0,0.0,0.0));

    // DualQuaternionScalar<double> resultDQ = Transfor*pointdd*(Transfor.DualConjugate2());

    // std::cout << "result dual quaternion ="<<std::endl;
    // resultDQ.Print();

    // Eigen::Matrix<double, 3, 3>  outerProd = outer_prod_tri_order(testScal, testScal2);

    // std::cout << "outer product ="<<outerProd<<std::endl;


    // with Numeric differentiation, more dual quaternions
    // constexpr int nbParam = 6*144;
    // CostFunction* cost_function =
    //       new NumericDiffCostFunction<NumericDiffPointToPlanePb, ceres::CENTRAL, 144 , nbParam>(
    //       new NumericDiffPointToPlanePb(OuterShell.NBVertices(), OuterShell.Vertices(), OuterShell.Normales(), OuterShell.Matches()));
    // problem.AddResidualBlock(cost_function, nullptr, &x[0]);

    // with Numeric differentiation, more dual quaternions
    // for (int i = 0; i < 144; ++i) {
    //     // first viewpoint : dstcloud, fixed
    //     // second viewpoint: srcCloud, moves
    //     // nor is normal of dst
    //    CostFunction* cost_function = new NumericDiffCostFunction<NumericDiffPointToPlanePbSingle, ceres::CENTRAL, 1 , 6>(
    //        new NumericDiffPointToPlanePbSingle(OuterShell.NBVertices(), OuterShell.Vertices(), OuterShell.Normales(), OuterShell.Matches(),i));
       
    //    //ICPCostFunctions::PointToPlaneError_CeresAngleAxis::Create(dst[i],src[i],nor[i]);
    //     problem.AddResidualBlock(cost_function, NULL, &x[6*i]);
    // }
    
    // with Numeric differentiation, more dual quaternions
    // for (int i = 0; i < 144; ++i) {
    //     // first viewpoint : dstcloud, fixed
    //     // second viewpoint: srcCloud, moves
    //     // nor is normal of dst
    //    CostFunction* cost_function = new NumericDiffCostFunction<NumericDiffPointToPlanePbSingleQuaternions, ceres::CENTRAL, 3 , 6>(
    //        new NumericDiffPointToPlanePbSingleQuaternions(OuterShell.NBVertices(), OuterShell.Vertices(), OuterShell.Normales(), OuterShell.Matches(),i));
       
    //    //ICPCostFunctions::PointToPlaneError_CeresAngleAxis::Create(dst[i],src[i],nor[i]);
    //     problem.AddResidualBlock(cost_function, NULL, &x[6*i]);
    // }
    




    // constexpr int nbParam = 6*144;

    // CostFunction* cost_function =
    //       new NumericDiffCostFunction<NumericDiffPointToPlanePb, ceres::CENTRAL, 144 , nbParam>(
    //       new NumericDiffPointToPlanePb(OuterShell.NBVertices(), OuterShell.Vertices(), OuterShell.Normales(), OuterShell.Matches()));
    // problem.AddResidualBlock(cost_function, nullptr, &x[0]);



    // with Automatic differentiation, more dual quaternions
    // for (int i = 0; i < 144; ++i) {
    //    CostFunction* cost_function = new ceres::AutoDiffCostFunction<AutoDiffPointToPlanePbSingleQuaternions<double>, 3, 6>(new AutoDiffPointToPlanePbSingleQuaternions<double>(OuterShell.NBVertices(), OuterShell.Vertices(), OuterShell.Normales(), OuterShell.Matches(),i));
    //    problem.AddResidualBlock(cost_function, NULL, &x[6*i]);
    // }



    // return (new ceres::AutoDiffCostFunction<PointToPointError_CeresAngleAxis, 3, 6>(new PointToPointError_CeresAngleAxis(observed, worldPoint)));

    // with Autodiff and ONLY quaternions OOOKKK
    // for (int i = 0; i < 144; ++i) {
    //    CostFunction* cost_function = new ceres::AutoDiffCostFunction<AutoDiffPbSolving, 3, 6>(new AutoDiffPbSolving(OuterShell.NBVertices(), OuterShell.Vertices(), OuterShell.Normales(), OuterShell.Matches(),i));
    //    problem.AddResidualBlock(cost_function, NULL, &x[6*i]);
    // }

    // with Autodiff and  dualquaternions OOOKKK
    // for (int i = 0; i < 144; ++i) {
    //    CostFunction* cost_function = new ceres::AutoDiffCostFunction<AutoDiffPbSolvingDualQuaternions, 3, 6>(new AutoDiffPbSolvingDualQuaternions(OuterShell.NBVertices(), OuterShell.Vertices(), OuterShell.Normales(), OuterShell.Matches(),i));
    //    problem.AddResidualBlock(cost_function, NULL, &x[6*i]);
    // }


    // with Autodiff and dualquaternions OOOKKK
    for (int idxVer = 0; idxVer < OuterShell.NBVertices(); ++idxVer) {
        // get the set of weights and indices associate
       const int idxNode1 = OuterShell.Indices()[5*(idxVer)+0];
       const int idxNode2 = OuterShell.Indices()[5*(idxVer)+1];
       const int idxNode3 = OuterShell.Indices()[5*(idxVer)+2];
       const int idxNode4 = OuterShell.Indices()[5*(idxVer)+3];
       const int idxNode5 = OuterShell.Indices()[5*(idxVer)+4];
       std::cout << "vertex num "<< idxVer << std::endl;
       std::cout << "idxNode 1 =" << idxNode1 << std::endl;
       std::cout << "idxNode 2 =" << idxNode2 << std::endl;
       std::cout << "idxNode 3 =" << idxNode3 << std::endl;
       std::cout << "idxNode 4 =" << idxNode4 << std::endl;
       std::cout << "idxNode 5 =" << idxNode5 << std::endl;

       CostFunction* cost_function = new ceres::AutoDiffCostFunction<AutoDiffPbSolvingDualQuaternionsWithWeights, 3, 6, 6, 6, 6, 6>(new AutoDiffPbSolvingDualQuaternionsWithWeights(OuterShell.NBVertices(), OuterShell.Vertices(), OuterShell.Normales(), OuterShell.Matches(), OuterShell.Weights(), OuterShell.Indices(),idxVer));
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
    std::cout << "x0 : " << (initial_x[0]) << " -> " << x[0] << "\n";
    std::cout << "x1 : " << (initial_x[1]) << " -> " << x[1] << "\n";
    std::cout << "x2 : " << (initial_x[2]) << " -> " << x[2] << "\n";
    std::cout << "x3 : " << (initial_x[3]) << " -> " << x[3] << "\n";
    std::cout << "x4 : " << (initial_x[4]) << " -> " << x[4] << "\n";
    std::cout << "x5 : " << (initial_x[5]) << " -> " << x[5] << "\n";
    std::cout << "x6 : " << (initial_x[6]) << " -> " << x[6] << "\n";
    std::cout << "x7 : " << (initial_x[7]) << " -> " << x[7] << "\n";


    std::cout << "x0 : " << (initial_x[8]) << " -> " << x[8] << "\n";
    std::cout << "x1 : " << (initial_x[9]) << " -> " << x[9] << "\n";
    std::cout << "x2 : " << (initial_x[10]) << " -> " << x[10] << "\n";
    std::cout << "x3 : " << (initial_x[11]) << " -> " << x[11] << "\n";
    std::cout << "x4 : " << (initial_x[12]) << " -> " << x[12] << "\n";
    std::cout << "x5 : " << (initial_x[13]) << " -> " << x[13] << "\n";
    std::cout << "x6 : " << (initial_x[14]) << " -> " << x[14] << "\n";
    std::cout << "x7 : " << (initial_x[15]) << " -> " << x[15] << "\n";

    Eigen::VectorXf residuWithourPerturbation(OuterShell.NBVertices());
    std::cout << "before computing RMSE" << std::endl;
    float totalErrorBefore = OuterShell.RMSE_with_update(residuWithourPerturbation);

    cout << "total error without perturbation is = "<<totalErrorBefore<<endl;


    // display the resulting
    std::vector<DualQuaternion> resulting_Warps;
    for(unsigned int i=0;i<OuterShell.NBNodes();++i){
        DualQuaternion currParameters = DualQuaternion(make_float3(x[6*i],x[6*i+1],x[6*i+2]),make_float3(x[6*i+3],x[6*i+4],x[6*i+5]));

        resulting_Warps.push_back(currParameters);
    }

    vector<Point3f> resultVertex3d;
    vector<Point3f> tildeVertex3d;
    
    float3 vtx;
    float3 nmle;

    vector<double> errorResult;
    double totalError = 0.0;

    Quaternion pointN = Quaternion(0.f, 0.f, 0.f, 0.f);
    // loop over all 
    // VERSION WITH BLENDING
    for(unsigned int idx=0 ; idx< OuterShell.NBVertices(); ++idx){
        int indx_Depth = idx;

        DualQuaternion point = DualQuaternion(Quaternion(0.0,0.0,0.0,1.0), Quaternion(OuterShell.Vertices()[3*(idx)], OuterShell.Vertices()[3*(idx)+1], OuterShell.Vertices()[3*(idx)+2], 0.0f));

        DualQuaternion Transfo = DualQuaternion(Quaternion(0.0,0.0,0.0,0.0), Quaternion(0.0,0.0,0.0,0.0));
        for(int v= 0; v<5; ++v){
            Transfo = Transfo +  resulting_Warps[OuterShell.Indices()[5*(idx)+v]]*OuterShell.Weights()[5*(idx)+v];
        }
        Transfo = Transfo.Normalize();

        point  = Transfo * point * Transfo.DualConjugate2();
        vtx = point.Dual().Vector();

        pointN = Quaternion(OuterShell.Normales()[3*(idx)], OuterShell.Normales()[3*(idx)+1], OuterShell.Normales()[3*(idx)+2], 0.0f);
        pointN  = Transfo.Real() * pointN * Transfo.Real().Conjugate();
        nmle = pointN.Vector();


        // compute the normal as well
        float scalarProd = nmle.x*(vtx.x - OuterShell.Matches()[3*indx_Depth]) + nmle.y*(vtx.y - OuterShell.Matches()[3*indx_Depth+1]) + nmle.z*(vtx.z - OuterShell.Matches()[3*indx_Depth+2]);
        totalError += std::abs(scalarProd);
        errorResult.push_back(totalError);

        resultVertex3d.push_back(Point3f(vtx.x,vtx.y,vtx.z));
        tildeVertex3d.push_back(Point3f(OuterShell.Matches()[3*indx_Depth],OuterShell.Matches()[3*indx_Depth+1],OuterShell.Matches()[3*indx_Depth+2]));
    }



    // VERSION WITHOUT BLENDING
    // for(unsigned int idx=0 ; idx<resulting_Warps.size() ; ++idx){
    //     int indx_Depth = idx;

    //     DualQuaternion Transfo = resulting_Warps[idx];

    //     DualQuaternion point = DualQuaternion(Quaternion(0.0,0.0,0.0,1.0), Quaternion(OuterShell.Vertices()[3*(idx)], OuterShell.Vertices()[3*(idx)+1], OuterShell.Vertices()[3*(idx)+2], 0.0f));
    //     point  = Transfo * point * Transfo.DualConjugate2();
    //     vtx = point.Dual().Vector();

    //     pointN = Quaternion(OuterShell.Normales()[3*(idx)], OuterShell.Normales()[3*(idx)+1], OuterShell.Normales()[3*(idx)+2], 0.0f);
    //     pointN  = Transfo.Real() * pointN * Transfo.Real().Conjugate();
    //     nmle = pointN.Vector();


    //     // compute the normal as well
    //     float scalarProd = nmle.x*(vtx.x - OuterShell.Matches()[3*indx_Depth]) + nmle.y*(vtx.y - OuterShell.Matches()[3*indx_Depth+1]) + nmle.z*(vtx.z - OuterShell.Matches()[3*indx_Depth+2]);
    //     totalError += std::abs(scalarProd);
    //     errorResult.push_back(totalError);

    //     resultVertex3d.push_back(Point3f(vtx.x,vtx.y,vtx.z));
    //     tildeVertex3d.push_back(Point3f(OuterShell.Matches()[3*indx_Depth],OuterShell.Matches()[3*indx_Depth+1],OuterShell.Matches()[3*indx_Depth+2]));
    // }

    std::cout << "total resulting error is "<< totalError << std::endl;

    // display the resulting
    viz::Viz3d cloudPointsWindowResult("Dual quaternion better");

    cloudPointsWindowResult.showWidget("coordinate", viz::WCoordinateSystem(10)); // default 100
    

    cloudPointsWindowResult.showWidget("pointsSphere", viz::WCloud(resultVertex3d, viz::Color::green()));
    cloudPointsWindowResult.setRenderingProperty( "pointsSphere", cv::viz::POINT_SIZE, 5.4 );
    cloudPointsWindowResult.showWidget("pointsCylind", viz::WCloud(tildeVertex3d, viz::Color::red()));
    cloudPointsWindowResult.setRenderingProperty( "pointsCylind", cv::viz::POINT_SIZE, 5.0 );

    cloudPointsWindowResult.spin();


	// glutInit(&argc, argv) ;	
	// glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	// glutInitWindowSize(width, height);
	// window = glutCreateWindow("Tetra Fusion Minimal") ;
	
	// // if (Init() != 0)
	// // 	return -1;

 //    glutReshapeFunc( reshapeWindow );
 //    glutDisplayFunc( renderScene );
 //    glutKeyboardFunc( handleKeyPress );
 //    setupScene();
 //    glutMainLoop();


    // glutInit( &argc, argv );
    // glutInitDisplayMode( GLUT_SINGLE | GLUT_RGB );
    // glutInitWindowSize( width, height );
    // glutCreateWindow( "Tetra Fusion Minimal" );
    // glutReshapeFunc( reshapeWindow );
    // glutDisplayFunc( renderScene );
    // glutKeyboardFunc( handleKeyPress );
    // setupScene();
    // glutMainLoop();


	return 0 ;
}

