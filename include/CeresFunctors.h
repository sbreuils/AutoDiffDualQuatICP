
#ifndef __CERES_FUNCTORS_H
#define __CERES_FUNCTORS_H


#include <stdio.h>
#include "include/DualQuaternion.h"
#include "ceres/ceres.h"
#include "ceres/dynamic_autodiff_cost_function.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <random>


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::DynamicAutoDiffCostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;




struct AutoDiffPbSolvingDualQuaternionsWithWeights{

  AutoDiffPbSolvingDualQuaternionsWithWeights(const int nbVertices, std::vector<double> vertices, std::vector<double> normal, std::vector<double> ptil, std::vector<double> weights, std::vector<int> indices, const int id) : Nvertex(nbVertices),
                                                                                                                                     _vertices(vertices),
                                                                                                                                     _normal(normal),
                                                                                                                                     _ptilde(ptil),
                                                                                                                                     _weights(weights),
                                                                                                                                     _indices(indices),
                                                                                                                                    _idx(id)
  {

  }

  template <typename Q>
  bool operator()(const Q* const x1,const Q* const x2,const Q* const x3,const Q* const x4,const Q* const x5, Q* residual) const { 
    int tot_matches = 0;
    float error = 0.0f;

    DualQuaternionScalar<Q> Transfo1 = DualQuaternionScalar<Q>(make_Scalar3<Q>((Q)x1[0],(Q)x1[1],(Q)x1[2]),
                                                            make_Scalar3<Q>((Q)x1[3],(Q)x1[4],(Q)x1[5])) ;//QuaternionScalar<Q>(Q(0.0),Q(0.0),Q(0.0),Q(0.0));

    DualQuaternionScalar<Q> Transfo2 = DualQuaternionScalar<Q>(make_Scalar3<Q>((Q)x2[0],(Q)x2[1],(Q)x2[2]),
                                                            make_Scalar3<Q>((Q)x2[3],(Q)x2[4],(Q)x2[5])) ;//QuaternionScalar<Q>(Q(0.0),Q(0.0),Q(0.0),Q(0.0));

    DualQuaternionScalar<Q> Transfo3 = DualQuaternionScalar<Q>(make_Scalar3<Q>((Q)x3[0],(Q)x3[1],(Q)x3[2]),
                                                            make_Scalar3<Q>((Q)x3[3],(Q)x1[4],(Q)x3[5])) ;//QuaternionScalar<Q>(Q(0.0),Q(0.0),Q(0.0),Q(0.0));

    DualQuaternionScalar<Q> Transfo4 = DualQuaternionScalar<Q>(make_Scalar3<Q>((Q)x4[0],(Q)x4[1],(Q)x4[2]),
                                                            make_Scalar3<Q>((Q)x4[3],(Q)x4[4],(Q)x4[5])) ;//QuaternionScalar<Q>(Q(0.0),Q(0.0),Q(0.0),Q(0.0));

    DualQuaternionScalar<Q> Transfo5 = DualQuaternionScalar<Q>(make_Scalar3<Q>((Q)x5[0],(Q)x5[1],(Q)x5[2]),
                                                            make_Scalar3<Q>((Q)x5[3],(Q)x5[4],(Q)x5[5])) ;//QuaternionScalar<Q>(Q(0.0),Q(0.0),Q(0.0),Q(0.0));


    DualQuaternionScalar<Q> point = DualQuaternionScalar<Q>(QuaternionScalar<Q>((Q)0.0,(Q)0.0,(Q)0.0,(Q)1.0),
                                                            QuaternionScalar<Q>((Q)_vertices[3*(_idx)], (Q)_vertices[3*(_idx)+1], (Q)_vertices[3*(_idx)+2],(Q)0.0) );//QuaternionScalar<Q>(Q(0.0),Q(0.0),Q(0.0),Q(0.0));
    DualQuaternionScalar<Q> pointN = DualQuaternionScalar<Q>(QuaternionScalar<Q>((Q)0.0,(Q)0.0,(Q)0.0,(Q)1.0),
                                                            QuaternionScalar<Q>((Q)_normal[3*(_idx)], (Q)_normal[3*(_idx)+1], (Q)_normal[3*(_idx)+2],(Q)0.0) );//QuaternionScalar<Q>(Q(0.0),Q(0.0),Q(0.0),Q(0.0));




    // DualQuaternion point = DualQuaternion(Quaternion(0.0,0.0,0.0,1.0), Quaternion(OuterShell.Vertices()[3*(idx)], OuterShell.Vertices()[3*(idx)+1], OuterShell.Vertices()[3*(idx)+2], 0.0f));
    DualQuaternionScalar<Q> Transfo = DualQuaternionScalar<Q>(QuaternionScalar<Q>((Q)0.0,(Q)0.0,(Q)0.0,(Q)0.0), QuaternionScalar<Q>((Q)0.0,(Q)0.0,(Q)0.0,(Q)0.0));

    Transfo = Transfo +  Transfo1*((Q)_weights[5*(_idx)+0]); //OuterShell.Warp()[_indices[5*(_idx)+v]]*OuterShell.Weights()[5*(_idx)+v];
    Transfo = Transfo +  Transfo2*((Q)_weights[5*(_idx)+1]);
    Transfo = Transfo +  Transfo3*((Q)_weights[5*(_idx)+2]);
    Transfo = Transfo +  Transfo4*((Q)_weights[5*(_idx)+3]);
    Transfo = Transfo +  Transfo5*((Q)_weights[5*(_idx)+4]);

    Transfo = Transfo.Normalize();

    DualQuaternionScalar<Q> transformedPoint  = Transfo * point * Transfo.DualConjugate2();




    Scalar3<Q> vtx, nmle;

    int indx_Depth=_idx;
    Q dist, prod_scal;
    vtx = transformedPoint.Dual().Vector();

        
    // pointN = QuaternionScalar<T>(_normal[3*(_idx)], _normal[3*(_idx)+1], _normal[3*(_idx)+2], 0.0f);
    // pointN  = Transfo * pointN * Transfo.Conjugate();
    // nmle = pointN.Vector();

    
    //Compute depth difference at reprojection error (reject outliers, by using normals also)
    // indx_Depth = _idx;
    // prod_scal = (vtx.x - _ptilde[3*_idx]) + (vtx.y - _ptilde[3*_idx+1]) + (vtx.z - _ptilde[3*_idx+2]); //(vtx.x - _ptilde[3*indx_Depth]) + (vtx.y - _ptilde[3*indx_Depth+1]) + (vtx.z - _ptilde[3*indx_Depth+2]);


    //Store correspondences into the matches array
    // error += fabs(dist);
    // tot_matches++;
    // residual(idx) = dist;//dist; STEPHANE
    std::cout << "out point  (" << _idx << ") = " << vtx.x << "-> " << vtx.y << ", "<< vtx.z << ";; "<< std::endl;
    std::cout << "ptilde  (" << _idx << ") = " << _ptilde[3*_idx] << "-> " << _ptilde[3*_idx+1] << ", "<< _ptilde[3*_idx+2] << "; "<< std::endl;

    residual[0] = (vtx.x - _ptilde[3*_idx]  );
    residual[1] = (vtx.y - _ptilde[3*_idx+1]);
    residual[2] = (vtx.z - _ptilde[3*_idx+2]);
        

    return true;
  }

  int Nvertex;
  std::vector<double> _vertices; // length 3 vector 
  std::vector<double> _normal; // length 3 vector

  std::vector<double> _ptilde;// length 3 vector
  std::vector<double> _weights;// weights
  std::vector<int> _indices;// Maps the weights associated to vertices and normals
  int _idx;

};







#endif 