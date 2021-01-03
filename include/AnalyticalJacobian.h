
#ifndef DUALQUATICPAUTO_ANALYTICALJACOBIAN_H
#define DUALQUATICPAUTO_ANALYTICALJACOBIAN_H


void Compute_Jacobian(std::vector<Eigen::Triplet<float>>& coefficientsJ, float *Vertices, float *Normales, Eigen::VectorXf& residual, DualQuaternion * Warp, float * Weights, int *Indices, int * matches, int* MatchesList, int *Perm, float * VMap, InputFrame *Frame, int nb_N, int nb_V, int nb_matches, int fact) {
    //float* VMap = Frame->getVMap();

}


#endif //DUALQUATICPAUTO_ANALYTICALJACOBIAN_H
