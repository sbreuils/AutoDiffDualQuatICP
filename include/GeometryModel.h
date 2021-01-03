

#ifndef __GEOMETRY_MODEL_H
#define __GEOMETRY_MODEL_H


#include <vector>
#include "DualQuaternion.h"
#include <random>

using namespace std;
using namespace cv;



class GeometryModel {

private:

    std::vector<double> _Weights; // weight is 1
    std::vector<int> _Indices; // map 
    std::vector<DualQuaternionScalar<double> > _Warp;// set of nodes 

    std::vector<double> _Vertices;// all the vertices
    std::vector<double> _VMap; // target vertices 
    std::vector<double> _Normals; // normal to each vertex

    int planarNumberOfWeights;
    int planarNumberOfVertices;
    int _Nb_Weights = 0;
    int _Nb_Warps = 0;
    int _Nb_Vertices = 0;



    unsigned seed;
    
public:

    // constructor
	GeometryModel() { 
        const float radiusSphere = 2.0;
        planarNumberOfWeights = 12;
        planarNumberOfVertices = 15;
        _Nb_Vertices = planarNumberOfVertices*planarNumberOfVertices;
        _Nb_Weights = planarNumberOfWeights*planarNumberOfWeights;
        _Nb_Warps =planarNumberOfVertices*planarNumberOfVertices;
        
        cout << "init geometry ..." << endl;

    
    for(unsigned int i = 0 ; i< _Nb_Vertices ; ++i){
        _Vertices.push_back(radiusSphere);
        _Vertices.push_back(0.0);
        _Vertices.push_back(0.0);
    }

    // init normal vectors
    for(unsigned int i = 0 ; i< _Nb_Vertices ; ++i){
        _Normals.push_back(1.0);
        _Normals.push_back(0.0);
        _Normals.push_back(0.0);
    }
    
    
    // init the dual quatenrions corresponding to the transformations that we want to apply
    for(unsigned int i=0 ; i<planarNumberOfWeights ; ++i){
        for(unsigned int j=0 ; j<planarNumberOfWeights ; ++j){

            // Quaternion rotCurr = Quaternion(-sin(M_PI*i/(len*2.0))*sin(M_PI*j/len),cos(M_PI*i/(len*2.0))*sin(M_PI*j/len),sin(M_PI*i/(len*2.0))*cos(M_PI*j/len),cos(M_PI*i/(len*2.0))*cos(M_PI*j/len));
            QuaternionScalar<double> rotCurr = QuaternionScalar<double>(-sin(M_PI*i/(planarNumberOfWeights*2.0))*sin(M_PI*j/planarNumberOfWeights),cos(M_PI*i/(planarNumberOfWeights*2.0))*sin(M_PI*j/planarNumberOfWeights),sin(M_PI*i/(planarNumberOfWeights*2.0))*cos(M_PI*j/planarNumberOfWeights),cos(M_PI*i/(planarNumberOfWeights*2.0))*cos(M_PI*j/planarNumberOfWeights));
            DualQuaternionScalar<double> curr = DualQuaternionScalar<double>(rotCurr,make_Scalar3<double>(3.0,4.,2.0));

            _Warp.push_back(curr);

        }
    }



    // init the dual quatenrions corresponding to the transformations that we want to apply
    int idxVertex = 0;
    for(unsigned int i=0 ; i<planarNumberOfVertices ; ++i){
        for(unsigned int j=0 ; j<planarNumberOfVertices ; ++j){

            // Quaternion rotCurr = Quaternion(-sin(M_PI*i/(len*2.0))*sin(M_PI*j/len),cos(M_PI*i/(len*2.0))*sin(M_PI*j/len),sin(M_PI*i/(len*2.0))*cos(M_PI*j/len),cos(M_PI*i/(len*2.0))*cos(M_PI*j/len));
//            QuaternionScalar<double> rotCurr = QuaternionScalar<double>(-sin(M_PI*i/(planarNumberOfVertices*2.0))*sin(M_PI*j/planarNumberOfVertices),cos(M_PI*i/(planarNumberOfVertices*2.0))*sin(M_PI*j/planarNumberOfVertices),sin(M_PI*i/(planarNumberOfVertices*2.0))*cos(M_PI*j/planarNumberOfVertices),cos(M_PI*i/(planarNumberOfVertices*2.0))*cos(M_PI*j/planarNumberOfVertices));
            QuaternionScalar<double> rotCurr = QuaternionScalar<double>(-sin(M_PI*i/(planarNumberOfVertices*2.0))*sin(M_PI*j/planarNumberOfVertices),cos(M_PI*i/(planarNumberOfVertices*2.0))*sin(M_PI*j/planarNumberOfVertices),sin(M_PI*i/(planarNumberOfVertices*2.0))*cos(M_PI*j/planarNumberOfVertices),cos(M_PI*i/(planarNumberOfVertices*2.0))*cos(M_PI*j/planarNumberOfVertices));

            DualQuaternionScalar<double> curr = DualQuaternionScalar<double>(rotCurr,make_Scalar3<double>(3.0,4.0,2.0));
            // apply the transformation to the point (0,0,radius)
            DualQuaternionScalar<double> vertexInput = DualQuaternionScalar<double>(make_Scalar3<double>(0.0,0.0,0.0),QuaternionScalar<double>(radiusSphere,0.0,0.0,0.0));

            // apply the transformation 
            vertexInput = curr * vertexInput * curr.DualConjugate2();

            // find the 5th nearest nodes 
            for(unsigned int numNode = 0 ; numNode < 5 ; ++numNode){
                float minDistance = 1000.0; 
                int minCurrIndices = 0;
                for(unsigned int idxWarp=0 ; idxWarp<_Warp.size() ; ++idxWarp){
                    DualQuaternionScalar<double> curr = _Warp[idxWarp];

                    // compute the transformed position that is, the associated position
                    DualQuaternionScalar<double> nodeInput = DualQuaternionScalar<double>(make_Scalar3<double>(0.0,0.0,0.0),QuaternionScalar<double>(radiusSphere,0.0,0.0,0.0));
                    // apply the transformation 
                    nodeInput = curr * nodeInput * curr.DualConjugate2();

                    // compute the distance between the transformed vertex and the transformed node
                    Scalar3<double> currentnodePos = nodeInput.Dual().Vector();
                    Scalar3<double> currentVertexPos = vertexInput.Dual().Vector();

                    double currentDistance = sqrt((currentnodePos.x-currentVertexPos.x)*(currentnodePos.x-currentVertexPos.x) +
                                                  (currentnodePos.y-currentVertexPos.y)*(currentnodePos.y-currentVertexPos.y) +
                                                  (currentnodePos.z-currentVertexPos.z)*(currentnodePos.z-currentVertexPos.z));

                    if(currentDistance < minDistance){
                        // we cannot add an already added node to the current vertex
                        // minCurrIndices = ;
                        bool nodeAlreadyAdded = false;
                        for(int idxBeginning = 0 ; idxBeginning<numNode; idxBeginning++){
                            if(_Indices[5*idxVertex+idxBeginning] == idxWarp ){
                                nodeAlreadyAdded=true;
                            }
                        }
                        if(!nodeAlreadyAdded){
                            minDistance = currentDistance;
                            minCurrIndices = idxWarp;
                        }
                    }
                }
            _Indices.push_back(minCurrIndices);

            }

            idxVertex++;

        }
    }

    std::cout << "the indices map is filled!, nb elements = "<<  _Indices.size() << std::endl;

    // let us display some indices for a vertex




    //    for(unsigned int i=0 ; i<planarNumberOfVertices ; ++i){
//        for(unsigned int j=0 ; j<planarNumberOfVertices ; ++j){
//
//            // Quaternion rotCurr = Quaternion(-sin(M_PI*i/(len*2.0))*sin(M_PI*j/len),cos(M_PI*i/(len*2.0))*sin(M_PI*j/len),sin(M_PI*i/(len*2.0))*cos(M_PI*j/len),cos(M_PI*i/(len*2.0))*cos(M_PI*j/len));
//            QuaternionScalar<double> rotCurr = QuaternionScalar<double>(-sin(M_PI*i/(planarNumberOfVertices*2.0))*sin(M_PI*j/planarNumberOfVertices),cos(M_PI*i/(planarNumberOfVertices*2.0))*sin(M_PI*j/planarNumberOfVertices),sin(M_PI*i/(planarNumberOfVertices*2.0))*cos(M_PI*j/planarNumberOfVertices),cos(M_PI*i/(planarNumberOfVertices*2.0))*cos(M_PI*j/planarNumberOfVertices));
//            DualQuaternionScalar<double> curr = DualQuaternionScalar<double>(rotCurr,make_Scalar3<double>(3.0,4.,2.0));
//
//            // apply the transformation to the point (0,0,radius)
//            DualQuaternionScalar<double> vertexInput = DualQuaternionScalar<double>(make_Scalar3<double>(0.0f,0.0f,0.0f),QuaternionScalar<double>(radiusSphere,0.0f,0.0f,0.0f));
//
//            // apply the transformation
//            vertexInput = curr * vertexInput * curr.DualConjugate2();
//
//            // place the target the transformation
//             DualQuaternionScalar<double> transfoTarget =  DualQuaternionScalar<double>(make_Scalar3<double>(0.07f,0.0f,0.0f),make_Scalar3<double>(0.02,0.1,0.01)); // these are the transformations that has to be found in the computation of the Jacobian
//             vertexInput = transfoTarget * vertexInput * transfoTarget.DualConjugate2();
//
//
//            // float3 vertexOut = vertexInput.Dual().Vector();
//            _VMap.push_back(vertexInput.Dual().Vector().x);
//            _VMap.push_back(vertexInput.Dual().Vector().y);
//            _VMap.push_back(vertexInput.Dual().Vector().z);
//
//
//        }
//    }


    // let us now look for the nearest weights wi
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> uniformRealDistributionWeights(0,0.2);


    // init the weights ; 
    for(unsigned int i=0 ; i<_Nb_Vertices ; ++i){ 
        _Weights.push_back(0.2);
        _Weights.push_back(0.2);
        _Weights.push_back(0.2);
        _Weights.push_back(0.2); // \todo random 
        _Weights.push_back(0.2); 
    }


    // init the dual quaternions corresponding to the target estimate or ptilde
    // these dual quaternions are the base for the computation of the
    // Jacobian
    for (int idx = 0; idx < _Nb_Vertices; idx++) {
        // Warp each vertex with the current state of the warp field
        // Blend the transformations
        DualQuaternionScalar<double> Transfo = DualQuaternionScalar<double>(QuaternionScalar<double>(0.0, 0.0, 0.0, 0.0),
                                                                            QuaternionScalar<double>(0.0, 0.0, 0.0, 0.0));
        for (int v = 0; v < 5; v++) {
            Transfo = Transfo + _Warp[_Indices[(idx) + v]] * _Weights[(idx) + v];
        }
        Transfo = Transfo.Normalize();

        DualQuaternionScalar<double> point = DualQuaternionScalar<double>(QuaternionScalar<double>(0.0, 0.0, 0.0, 1.0),
                                                                          QuaternionScalar<double>(_Vertices[3 * (idx)],
                                                                                                   _Vertices[3 * (idx) + 1],
                                                                                                   _Vertices[3 * (idx) + 2], 0.0f));
        point = Transfo * point * Transfo.DualConjugate2();

        // apply a perturbation
        DualQuaternionScalar<double> transfoTarget =  DualQuaternionScalar<double>(make_Scalar3<double>(0.07f,0.0f,0.0f),make_Scalar3<double>(0.02,0.1,0.01)); // these are the transformations that has to be found in the computation of the Jacobian
        point = transfoTarget * point * transfoTarget.DualConjugate2();


        Scalar3<double> vtx = point.Dual().Vector();
        _VMap.push_back(vtx.x);
        _VMap.push_back(vtx.y);
        _VMap.push_back(vtx.z);
    }




    }

	// destructor
	virtual ~GeometryModel() {

    };

    
    inline int NBVertices() {return _Nb_Vertices;}
    inline int NBNodes() {return _Nb_Weights;}
    
    
    inline std::vector<double> Vertices() {return _Vertices;}
    
    inline std::vector<double> Normales() {return _Normals;}
    

    inline std::vector<double> Matches() {return _VMap;}

    
    inline std::vector<double> Weights() {return _Weights;}
    
    inline std::vector<int> Indices() {return _Indices;}


    inline std::vector<DualQuaternionScalar<double> > Warp() {return _Warp;}

    double RMSE_with_update(Eigen::VectorXd& residual);

    //void InitializeSolver();

    //void CallOptimization();
};




#endif
