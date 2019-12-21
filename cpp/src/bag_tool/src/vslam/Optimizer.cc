#include "Optimizer.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "so3.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include <Eigen/StdVector>
#include "KeyFrame.h"
#include "PanoMap.h"
namespace g2o {
    class VertexSO3 : public BaseVertex<3, Sophus::SO3>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexSO3(){}

        virtual void setToOriginImpl() {
          _estimate=Sophus::SO3();
        }

        virtual void oplusImpl(const double* update)
        {
            Sophus::SO3 up=Sophus::SO3::exp(Eigen::Vector3d(update[0], update[1], update[2]));
            _estimate = up*_estimate;
        }

        virtual bool read(std::istream& is){
            return true;
        }
        virtual bool write(std::ostream& os) const{
            return true;
        }

    };
    class EdgePreSO3 : public BaseUnaryEdge<3, Sophus::SO3, VertexSO3>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgePreSO3(){};

        bool read(std::istream& is){return true;};

        bool write(std::ostream& os) const{return true;};

        void computeError()  {
            const VertexSO3* v1 = static_cast<const VertexSO3*>(_vertices[0]);
            _error= (v1->estimate().inverse()*_measurement).log();
        }
    };
    class EdgeSO3 : public BaseBinaryEdge<3, Sophus::SO3, VertexSO3, VertexSO3>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeSO3(){};

        bool read(std::istream& is){return true;};

        bool write(std::ostream& os) const{return true;};

        void computeError()  {
            const VertexSO3* v1 = static_cast<const VertexSO3*>(_vertices[0]);
            const VertexSO3* v2 = static_cast<const VertexSO3*>(_vertices[1]);
            Sophus::SO3 C(_measurement);
            Sophus::SO3 error_=v1->estimate().inverse()*v2->estimate()*C.inverse();
            _error = error_.log();
        }
    };
}

namespace PANO
{
void BundleAdjustment(PanoMap * map){
    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        new g2o::BlockSolverX(
            new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()));
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);
    std::vector<g2o::VertexSO3*> v_list;
    std::map<int, g2o::VertexSO3*> map_id_vertex;
    for (int i=0; i<map->frames.size(); i++) {
        g2o::VertexSO3* vSO3 = new g2o::VertexSO3();
        vSO3->setEstimate(Sophus::SO3(map->frames[i]->direction));
        vSO3->setId(i);
        map->frames[i]->frame_id=i;
        map_id_vertex[i]=vSO3;
        v_list.push_back(vSO3);
        optimizer.addVertex(vSO3);
    }
    std::vector<g2o::EdgeSO3*> so3_edge_list;
    for (int i=0; i<map->rot_graph.size(); i++) {
        g2o::EdgeSO3* e= new g2o::EdgeSO3();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(map_id_vertex[map->rot_graph[i].frame1->frame_id]));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(map_id_vertex[map->rot_graph[i].frame2->frame_id]));
        e->setMeasurement(Sophus::SO3(map->rot_graph[i].rot_1_2));
        Eigen::Matrix<double, 3, 3> con_mat = Eigen::Matrix<double, 3, 3>::Identity();
        e->information()=con_mat;
        optimizer.addEdge(e);
        so3_edge_list.push_back(e);
    }
    std::vector<g2o::EdgePreSO3*> pre_edge_list;
    for (int i=0; i<map->frames.size(); i++) {
        g2o::EdgePreSO3* e= new g2o::EdgePreSO3();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(map_id_vertex[map->frames[i]->frame_id]));
        e->setMeasurement(Sophus::SO3(map->frames[i]->direction));
        Eigen::Matrix<double, 3, 3> con_mat = Eigen::Matrix<double, 3, 3>::Identity();
        e->information()=con_mat;
        optimizer.addEdge(e);
        pre_edge_list.push_back(e);
    }
    
    float avg_error=0;
    for(int i=0; i<pre_edge_list.size(); i++){
        pre_edge_list[i]->computeError();
        avg_error=avg_error+sqrt(pre_edge_list[i]->chi2())/pre_edge_list.size();
    }
    std::cout<<"pre edge err before: "<<avg_error<<std::endl;
    avg_error=0;
    for(int i=0; i<so3_edge_list.size(); i++){
        so3_edge_list[i]->computeError();
        avg_error=avg_error+sqrt(so3_edge_list[i]->chi2())/so3_edge_list.size();
        if(avg_error<0){
            return;
        }
    }
    std::cout<<"so3 edge err before: "<<avg_error<<std::endl;
    
    optimizer.initializeOptimization();
    optimizer.computeInitialGuess();
    optimizer.optimize(100);
    
    avg_error=0;
    for(int i=0; i<pre_edge_list.size(); i++){
        pre_edge_list[i]->computeError();
        avg_error=avg_error+sqrt(pre_edge_list[i]->chi2())/pre_edge_list.size();
    }
    std::cout<<"pre edge err before: "<<avg_error<<std::endl;
    avg_error=0;
    for(int i=0; i<so3_edge_list.size(); i++){
        so3_edge_list[i]->computeError();
        avg_error=avg_error+sqrt(so3_edge_list[i]->chi2())/so3_edge_list.size();
        if(avg_error<0){
            return;
        }
    }
    std::cout<<"so3 edge err before: "<<avg_error<<std::endl;
    for(int i=0;i<v_list.size(); i++){
        map->frames[i]->direction=v_list[i]->estimate().matrix();
    }
}

}
