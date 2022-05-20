#include "graph_handler.h"

#include <boost/format.hpp>
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <artslam_g2o/edge_se3_plane.hpp>
#include <artslam_g2o/edge_se3_priorxy.hpp>
#include <artslam_g2o/edge_se3_priorxyz.hpp>
#include <artslam_g2o/edge_se3_priorvec.hpp>
#include <artslam_g2o/edge_se3_priorquat.hpp>
#include <artslam_g2o/edge_plane_prior.hpp>
#include <artslam_g2o/edge_plane_identity.hpp>
#include <artslam_g2o/edge_plane_parallel.hpp>
#include <memory>
#include <artslam_g2o/robust_kernel_io.hpp>

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)  // be aware of that cholmod brings GPL dependency
G2O_USE_OPTIMIZATION_LIBRARY(csparse)  // be aware of that csparse brings LGPL unless it is dynamically linked

namespace g2o {
    G2O_REGISTER_TYPE(EDGE_SE3_PLANE, EdgeSE3Plane)
    G2O_REGISTER_TYPE(EDGE_SE3_PRIORXY, EdgeSE3PriorXY)
    G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
    G2O_REGISTER_TYPE(EDGE_SE3_PRIORVEC, EdgeSE3PriorVec)
    G2O_REGISTER_TYPE(EDGE_SE3_PRIORQUAT, EdgeSE3PriorQuat)
    G2O_REGISTER_TYPE(EDGE_PLANE_PRIOR_NORMAL, EdgePlanePriorNormal)
    G2O_REGISTER_TYPE(EDGE_PLANE_PRIOR_DISTANCE, EdgePlanePriorDistance)
    G2O_REGISTER_TYPE(EDGE_PLANE_IDENTITY, EdgePlaneIdentity)
    G2O_REGISTER_TYPE(EDGE_PLANE_PARALLEL, EdgePlaneParallel)
    G2O_REGISTER_TYPE(EDGE_PLANE_PAERPENDICULAR, EdgePlanePerpendicular)
}  // namespace g2o

using namespace artslam::laser3d;

GraphHandler::GraphHandler(const std::string &solver_type) {
    graph_ = std::make_unique<g2o::SparseOptimizer>();
    g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(graph_.get());

    std::cout << "construct solver: " << solver_type << std::endl;
    g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);
    graph->setAlgorithm(solver);

    if(!graph->solver()) {
        std::cerr << std::endl;
        std::cerr << "error : failed to allocate solver!!" << std::endl;
        solver_factory->listSolvers(std::cerr);
        std::cerr << "-------------" << std::endl;
        std::cin.ignore(1);
        return;
    }

    robust_kernel_factory_ = g2o::RobustKernelFactory::instance();
}

GraphHandler::~GraphHandler() {
    graph_.reset();
}

void GraphHandler::set_solver(const std::string& solver_type) const {
    g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(graph_.get());

    std::cout << "construct solver: " << solver_type << std::endl;
    g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);
    graph->setAlgorithm(solver);

    if(!graph->solver()) {
        std::cerr << std::endl;
        std::cerr << "error : failed to allocate solver!!" << std::endl;
        solver_factory->listSolvers(std::cerr);
        std::cerr << "-------------" << std::endl;
        std::cin.ignore(1);
        return;
    }
    std::cout << "done" << std::endl;
}

int GraphHandler::num_vertices() const {
    return graph_->vertices().size();
}
int GraphHandler::num_edges() const {
    return graph_->edges().size();
}

g2o::VertexSE3* GraphHandler::add_se3_node(const Eigen::Isometry3d& pose) const {
    g2o::VertexSE3* vertex(new g2o::VertexSE3());
    vertex->setId(static_cast<int>(graph_->vertices().size()));
    vertex->setEstimate(pose);
    graph_->addVertex(vertex);

    return vertex;
}

g2o::VertexPlane* GraphHandler::add_plane_node(const Eigen::Vector4d& plane_coeffs) const {
    g2o::VertexPlane* vertex(new g2o::VertexPlane());
    vertex->setId(static_cast<int>(graph_->vertices().size()));
    vertex->setEstimate(plane_coeffs);
    graph_->addVertex(vertex);

    return vertex;
}

g2o::VertexPointXYZ* GraphHandler::add_point_xyz_node(const Eigen::Vector3d& xyz) const {
    g2o::VertexPointXYZ* vertex(new g2o::VertexPointXYZ());
    vertex->setId(static_cast<int>(graph_->vertices().size()));
    vertex->setEstimate(xyz);
    graph_->addVertex(vertex);

    return vertex;
}

g2o::EdgeSE3* GraphHandler::add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix) const {
    g2o::EdgeSE3* edge(new g2o::EdgeSE3());
    edge->setMeasurement(relative_pose);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v1;
    edge->vertices()[1] = v2;
    graph_->addEdge(edge);

    return edge;
}

g2o::EdgeSE3Plane* GraphHandler::add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix) const {
    g2o::EdgeSE3Plane* edge(new g2o::EdgeSE3Plane());
    edge->setMeasurement(plane_coeffs);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    edge->vertices()[1] = v_plane;
    graph_->addEdge(edge);

    return edge;
}

g2o::EdgeSE3PointXYZ* GraphHandler::add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) const {
    g2o::EdgeSE3PointXYZ* edge(new g2o::EdgeSE3PointXYZ());
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    edge->vertices()[1] = v_xyz;
    graph_->addEdge(edge);

    return edge;
}

g2o::EdgePlanePriorNormal* GraphHandler::add_plane_normal_prior_edge(g2o::VertexPlane* v, const Eigen::Vector3d& normal, const Eigen::MatrixXd& information_matrix) const {
    g2o::EdgePlanePriorNormal* edge(new g2o::EdgePlanePriorNormal());
    edge->setMeasurement(normal);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v;
    graph_->addEdge(edge);

    return edge;
}

g2o::EdgePlanePriorDistance* GraphHandler::add_plane_distance_prior_edge(g2o::VertexPlane* v, double distance, const Eigen::MatrixXd& information_matrix) const {
    g2o::EdgePlanePriorDistance* edge(new g2o::EdgePlanePriorDistance());
    edge->setMeasurement(distance);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v;
    graph_->addEdge(edge);

    return edge;
}

g2o::EdgeSE3PriorXY* GraphHandler::add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix) const {
    g2o::EdgeSE3PriorXY* edge(new g2o::EdgeSE3PriorXY());
    edge->setMeasurement(xy);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_->addEdge(edge);

    return edge;
}

g2o::EdgeSE3PriorXYZ* GraphHandler::add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) const {
    g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_->addEdge(edge);

    return edge;
}

g2o::EdgeSE3PriorVec* GraphHandler::add_se3_prior_vec_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix) const {
    Eigen::Matrix<double, 6, 1> m;
    m.head<3>() = direction;
    m.tail<3>() = measurement;

    g2o::EdgeSE3PriorVec* edge(new g2o::EdgeSE3PriorVec());
    edge->setMeasurement(m);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_->addEdge(edge);

    return edge;
}

g2o::EdgeSE3PriorQuat* GraphHandler::add_se3_prior_quat_edge(g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat, const Eigen::MatrixXd& information_matrix) const {
    g2o::EdgeSE3PriorQuat* edge(new g2o::EdgeSE3PriorQuat());
    edge->setMeasurement(quat);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_->addEdge(edge);

    return edge;
}

g2o::EdgePlane* GraphHandler::add_plane_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information) const {
    g2o::EdgePlane* edge(new g2o::EdgePlane());
    edge->setMeasurement(measurement);
    edge->setInformation(information);
    edge->vertices()[0] = v_plane1;
    edge->vertices()[1] = v_plane2;
    graph_->addEdge(edge);

    return edge;
}

g2o::EdgePlaneIdentity* GraphHandler::add_plane_identity_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information) const {
    g2o::EdgePlaneIdentity* edge(new g2o::EdgePlaneIdentity());
    edge->setMeasurement(measurement);
    edge->setInformation(information);
    edge->vertices()[0] = v_plane1;
    edge->vertices()[1] = v_plane2;
    graph_->addEdge(edge);

    return edge;
}

g2o::EdgePlaneParallel* GraphHandler::add_plane_parallel_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::Matrix3d& information) const {
    g2o::EdgePlaneParallel* edge(new g2o::EdgePlaneParallel());
    edge->setMeasurement(measurement);
    edge->setInformation(information);
    edge->vertices()[0] = v_plane1;
    edge->vertices()[1] = v_plane2;
    graph_->addEdge(edge);

    return edge;
}

g2o::EdgePlanePerpendicular* GraphHandler::add_plane_perpendicular_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information) const {
    g2o::EdgePlanePerpendicular* edge(new g2o::EdgePlanePerpendicular());
    edge->setMeasurement(measurement);
    edge->setInformation(information);
    edge->vertices()[0] = v_plane1;
    edge->vertices()[1] = v_plane2;
    graph_->addEdge(edge);

    return edge;
}

void GraphHandler::add_robust_kernel(g2o::HyperGraph::Edge* edge, const std::string& kernel_type, double kernel_size) const {
    if(kernel_type == "NONE") {
        return;
    }

    g2o::RobustKernel* kernel = robust_kernel_factory_->construct(kernel_type);
    if(kernel == nullptr) {
        std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
        return;
    }

    kernel->setDelta(kernel_size);

    g2o::OptimizableGraph::Edge* edge_ = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
    edge_->setRobustKernel(kernel);
}

int GraphHandler::optimize(int num_iterations) {
    std::stringstream msg;
    auto start = std::chrono::high_resolution_clock::now();

    g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(graph_.get());
    if(graph->edges().size() < 10) {
        return -1;
    }

    std::cout << std::endl;
    std::cout << "--- pose graph optimization ---" << std::endl;
    /*std::cout << "nodes: " << graph->vertices().size() << "   edges: " << graph->edges().size() << std::endl;
    std::cout << "optimizing... " << std::flush;*/

    //std::cout << "init" << std::endl;
    graph->initializeOptimization();
    graph->setVerbose(false);

    //std::cout << "chi2" << std::endl;
    double chi2 = graph->chi2();

    //std::cout << "optimize!!" << std::endl;
    auto t1 = std::chrono::high_resolution_clock::now();
    int iterations = graph->optimize(num_iterations);

    // TODO check if this is correct
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double,std::ratio<1,1>> duration = t2 - t1;
    /*std::cout << "done" << std::endl;
    std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
    std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;*/
    std::cout << "time: " << boost::format("%.3f") % duration.count() << "[sec]" << std::endl;

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    total_time_ += duration2.count();
    count_++;

    if(verbose_) {
        msg << "[LoopDetector] Total time and count: " << total_time_ << ", " << count_ << "\n";
        std::cout << msg.str();
    }

    return iterations;
}

void GraphHandler::save(const std::string& filename) const {
    g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(graph_.get());

    std::ofstream ofs(filename);
    graph->save(ofs);

    g2o::save_robust_kernels(filename + ".kernels", graph);
}

bool GraphHandler::load(const std::string& filename) const {
    std::cout << "loading pose graph..." << std::endl;
    g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(graph_.get());

    std::ifstream ifs(filename);
    if(!graph->load(ifs)) {
        return false;
    }

    std::cout << "nodes  : " << graph->vertices().size() << std::endl;
    std::cout << "edges  : " << graph->edges().size() << std::endl;

    if(!g2o::load_robust_kernels(filename + ".kernels", graph)) {
        return false;
    }

    return true;
}