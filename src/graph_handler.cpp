/** @file graph_handler.cpp
 * @brief Definition of class GraphHandler
 * @author Matteo Frosi
*/

#include <graph_handler.h>

#include <memory>

#include <boost/format.hpp>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_plane_prior.hpp>
#include <g2o/edge_plane_identity.hpp>
#include <g2o/edge_plane_parallel.hpp>
#include <g2o/robust_kernel_io.h>

/**
 * Linking to the various solvers.
 */
G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

G2O_USE_TYPE_GROUP(slam3d);

/**
 * Registering the custom g2o element types, to be saved later in a file.
 */
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
    G2O_REGISTER_TYPE(EDGE_PLANE_PERPENDICULAR, EdgePlanePerpendicular)
}

/**
 * Class constructor, it takes as input the name of the solver to be used for optimisation (default is Levenberg-Marquardt, LM).
 */
GraphHandler::GraphHandler(const std::string &solver_type) {
    this->graph = std::make_unique<g2o::SparseOptimizer>();
    g2o::SparseOptimizer* tmp_graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

    std::cout << "[GraphHandler::INFO] constructing solver: " << solver_type << std::endl;
    g2o::OptimizationAlgorithmFactory* solver_algo_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_algo_property;
    g2o::OptimizationAlgorithm* solver_algo = solver_algo_factory->construct(solver_type, solver_algo_property);
    tmp_graph->setAlgorithm(solver_algo);

    if(!tmp_graph->solver()) {
        std::cerr << std::endl;
        std::cerr << "[GraphHandler::ERROR] failed to construct solver" << std::endl;
        solver_algo_factory->listSolvers(std::cerr);
        std::cerr << "[GraphHandler::ERROR] --------------------------" << std::endl;
        std::cin.ignore(1);
        return;
    }

    std::cout << "[GraphHandler::INFO] done constructing solver" << std::endl;

    std::cout << "[GraphHandler::INFO] constructing kernel factory" << std::endl;
    this->robust_kernel_factory = g2o::RobustKernelFactory::instance();
    std::cout << "[GraphHandler::INFO] done constructing kernel factory" << std::endl;
}

/**
 * Class destructor. It deletes nodes and edges previously allocated (with new) and it clears the pointer to the pose graph.
 */
GraphHandler::~GraphHandler() {
   int miao = 0;
}

/**
 * Sets the solver used to optimise the pose graph, similarly as done in the class constructor.
 */
void GraphHandler::set_solver(const std::string &solver_type) {
    g2o::SparseOptimizer* tmp_graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

    std::cout << "[GraphHandler::INFO] setting solver: " << solver_type << std::endl;
    g2o::OptimizationAlgorithmFactory* solver_algo_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_algo_property;
    g2o::OptimizationAlgorithm* solver_algo = solver_algo_factory->construct(solver_type, solver_algo_property);
    tmp_graph->setAlgorithm(solver_algo);

    if(!tmp_graph->solver()) {
        std::cerr << std::endl;
        std::cerr << "[GraphHandler::ERROR] failed to set solver" << std::endl;
        solver_algo_factory->listSolvers(std::cerr);
        std::cerr << "[GraphHandler::ERROR] --------------------" << std::endl;
        std::cin.ignore(1);
        return;
    }

    std::cout << "[GraphHandler::INFO] done constructing solver: " << std::endl;
}

/**
 * Gets the number of nodes in the pose graph.
 */
int GraphHandler::num_vertices() const {
    return this->graph->vertices().size();
}

/**
 * Gets the number of edges in the pose graph.
 */
int GraphHandler::num_edges() const {
    return this->graph->edges().size();
}

/**
 * Adds a SE3 node to the graph.
 */
g2o::VertexSE3* GraphHandler::add_se3_node(const Eigen::Isometry3d& pose) {
    g2o::VertexSE3* vertex(new g2o::VertexSE3());
    vertex->setId(static_cast<int>(this->graph->vertices().size()));
    vertex->setEstimate(pose);
    this->graph->addVertex(vertex);

    return vertex;
}

/**
 * Adds a plane node to the graph, described by its coefficients (a,b,c and d of the plane equation "ax + by + cz + d = 0").
 */
g2o::VertexPlane* GraphHandler::add_plane_node(const Eigen::Vector4d& plane_coeffs) {
    g2o::VertexPlane* vertex(new g2o::VertexPlane());
    vertex->setId(static_cast<int>(this->graph->vertices().size()));
    vertex->setEstimate(plane_coeffs);
    this->graph->addVertex(vertex);

    return vertex;
}

/**
 * Adds a point_xyz to the graph, described by its 3D coordinates: x, y and z.
 */
g2o::VertexPointXYZ* GraphHandler::add_point_xyz_node(const Eigen::Vector3d& xyz) {
    g2o::VertexPointXYZ* vertex(new g2o::VertexPointXYZ());
    vertex->setId(static_cast<int>(graph->vertices().size()));
    vertex->setEstimate(xyz);
    this->graph->addVertex(vertex);

    return vertex;
}

/**
 * Adds an edge between two SE3 nodes, described by their relative pose and the corresponding information matrix.
 */
g2o::EdgeSE3* GraphHandler::add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix) {
    g2o::EdgeSE3* edge(new g2o::EdgeSE3());
    edge->vertices()[0] = v1;
    edge->vertices()[1] = v2;
    edge->setMeasurement(relative_pose);
    edge->setInformation(information_matrix);
    this->graph->addEdge(edge);

    return edge;
}

/**
 * Adds an edge between a SE3 node and a plane node, described by the plane coefficients w.r.t. the SE3 node and the
 * corresponding information matrix.
 */
g2o::EdgeSE3Plane* GraphHandler::add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix) {
    g2o::EdgeSE3Plane* edge(new g2o::EdgeSE3Plane());

    edge->vertices()[0] = v_se3;
    edge->vertices()[1] = v_plane;
    edge->setMeasurement(plane_coeffs);
    edge->setInformation(information_matrix);
    this->graph->addEdge(edge);

    return edge;
}

/**
 * Adds an edge between a SE3 node and a xyz_node, described by the difference of the coordinates of their poses and the
 * corresponding information matrix.
 */
g2o::EdgeSE3PointXYZ* GraphHandler::add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
    g2o::EdgeSE3PointXYZ* edge(new g2o::EdgeSE3PointXYZ());
    edge->vertices()[0] = v_se3;
    edge->vertices()[1] = v_xyz;
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    this->graph->addEdge(edge);

    return edge;
}

/**
 * Adds an edge between a plane node and itself, using a prior normal to adjust it, along with the corresponding information
 * matrix.
 */
g2o::EdgePlanePriorNormal* GraphHandler::add_plane_normal_prior_edge(g2o::VertexPlane* v_plane, const Eigen::Vector3d& normal, const Eigen::MatrixXd& information_matrix) {
    g2o::EdgePlanePriorNormal* edge(new g2o::EdgePlanePriorNormal());
    edge->vertices()[0] = v_plane;
    edge->setMeasurement(normal);
    edge->setInformation(information_matrix);
    this->graph->addEdge(edge);

    return edge;
}

/**
 * Adds an edge between a plane node and itself, using a prior distance to adjust it, along with the corresponding information
 * matrix.
 */
g2o::EdgePlanePriorDistance* GraphHandler::add_plane_distance_prior_edge(g2o::VertexPlane* v_plane, double distance, const Eigen::MatrixXd& information_matrix) {
    g2o::EdgePlanePriorDistance* edge(new g2o::EdgePlanePriorDistance());
    edge->vertices()[0] = v_plane;
    edge->setMeasurement(distance);
    edge->setInformation(information_matrix);
    this->graph->addEdge(edge);

    return edge;
}

/**
 * Adds an edge between a SE3 node and itself, using prior coordinates (x and y) to adjust its planar location, along with
 * the corresponding information matrix.
 */
g2o::EdgeSE3PriorXY* GraphHandler::add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix) {
    g2o::EdgeSE3PriorXY* edge(new g2o::EdgeSE3PriorXY());
    edge->vertices()[0] = v_se3;
    edge->setMeasurement(xy);
    edge->setInformation(information_matrix);
    this->graph->addEdge(edge);

    return edge;
}

/**
 * Adds an edge between a SE3 node and itself, using prior coordinates (x, y and z) to adjust its spatial location, along
 * with the corresponding information matrix.
 */
g2o::EdgeSE3PriorXYZ* GraphHandler::add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) {
    g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
    edge->vertices()[0] = v_se3;
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    this->graph->addEdge(edge);

    return edge;
}

/**
 * Adds an edge between a SE3 node and itself, using a prior orientation (in quaternion form) to adjust its orientation,
 * along with the corresponding information matrix.
 */
g2o::EdgeSE3PriorQuat* GraphHandler::add_se3_prior_quat_edge(g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat, const Eigen::MatrixXd& information_matrix) {
    g2o::EdgeSE3PriorQuat* edge(new g2o::EdgeSE3PriorQuat());
    edge->vertices()[0] = v_se3;
    edge->setMeasurement(quat);
    edge->setInformation(information_matrix);
    this->graph->addEdge(edge);

    return edge;
}

/**
 * Adds an edge between a SE3 node and itself, using a prior orientation (in 3-vector form) to adjust its orientation,
 * along with the corresponding information matrix.
 */
g2o::EdgeSE3PriorVec* GraphHandler::add_se3_prior_vec_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix) {
    Eigen::Matrix<double, 6, 1> m;
    m.head<3>() = direction;
    m.tail<3>() = measurement;

    g2o::EdgeSE3PriorVec* edge(new g2o::EdgeSE3PriorVec());
    edge->vertices()[0] = v_se3;
    edge->setMeasurement(m);
    edge->setInformation(information_matrix);
    this->graph->addEdge(edge);

    return edge;
}

/**
 * Adds an edge between two plane nodes, using their relative position (calculated using rotations and projections).
 */
g2o::EdgePlane* GraphHandler::add_plane_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information_matrix) {
    g2o::EdgePlane* edge(new g2o::EdgePlane());
    edge->vertices()[0] = v_plane1;
    edge->vertices()[1] = v_plane2;
    edge->setMeasurement(measurement);
    edge->setInformation(information_matrix);
    this->graph->addEdge(edge);

    return edge;
}

/**
 * Adds an edge between two plane nodes, using the distance between their four parameters (a, b, c and d).
 */
g2o::EdgePlaneIdentity* GraphHandler::add_plane_identity_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information_matrix) {
    g2o::EdgePlaneIdentity* edge(new g2o::EdgePlaneIdentity());
    edge->vertices()[0] = v_plane1;
    edge->vertices()[1] = v_plane2;
    edge->setMeasurement(measurement);
    edge->setInformation(information_matrix);
    this->graph->addEdge(edge);

    return edge;
}

/**
 * Adds an edge between two plane nodes, using the distance between their normals (a, b and c parameters).
 */
g2o::EdgePlaneParallel* GraphHandler::add_plane_parallel_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::Matrix3d& information_matrix) {
    g2o::EdgePlaneParallel* edge(new g2o::EdgePlaneParallel());
    edge->vertices()[0] = v_plane1;
    edge->vertices()[1] = v_plane2;
    edge->setMeasurement(measurement);
    edge->setInformation(information_matrix);
    this->graph->addEdge(edge);

    return edge;
}

/**
 * Adds an edge between two planes nodes which correspond to perpendicular planes. The error is computed as dot product
 * of their normals.
 */
g2o::EdgePlanePerpendicular * GraphHandler::add_plane_perpendicular_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix) {
    g2o::EdgePlanePerpendicular* edge(new g2o::EdgePlanePerpendicular());
    edge->vertices()[0] = v_plane1;
    edge->vertices()[1] = v_plane2;
    edge->setMeasurement(measurement);
    edge->setInformation(information_matrix);
    this->graph->addEdge(edge);

    return edge;
}

/**
 * Sets the kernel used for optimisation on an edge of the graph.
 */
void GraphHandler::add_robust_kernel(g2o::HyperGraph::Edge* edge, const std::string& kernel_type, double kernel_size) {
    if(kernel_type == "NONE") {
        return;
    }

    g2o::RobustKernel* kernel = this->robust_kernel_factory->construct(kernel_type);
    if(kernel == nullptr) {
        std::cerr << "[GraphHandler::ERROR] invalid robust kernel type: " << kernel_type << std::endl;
        return;
    }

    kernel->setDelta(kernel_size);

    g2o::OptimizableGraph::Edge* t_edge = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
    t_edge->setRobustKernel(kernel);
}

/**
 * Optimises the graph, using a maximum number of iterations (at the end, they can be less than that).
 */
int GraphHandler::optimize(int num_iterations) {
    g2o::SparseOptimizer* t_graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
    if(t_graph->edges().size() < 10) {
        return -1;
    }

    std::cout << std::endl;
    std::cout << "[GraphHandler::INFO] --- pose graph optimisation ---" << std::endl;
    std::cout << "[GraphHandler::INFO] nodes: " << t_graph->vertices().size() << ", edges: " << t_graph->edges().size() << std::endl;
    std::cout << "[GraphHandler::INFO] optimising..." << std::flush;

    std::cout << "[GraphHandler::INFO] init" << std::endl;
    t_graph->initializeOptimization();
    t_graph->setVerbose(true);

    std::cout << "[GraphHandler::INFO] chi2" << std::endl;
    double chi2 = t_graph->chi2();

    std::cout << "[GraphHandler::INFO] optimisation" << std::endl;
    auto begin_wall = std::chrono::high_resolution_clock::now();
    auto begin_clock = std::clock();
    int iterations = t_graph->optimize(num_iterations);

    auto end_wall = std::chrono::high_resolution_clock::now();
    auto end_clock = std::clock();

    double delta_wall = std::chrono::duration<double>(std::chrono::duration_cast<std::chrono::seconds>(end_wall - begin_wall)).count();
    double delta_clock = (end_clock - begin_clock) / (double) CLOCKS_PER_SEC;
    std::cout << "[GraphHandler::INFO] done" << std::endl;
    std::cout << "[GraphHandler::INFO] iterations: " << iterations << " / " << num_iterations << std::endl;
    std::cout << "[GraphHandler::INFO] chi2: (before) " << chi2 << " -> (after) " << t_graph->chi2() << std::endl;
    std::cout << "[GraphHandler::INFO] time (wall): " << boost::format("%.5f") % delta_wall << "[sec]" << std::endl;
    std::cout << "[GraphHandler::INFO] time (clock): " << boost::format("%.5f") % delta_clock << "[sec]" << std::endl;

    return iterations;
}

/**
 * Saves the pose graph and kernels to a file.
 */
void GraphHandler::save(const std::string& filename) {
    std::cout << "[GraphHandler::INFO] saving the pose graph to: " << filename << std::endl;
    g2o::SparseOptimizer* t_graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

    std::ofstream ofs(filename);
    t_graph->save(ofs);

    std::cout << "[GraphHandler::INFO] saving the kernels to " << filename + ".kernels" << std::endl;
    g2o::save_robust_kernels(filename + ".kernels", t_graph);

    std::cout << "[GraphHandler::INFO] done saving the pose graph" << std::endl;
}

/**
 * Loads the pose graph and kernels from a file.
 */
bool GraphHandler::load(const std::string &filename) {
    std::cout << "[GraphHandler::INFO] loading the pose graph from: " << filename << std::endl;
    g2o::SparseOptimizer* t_graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

    std::ifstream ifs(filename);
    if(!t_graph->load(ifs,true)) {
        std::cerr << "[GraphHandler::ERROR] failed to load the pose graph" << std::endl;
        return false;
    }

    std::cout << "[GraphHandler::INFO] #nodes: " << t_graph->vertices().size() << std::endl;
    std::cout << "[GraphHandler::INFO] #edges: " << t_graph->edges().size() << std::endl;


    if(!g2o::load_robust_kernels(filename + ".kernels", t_graph)) {
        std::cerr << "[GraphHandler::ERROR] failed to load the kernels" << std::endl;
        return false;
    }

    std::cout << "[GraphHandler::INFO] done loading the pose graph" << std::endl;
    return true;
}