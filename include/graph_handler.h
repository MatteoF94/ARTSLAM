/** @file graph_handler.h
 * @brief Declaration of class GraphHandler
 * @author Matteo Frosi
*/

#ifndef ARTSLAM_GRAPH_HANDLER_H
#define ARTSLAM_GRAPH_HANDLER_H


#include <memory>

#include <g2o/core/hyper_graph.h>

namespace g2o {
    class VertexSE3;
    class VertexPlane;
    class VertexPointXYZ;
    class EdgeSE3;
    class EdgeSE3Plane;
    class EdgeSE3PointXYZ;
    class EdgeSE3PriorXY;
    class EdgeSE3PriorXYZ;
    class EdgeSE3PriorVec;
    class EdgeSE3PriorQuat;
    class EdgePlane;
    class EdgePlaneIdentity;
    class EdgePlaneParallel;
    class EdgePlanePerpendicular;
    class EdgePlanePriorNormal;
    class EdgePlanePriorDistance;
    class RobustKernelFactory;
}

/**
 * @class GraphHandler
 * @brief This class handles the pose graph. It adds nodes and edges; it saves and loads the pose graph and sets the
 * solver used for optimisation.
 */
class GraphHandler {
public:
    /**
     * @brief Class constructor, with parameter.
     * @param solver_type The name of the solver (algorithm) used for optimisation. The default one is Levenberg-Marquardt (LM).
     */
    GraphHandler(const std::string& solver_type = "lm_var");

    /**
     * @brief Class destructor.
     */
    virtual ~GraphHandler();

    /**
     * @brief Gets the number of nodes in the pose graph.
     * @return The number of nodes in the pose graph.
     */
    int num_vertices() const;

    /**
     * @brief Gets the number of edges in the pose graph.
     * @return The number of edges in the pose graph.
     */
    int num_edges() const;

    /**
     * @brief Sets the solver (algorithm) used for optimisation.
     * @param solver_type The name of the solver.
     */
    void set_solver(const std::string& solver_type);

    /**
     * @brief Adds a SE3 node to the graph.
     * @param pose The pose of the node
     * @return The node registered in the graph.
     */
    g2o::VertexSE3* add_se3_node(const Eigen::Isometry3d& pose);

    /**
     * @brief Adds a plane node to the graph
     * @param plane_coeffs The coefficients of the plane, expressed as "ax + by + cz + d = 0".
     * @return The node registered in the graph.
     */
    g2o::VertexPlane* add_plane_node(const Eigen::Vector4d& plane_coeffs);

    /**
     * @brief Adds a point_xyz to the graph.
     * @param xyz The vector containing the 3D coordinates of the point.
     * @return The node registered in the graph.
     */
    g2o::VertexPointXYZ* add_point_xyz_node(const Eigen::Vector3d& xyz);

    /**
     * @brief Adds an edge between two SE3 nodes.
     * @param v1 The first node.
     * @param v2 The second node.
     * @param relative_pose The relative pose between the two nodes.
     * @param information_matrix The information matrix associated to the edge (mandatory 6x6 size).
     * @return The registered edge in the graph.
     */
    g2o::EdgeSE3* add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix);

    /**
     * @brief Adds an edge between a SE3 node and a plane node.
     * @param v_se3 The SE3 node.
     * @param v_plane The plane node.
     * @param plane_coeffs The plane coefficients w.r.t. the SE3 node.
     * @param information_matrix The information matrix associated to the edge (mandatory 3x3 size).
     * @return The registered edge in the graph.
     */
    g2o::EdgeSE3Plane* add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix);

    /**
     * @brief Adds an edge between a SE3 node and a xyz_node.
     * @param v_se3 The SE3 node.
     * @param v_xyz The xyz_node.
     * @param xyz The difference between the poses of the two nodes.
     * @param information_matrix The information matrix associated to the edge (mandatory 3x3 size).
     * @return The registered edge in the graph.
     */
    g2o::EdgeSE3PointXYZ* add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix);

    /**
     * @brief Adds an edge between a plane node and itself, to adjust its normal.
     * @param v_plane The plane node.
     * @param normal The normal.
     * @param information_matrix The information matrix associated to the edge ().
     * @return The registered edge in the graph.
     */
    g2o::EdgePlanePriorNormal* add_plane_normal_prior_edge(g2o::VertexPlane* v_plane, const Eigen::Vector3d& normal, const Eigen::MatrixXd& information_matrix);

    /**
     * @brief Adds an edge between a plane node and itself, to adjust its location.
     * @param v_plane The plane node.
     * @param distance The distance w.r.t. the real position of the plane (orthogonal distance).
     * @param information_matrix The information matrix associated to the edge ().
     * @return The registered edge in the graph.
     */
    g2o::EdgePlanePriorDistance* add_plane_distance_prior_edge(g2o::VertexPlane* v_plane, double distance, const Eigen::MatrixXd& information_matrix);

    /**
     * @brief Adds an edge between a SE3 node and itself, adjusting its planar location.
     * @param v_se3 The SE3 node.
     * @param xy The x and y coordinates used to adjust the node pose (e.g. from GPS).
     * @param information_matrix The information matrix associated to the edge ().
     * @return The registered edge in the graph.
     */
    g2o::EdgeSE3PriorXY* add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix);

    /**
     * @brief Adds an edge between a SE3 node and itself, adjusting its spatial location.
     * @param v_se3 The SE3 node.
     * @param xyz The x, y and z coordinates used to adjust the node pose (e.g. from GPS).
     * @param information_matrix The information matrix associated to the edge ().
     * @return The registered edge in the graph.
     */
    g2o::EdgeSE3PriorXYZ* add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix);

    /**
     * @brief Adds an edge between a SE3 node and itself, adjusting its orientation.
     * @param v_se3 The SE3 node.
     * @param quat The orientation (in quaternion form) used to adjust the node orientation (e.g. from IMU).
     * @param information_matrix The information matrix associated to the edge ().
     * @return The registered edge in the graph.
     */
    g2o::EdgeSE3PriorQuat* add_se3_prior_quat_edge(g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat, const Eigen::MatrixXd& information_matrix);

    /**
     * @brief Adds an edge between a SE3 node and itself, adjusting its orientation.
     * @param v_se3 The SE3 node.
     * @param direction The orientation (in 3-vector form).
     * @param measurement The measured orientation used to adjust the node orientation (e.g. from IMU).
     * @param information_matrix The information matrix associated to the edge ().
     * @return The registered edge in the graph.
     */
    g2o::EdgeSE3PriorVec* add_se3_prior_vec_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix);

    /**
     * @brief Adds an edge between two plane nodes.
     * @param v_plane1 The first plane node.
     * @param v_plane2 The second plane node.
     * @param measurement The relative position between the plane nodes (difference calculated with rotations).
     * @param information The information matrix associated to the edge (mandatory 4x4 size).
     * @return The registered edge in the graph.
     */
    g2o::EdgePlane* add_plane_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information_matrix);

    /**
     * @brief Adds an edge between two plane nodes, evaluating the distance between their four parameters (a, b, c and d).
     * @param v_plane1 The first plane node.
     * @param v_plane2 The second plane node.
     * @param measurement The relative position between the plane nodes (difference in normals and distance).
     * @param information The information matrix associated to the edge (mandatory 4x4 size).
     * @return The registered edge in the graph.
     */
    g2o::EdgePlaneIdentity* add_plane_identity_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information_matrix);

    /**
     * @brief Adds an edge between two plane nodes, evaluating the distance between their normals.
     * @param v_plane1 The first plane node.
     * @param v_plane2 The second plane node.
     * @param measurement The relative position between the plane nodes (difference in normals).
     * @param information The information matrix associated to the edge (mandatory 3x3 size).
     * @return The registered edge in the graph.
     */
    g2o::EdgePlaneParallel* add_plane_parallel_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::Matrix3d& information_matrix);

    /**
     * @brief Adds an edge between two planes nodes which correspond to perpendicular planes.
     * @param v_plane1 The first plane node.
     * @param v_plane2 The second plane node.
     * @param measurement The relative position between the plane nodes (not used in the error estimation).
     * @param information The information matrix associated to the edge ().
     * @return The registered edge in the graph.
     */
    g2o::EdgePlanePerpendicular* add_plane_perpendicular_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix);

    /**
     * @brief Sets the kernel used for optimisation on an edge of the graph.
     * @param edge The edge of the graph.
     * @param kernel_type String describing the kernel type.
     * @param kernel_size The size of the kernel, also known as delta.
     */
    void add_robust_kernel(g2o::HyperGraph::Edge* edge, const std::string& kernel_type, double kernel_size);

    /**
     * @brief Optimises the graph.
     * @param num_iterations Number of max iterations used for the optimisation.
     * @return The number of iterations actually used, -1 if the optimisation cannot be done or fails.
     */
    int optimize(int num_iterations);

    /**
     * @brief Saves the pose graph to a file.
     * @param filename The name of the file where the pose graph should be saved to.
     */
    void save(const std::string& filename);

    /**
     * @brief Loads the pose graph from a file.
     * @param filename The name of the file where the pose graph should be loaded from.
     * @return True if the graph has been loaded correctly, false otherwise.
     */
    bool load(const std::string& filename);

public:
    g2o::RobustKernelFactory* robust_kernel_factory;    /**< The kernel factory used to build the kernels for multiple edges */
    std::unique_ptr<g2o::HyperGraph> graph;             /**< The g2o graph to optimize */
};


#endif //ARTSLAM_GRAPH_HANDLER_H
