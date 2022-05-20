#ifndef ARTSLAM_LASER_3D_GRAPH_HANDLER_H
#define ARTSLAM_LASER_3D_GRAPH_HANDLER_H

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
}  // namespace g2o

namespace artslam::laser3d {
    class GraphHandler {
    public:
        GraphHandler(const std::string& solver_type = "lm_var");
        virtual ~GraphHandler();

        int num_vertices() const;
        int num_edges() const;

        void set_solver(const std::string& solver_type) const;


        g2o::VertexSE3* add_se3_node(const Eigen::Isometry3d& pose) const;

        g2o::VertexPlane* add_plane_node(const Eigen::Vector4d& plane_coeffs) const;

        g2o::VertexPointXYZ* add_point_xyz_node(const Eigen::Vector3d& xyz) const;

        g2o::EdgeSE3* add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix) const;

        g2o::EdgeSE3Plane* add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix) const;

        g2o::EdgeSE3PointXYZ* add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) const;

        g2o::EdgePlanePriorNormal* add_plane_normal_prior_edge(g2o::VertexPlane* v, const Eigen::Vector3d& normal, const Eigen::MatrixXd& information_matrix) const;

        g2o::EdgePlanePriorDistance* add_plane_distance_prior_edge(g2o::VertexPlane* v, double distance, const Eigen::MatrixXd& information_matrix) const;

        g2o::EdgeSE3PriorXY* add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix) const;

        g2o::EdgeSE3PriorXYZ* add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix) const;

        g2o::EdgeSE3PriorQuat* add_se3_prior_quat_edge(g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat, const Eigen::MatrixXd& information_matrix) const;

        g2o::EdgeSE3PriorVec* add_se3_prior_vec_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix) const;

        g2o::EdgePlane* add_plane_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information) const;

        g2o::EdgePlaneIdentity* add_plane_identity_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector4d& measurement, const Eigen::Matrix4d& information) const;

        g2o::EdgePlaneParallel* add_plane_parallel_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::Matrix3d& information) const;

        g2o::EdgePlanePerpendicular* add_plane_perpendicular_edge(g2o::VertexPlane* v_plane1, g2o::VertexPlane* v_plane2, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information) const;

        void add_robust_kernel(g2o::HyperGraph::Edge* edge, const std::string& kernel_type, double kernel_size) const;

        int optimize(int num_iterations);

        void save(const std::string& filename) const;

        bool load(const std::string& filename) const;

        // testing purposes
        uint64_t total_time_ = 0;
        uint32_t count_ = 0;

    public:
        g2o::RobustKernelFactory* robust_kernel_factory_;
        std::unique_ptr<g2o::HyperGraph> graph_;  // g2o graph

        bool verbose_ = false;
    };
}


#endif //ARTSLAM_LASER_3D_GRAPH_HANDLER_H
