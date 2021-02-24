//
// Created by matteo on 18/11/20.
//

#ifndef ARTSLAM_ROBUST_KERNEL_IO_H
#define ARTSLAM_ROBUST_KERNEL_IO_H


#include <string>
#include <g2o/core/sparse_optimizer.h>

namespace g2o {

    std::string kernel_type(g2o::RobustKernel* kernel);

    bool save_robust_kernels(const std::string& filename, g2o::SparseOptimizer* graph);

    bool load_robust_kernels(const std::string& filename, g2o::SparseOptimizer* graph);

} // namespace g2o

#endif //ARTSLAM_ROBUST_KERNEL_IO_H
