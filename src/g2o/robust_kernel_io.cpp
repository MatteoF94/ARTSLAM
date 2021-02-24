#include <g2o/robust_kernel_io.h>

#include <fstream>
#include <iostream>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/sparse_optimizer.h>

namespace g2o {

    std::string kernel_type(g2o::RobustKernel* kernel) {
        if(dynamic_cast<g2o::RobustKernelHuber*>(kernel)) {
            return "Huber";
        }
        if(dynamic_cast<g2o::RobustKernelCauchy*>(kernel)) {
            return "Cauchy";
        }
        if(dynamic_cast<g2o::RobustKernelDCS*>(kernel)) {
            return "DCS";
        }
        if(dynamic_cast<g2o::RobustKernelFair*>(kernel)) {
            return "Fair";
        }
        if(dynamic_cast<g2o::RobustKernelGemanMcClure*>(kernel)) {
            return "GemanMcClure";
        }
        if(dynamic_cast<g2o::RobustKernelPseudoHuber*>(kernel)) {
            return "PseudoHuber";
        }
        if(dynamic_cast<g2o::RobustKernelSaturated*>(kernel)) {
            return "Saturated";
        }
        if(dynamic_cast<g2o::RobustKernelTukey*>(kernel)) {
            return "Tukey";
        }
        if(dynamic_cast<g2o::RobustKernelWelsch*>(kernel)) {
            return "Welsch";
        }

        return "";
    }

    bool save_robust_kernels(const std::string& filename, g2o::SparseOptimizer* graph) {
        std::ofstream ofs(filename);
        if(!ofs) {
            std::cerr << "failed to open output stream" << std::endl;
            return false;
        }

        // write kernels, but TODO wonder if different edges have different kernels...
        for(const auto& graph_edge: graph->edges()) {
            g2o::OptimizableGraph::Edge* edge = dynamic_cast<g2o::OptimizableGraph::Edge*>(graph_edge);
            g2o::RobustKernel* kernel = edge->robustKernel();
            if(!kernel) {
                continue;
            }

            std::string type = kernel_type(kernel);
            if(type.empty()) {
                std::cerr << "unknown kernel type" << std::endl;
                continue;
            }

            ofs << edge->vertices().size() << " ";
            for(auto & vertex : edge->vertices()) {
                ofs << vertex->id() << " ";
            }
            ofs << type << " " << kernel->delta() << std::endl;
        }

        return true;
    }

    class KernelData{
    public:
        KernelData(const std::string& line) {
            std::stringstream sst(line);
            size_t num_vertices;
            sst >> num_vertices;

            vertex_indices.resize(num_vertices);
            for(size_t i = 0; i < num_vertices; i++) {
                sst >> vertex_indices[i];
            }

            sst >> type >> delta;
        }

        bool match(g2o::OptimizableGraph::Edge* edge) const {
            if(edge->vertices().size() != vertex_indices.size()) {
                return false;
            }

            for(size_t i = 0; i < edge->vertices().size(); i++) {
                if(edge->vertices()[i]->id() != vertex_indices[i]) {
                    return false;
                }
            }

            return true;
        }

        g2o::RobustKernel* create() const {
            g2o::RobustKernelFactory* factory = g2o::RobustKernelFactory::instance();
            g2o::RobustKernel* kernel = factory->construct(type);
            kernel->setDelta(delta);

            return kernel;
        }

    public:
        std::vector<int> vertex_indices;
        std::string type;
        double delta;
    };

    bool load_robust_kernels(const std::string& filename, g2o::SparseOptimizer* graph) {
        std::ifstream ifs(filename);
        if(!ifs) {
            std::cerr << "failed to open input stream" << std::endl;
            return false;
        }

        std::vector<KernelData> kernels;

        while(!ifs.eof()) {
            std::string line;
            std::getline(ifs,line);
            if(line.empty()) {
                continue;
            }

            kernels.emplace_back(KernelData(line));
        }

        std::cout << "#kernels: " << kernels.size() << std::endl;

        for(auto& graph_edge: graph->edges()) {
            g2o::OptimizableGraph::Edge* edge = dynamic_cast<g2o::OptimizableGraph::Edge*>(graph_edge);

            for(auto itr = kernels.begin(); itr != kernels.end(); itr++) {
                if(itr->match(edge)) {
                    edge->setRobustKernel(itr->create());
                    kernels.erase(itr);
                    break;
                }
            }
        }

        if(!kernels.empty()) {
            std::cerr << "there are non-associated kernels" << std::endl;
        }

        return true;
    }
}
