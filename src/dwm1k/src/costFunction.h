#ifndef _COSTFUNCTION_H_INCLUDED
#define _COSTFUNCTION_H_INCLUDED

#include <eigen3/Eigen/Dense>

#include "ceres/ceres.h"

using namespace ceres;

class LocoCostFunctor {
public:
    LocoCostFunctor(const Eigen::MatrixXd &anchors) : anchors_(anchors) {
        dists_ = Eigen::VectorXd(anchors.rows());
    }

    template <typename T>
    bool operator()(const T* const x, T* e) const {
        T v0 =  ceres::abs(T(dists_[0]) + T(1.0)) < T(std::numeric_limits<T>::epsilon()) ? T(0.0) : 
                (T(anchors_(0, 0)) - x[0]) * (T(anchors_(0, 0)) - x[0]) + 
                (T(anchors_(0, 1)) - x[1]) * (T(anchors_(0, 1)) - x[1]) + 
                (T(anchors_(0, 2)) - x[2]) * (T(anchors_(0, 2)) - x[2]) -
                T(dists_[0]) * T(dists_[0]);

        T v1 =  ceres::abs(T(dists_[1]) + T(1.0) < T(std::numeric_limits<T>::epsilon())) ? T(0.0) : 
                (T(anchors_(1, 0)) - x[0]) * (T(anchors_(1, 0)) - x[0]) + 
                (T(anchors_(1, 1)) - x[1]) * (T(anchors_(1, 1)) - x[1]) + 
                (T(anchors_(1, 2)) - x[2]) * (T(anchors_(1, 2)) - x[2]) -
                T(dists_[1]) * T(dists_[1]);

        T v2 =  ceres::abs(T(dists_[2]) + T(1.0)) < T(std::numeric_limits<T>::epsilon()) ? T(0.0) : 
                (T(anchors_(2, 0)) - x[0]) * (T(anchors_(2, 0)) - x[0]) + 
                (T(anchors_(2, 1)) - x[1]) * (T(anchors_(2, 1)) - x[1]) + 
                (T(anchors_(2, 2)) - x[2]) * (T(anchors_(2, 2)) - x[2]) -
                T(dists_[2]) * T(dists_[2]);

        e[0] = v0 + v1 + v2;
        
        return true;
    }

    Eigen::VectorXd dists_;

private:
    const Eigen::MatrixXd anchors_;
};

static Problem problem_;
static Solver::Options options_;
static Solver::Summary summary_;
static CostFunction *cost_function_ = nullptr;
static LocoCostFunctor* loco_functor_ = nullptr;

static double pos_tag_[3] = {0.0, 0.0, 0.0};

#endif  // _COSTFUNCTION_H