#ifndef _COSTFUNCTION_H_INCLUDED
#define _COSTFUNCTION_H_INCLUDED

#include <eigen3/Eigen/Dense>

#include "ceres/ceres.h"

class LocoCostFunctor {
public:
    LocoCostFunctor(const Eigen::MatrixXd &anchors, uint8_t dim) :
        anchors_(anchors), dim_(dim) {
        dists_ = Eigen::VectorXd(anchors.rows());
    }

    template <typename T>
    bool operator()(const T* const x, T* e) const {
        e[0] = T(0);
        for (int i = 0; i < dists_.size(); ++i) {
            if (ceres::abs(T(dists_[i]) + T(1.0)) > T(std::numeric_limits<T>::epsilon())) {
                for (int j = 0; j < dim_; ++j) {
                    e[0] += (T(anchors_(i, j)) - x[j]) * (T(anchors_(i, j)) - x[j]);
                }
                e[0] -= T(dists_[i]) * T(dists_[i]);
            }
        }

        return true;
    }

    Eigen::VectorXd dists_;

private:
    const Eigen::MatrixXd anchors_;
    const uint8_t dim_;
};

static ceres::Problem problem_;
static ceres::Solver::Options options_;
static ceres::Solver::Summary summary_;
static ceres::CostFunction *cost_function_ = nullptr;
static LocoCostFunctor* loco_functor_ = nullptr;

#endif  // _COSTFUNCTION_H