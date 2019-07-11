#ifndef _COSTFUNCTION_H_INCLUDED
#define _COSTFUNCTION_H_INCLUDED

#include <eigen3/Eigen/Dense>

#include "ceres/ceres.h"

class LocoCostFunctor {
public:
    LocoCostFunctor(const Eigen::MatrixXd &anchors, uint8_t dim) :
        anchors_(anchors), dim_(dim), num_anchor_(anchors.rows()) { }

    template <typename T>
    bool operator()(const T* x, const T* y, T* e) const {
        e[0] = T(0);
        for (int i = 0; i < num_anchor_; ++i) {
            if (ceres::abs(y[i] + T(1.0)) > T(std::numeric_limits<T>::epsilon())) {
                for (int j = 0; j < dim_; ++j) {
                    e[0] += (T(anchors_(i, j)) - x[j]) * (T(anchors_(i, j)) - x[j]);
                }
                e[0] -= y[i] * y[i];
            }
        }

        return true;
    }

private:
    const Eigen::MatrixXd anchors_;
    const uint8_t dim_;
    const uint8_t num_anchor_;
};

static ceres::Problem problem_;
static ceres::Solver::Options options_;
static ceres::Solver::Summary summary_;
static ceres::CostFunction *cost_function_ = nullptr;
static LocoCostFunctor* loco_functor_ = nullptr;

#endif  // _COSTFUNCTION_H