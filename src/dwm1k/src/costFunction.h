#ifndef _COSTFUNCTION_H_INCLUDED
#define _COSTFUNCTION_H_INCLUDED

#include <eigen3/Eigen/Dense>

#include "ceres/ceres.h"

class LocoCostFunctor {
public:
    LocoCostFunctor(const Eigen::MatrixXd &anchors) : anchors_(anchors) {
        dists_ = Eigen::VectorXd(anchors.rows());
    }

    template <typename T>
    bool operator()(const T* const x, T* e) const {
        e[0] = T(0);
        for (int i = 0; i < dists_.size(); ++i) {
            e[0] += ceres::abs(T(dists_[i]) + T(1.0)) < T(std::numeric_limits<T>::epsilon()) ? T(0.0) :
                    (T(anchors_(i, 0)) - x[0]) * (T(anchors_(i, 0)) - x[0]) +
                    (T(anchors_(i, 1)) - x[1]) * (T(anchors_(i, 1)) - x[1]) +
                    (T(anchors_(i, 2)) - x[2]) * (T(anchors_(i, 2)) - x[2]) -
                    T(dists_[i]) * T(dists_[i]);;
        }

        return true;
    }

    Eigen::VectorXd dists_;

private:
    const Eigen::MatrixXd anchors_;
};

static ceres::Problem problem_;
static ceres::Solver::Options options_;
static ceres::Solver::Summary summary_;
static ceres::CostFunction *cost_function_ = nullptr;
static LocoCostFunctor* loco_functor_ = nullptr;

static double pos_tag_[3] = {0.0, 0.0, 0.0};

#endif  // _COSTFUNCTION_H