#include "osqp_interface.h"
#include <iostream>

using namespace std;
using namespace Eigen;

namespace osqp
{
    OSQPInterface::OSQPInterface()
    {
        settings = new OSQPSettings;
        data = new OSQPData;
    }
    OSQPInterface::~OSQPInterface()
    {
        delete settings;
        delete data;
        if (work)
            osqp_cleanup(work); // 确保只清理一次
    }
    int OSQPInterface::updateMatrices(
        Eigen::SparseMatrix<double> &P_,
        Eigen::VectorXd &q_,
        Eigen::SparseMatrix<double> &A_,
        Eigen::VectorXd &low_,
        Eigen::VectorXd &upp_,
        int warmStart)
    {
        // change P_ upper triangular
        // P: 用于计算变量的二次形式 (1/2) x' P x
        P_ = P_.triangularView<Upper>(); // 上三角矩阵
        // compress the matrices for osqp data
        P_.makeCompressed();
        // q: 这是一个向量，对应于线性项系数，在目标函数中与变量的一次项相关联，即 q' x。这里的 ' 表示转置操作。

        // A: 用于表达形如 A x 的约束。
        A_.makeCompressed();
        // low_:  l <= A x。对于没有下界的约束，可以设置为 -INFINITY
        // upp_:  A x <= u。对于没有上界的约束，可以设置为 INFINITY

        data->n = P_.rows();
        data->m = A_.rows();
        // use the eigen sparse ptrs make it faster
        data->P = csc_matrix(data->n, data->n, P_.nonZeros(),
                             P_.valuePtr(), P_.innerIndexPtr(), P_.outerIndexPtr());

        data->q = q_.data();
        data->A = csc_matrix(data->m, data->n, A_.nonZeros(),
                             A_.valuePtr(), A_.innerIndexPtr(), A_.outerIndexPtr());

        data->l = low_.data();
        data->u = upp_.data();
        if (settings)
            osqp_set_default_settings(settings);
        settings->warm_start = warmStart;
        exitflag = osqp_setup(&work, data, settings);
        // delete data->q;
        // delete data->l;
        // delete data->u;
        return 0;
    }
    int OSQPInterface::solveQP()
    {
        // Solve Problem
        return osqp_solve(work);
    }
    int OSQPInterface::solveStatus()
    {
        return work->info->status_val;
    }
    OSQPSolution *OSQPInterface::solPtr()
    {
        return work->solution;
    }
} // namespace osqp