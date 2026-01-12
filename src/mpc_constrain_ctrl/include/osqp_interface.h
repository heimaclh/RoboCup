#ifndef PROJECT_OSQP_INTERFACE_H
#define PROJECT_OSQP_INTERFACE_H
#include <osqp/osqp.h>
#include <Eigen/Sparse>
#include <Eigen/Dense>

namespace osqp
{
    class OSQPInterface
    {
    private:
        c_int P_nnz;
        c_int A_nnz;
        // Exitflag
        c_int exitflag = 0;

        // Workspace structures
        OSQPWorkspace *work;
        OSQPSettings *settings;
        OSQPData *data;

    public:
        OSQPInterface();
        ~OSQPInterface();
        int updateMatrices(
            Eigen::SparseMatrix<double> &P_,
            Eigen::VectorXd &q_,
            Eigen::SparseMatrix<double> &A_,
            Eigen::VectorXd &low_,
            Eigen::VectorXd &upp_,
            int warmStart = WARM_START);
        int solveQP();
        int solveStatus();
        OSQPSolution *solPtr();
    };
} // namespace osqp

#endif
