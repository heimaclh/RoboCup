#include <osqp/osqp.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <vector>
#include "osqp_interface.h"
#include <limits>
#include "mpc_model.h"

int main() 
{
    // Define the MPC model
    //MPCModel mpc_model;
    //mpc_model.ComPuteMPC_Constraints();
    // 依次为二次型、线性项、约束矩阵、下界、上界
    //auto [P_sparse, q_extended, A_sparse, l_constraint, u_constraint] = mpc_model.get_MPC_Constraints();
    // 使用OSQP解决问题
    osqp::OSQPInterface osqp;
    // 依次为二次型、线性项、约束矩阵、下界、上界
    // osqp.updateMatrices(P_sparse,q_extended,P_sparse,l_constraint,u_constraint);
    // osqp.solveQP();
    // auto solPtr = osqp.solPtr();
    // std::cout << "u[0] = " << solPtr->x[0] << std::endl;
    

    return 0;
}

