#include "mpc_model.h"

// 修正函数名拼写错误
Eigen::MatrixXd matrix_pow(const Eigen::MatrixXd &A, int n)
{
    if (n == 0) {
        return Eigen::MatrixXd::Identity(A.rows(), A.cols());
    }
    Eigen::MatrixXd result = A;
    for(int i = 1; i < n; i++)
    {
        result = result * A;
    }
    return result;
}

MPCModel::MPCModel() {
    // 系统参数
    constexpr double Ts = 0.1; // 采样时间
    A.resize(2, 2);
    B.resize(2, 1);
    Q.resize(2, 2);
    R.resize(1, 1);
    Ca.resize(2, 4);
    Qa.resize(2, 2);
    Aa.resize(4, 4);
    Ba.resize(4, 1);
    state_x.resize(4, 1);

    A << 1, Ts,
         0, 1;
    B << 0.5*Ts*Ts,
         1*Ts;
    Q << 1, 0,
         0, 1;
    R << 1;
    n = A.rows();     // Aa.rows()
    p = B.cols();     // Ba.cols()
    // 轨迹跟踪矩阵
    Ca.setZero(n, 2*n);
    Aa.setZero(2*n, 2*n);
    Ca.block(0,0,n,n) = -Eigen::MatrixXd::Identity(n,n);
    Ca.block(0,n,n,n) = Eigen::MatrixXd::Identity(n,n);
    Aa.block(0,0,n,n) = A;
    Aa.block(n,n,n,n) = Eigen::MatrixXd::Identity(n,n);
    Ba.setZero(2*n,p);
    Ba.block(0,0,n,p) = B;
    n = Aa.rows();
    // 状态变量
    state_x.setZero(n,1);
    state_x<< 0, 
              0,
              1,
              0; // 设定初始状态
    Qa = Ca.transpose() * Q * Ca;

    // 滚动矩阵
    PSai.resize(p * Np,p * Np);
    Omega.resize(n * Np,n * Np);
    Phi.resize(n * Np,n);
    Gamma.resize(n * Np , p * Np);
    PSai.setZero(p * Np,p * Np);
    for(int i = 0; i < Np; i++)
        PSai.block(i*p,i*p,p,p) = R;
    Omega.setZero(n * Np,n * Np);
    for(int i = 0; i < Np; i++)
        Omega.block(i*n,i*n,n,n) = Qa;
    Phi.setZero(n * Np,n);
    for(int i = 0; i < Np; i++)
        Phi.block(i*n,0,n,n) = matrix_pow(Aa,i+1);

    Gamma.setZero(n * Np , p * Np);

    for(int i = 0; i < Np; i++){
        Gamma.block(i*n,i*p,n,p) = Ba;
        for(int j = i + 1; j < Np; j++)
            Gamma.block(j*n,i*p,n,p) = matrix_pow(Aa,j-i) * Ba;
    }
    F_ = Gamma.transpose() * Omega * Phi;
    H_ = PSai + Gamma.transpose() * Omega * Gamma;
    // MPC约束矩阵
    Mju.resize(2*n+2*p,n);
    Mju_hat.resize((2*n+2*p)*Np+2*n,n);
    Mju_hat_hat.resize(Np*(2*n+2*p) + 2*n,n*Np);
    Epsilon.resize(2*n+2*p,p);
    Epsilon_hat.resize((2*n+2*p)*Np + 2*n,p*Np);
    Beta.resize(2*n+2*p,1);
    Beta_Np.resize(2*n,1);
    Beta_hat.resize((2*n+2*p)*Np + 2*n,1);

    Mju.block(2*p,0,n,n) = -Eigen::MatrixXd::Identity(n,n);
    Mju.block(2*p+n,0,n,n) = Eigen::MatrixXd::Identity(n,n);
    Mju_hat.block(0,0,2*n+2*p,n) = Mju;
    
    for(int i = 0; i < Np -1; i++)
        Mju_hat_hat.block(2*(p+n)*(i+1),i*n,2*n+2*p,n) = Mju;

    Mju_hat_hat.block(2*(p+n)*Np,(Np-1)*n,n,n) = -Eigen::MatrixXd::Identity(n,n);
    Mju_hat_hat.block(2*(p+n)*Np + n,(Np-1)*n,n,n) = Eigen::MatrixXd::Identity(n,n);
    Epsilon.block(0,0,p,p) = -Eigen::MatrixXd::Identity(p,p);
    Epsilon.block(p,0,p,p) = Eigen::MatrixXd::Identity(p,p);

    for(int i = 0; i < Np; i++)
        Epsilon_hat.block(i*2*n+i*2*p,i*p,2*n+2*p,p) = Epsilon;
    // std::cout << "Epsilon_hat:\n" << Epsilon_hat << std::endl;
    Beta(0,0) = -u_min;
    Beta(1,0) = u_max;
    Beta(2,0) = -pos_min;
    Beta(3,0) = -vel_min;
    Beta(4,0) = -pos_min;
    Beta(5,0) = -vel_min;
    Beta(6,0) = pos_max;
    Beta(7,0) = vel_max;
    Beta(8,0) = pos_max;
    Beta(9,0) = vel_max;
    Beta_Np(0,0) = -pos_min;
    Beta_Np(1,0) = -vel_min;
    Beta_Np(2,0) = -pos_min;
    Beta_Np(3,0) = -vel_min;
    Beta_Np(4,0) = pos_max;
    Beta_Np(5,0) = vel_max;
    Beta_Np(6,0) = pos_max;
    Beta_Np(7,0) = vel_max;
    Beta_hat.block(0,0,2*n+2*p,1) = Beta;
    for(int i = 1; i < Np; i++)
        Beta_hat.block(i*(2*n+2*p),0,2*n+2*p,1) = Beta;
    Beta_hat.block(Np*(2*n+2*p),0,2*n,1) = Beta_Np;
    // osqp约束
    P_sparse = H_.sparseView();
    q_extended = F_*state_x;
}

void MPCModel::InputDesire_state(std::vector<double> & state)
{
    state_x(2,0) = state[0];
    state_x(3,0) = state[1];
}
void MPCModel::UpdateModel()
{
    Beta(0,0) = -u_min;
    Beta(1,0) = u_max;
    Beta(2,0) = -pos_min;
    Beta(3,0) = -vel_min;
    Beta(4,0) = -pos_min;
    Beta(5,0) = -vel_min;
    Beta(6,0) = pos_max;
    Beta(7,0) = vel_max;
    Beta(8,0) = pos_max;
    Beta(9,0) = vel_max;
    Beta_Np.setZero(2*n,1);
    Beta_Np(0,0) = -pos_min;
    Beta_Np(1,0) = -vel_min;
    Beta_Np(2,0) = -pos_min;
    Beta_Np(3,0) = -vel_min;
    Beta_Np(4,0) = pos_max;
    Beta_Np(5,0) = vel_max;
    Beta_Np(6,0) = pos_max;
    Beta_Np(7,0) = vel_max;   
    for(int i = 0; i < Np; i++)
        Beta_hat.block(i*(2*n+2*p),0,2*n+2*p,1) = Beta;
    Beta_hat.block(Np*(2*n+2*p),0,2*n,1) = Beta_Np;
     
}
void MPCModel::ComPuteMPC_Constraints()
{
    A_constraint = Mju_hat_hat * Gamma + Epsilon_hat;
    u_constraint = Beta_hat + -(Mju_hat + Mju_hat_hat * Phi) * state_x;
    l_constraint = Eigen::VectorXd::Zero(A_constraint.rows());
    for(int i = 0; i < l_constraint.rows(); i++)
        l_constraint(i) = -std::numeric_limits<double>::infinity();
    q_extended = F_*state_x;
}

std::tuple<Eigen::SparseMatrix<double>, Eigen::VectorXd,Eigen::MatrixXd,Eigen::VectorXd,Eigen::VectorXd> 
MPCModel::get_MPC_Constraints()
{
    return std::make_tuple(P_sparse, q_extended, A_constraint, u_constraint, l_constraint);
}
MPCModel::~MPCModel() {

}
