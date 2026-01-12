#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <vector>
#include <limits>
#include <tuple> // 添加tuple头文件
constexpr double Ts = 0.1; // 采样时间
constexpr int Np = 20; // 预测范围
class MPCModel {
    public:
    MPCModel();
    void InputDesire_state(std::vector<double> & state);
    ~MPCModel();
    void UpdateModel();
    void ComPuteMPC_Constraints();
    std::tuple<Eigen::SparseMatrix<double>, Eigen::VectorXd,Eigen::MatrixXd,Eigen::VectorXd,Eigen::VectorXd> 
    get_MPC_Constraints();
    // 系统参数
    private:
    // 状态矩阵
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd Ca;
    Eigen::MatrixXd Qa;
    Eigen::MatrixXd Aa;
    Eigen::MatrixXd Ba;
    Eigen::MatrixXd state_x;

    // MPC滚动矩阵
    Eigen::MatrixXd PSai;
    Eigen::MatrixXd Omega;
    Eigen::MatrixXd Phi;
    Eigen::MatrixXd Gamma;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd H_;
    // osqp约束
    Eigen::SparseMatrix<double> P_sparse;
    Eigen::VectorXd q_extended;
    Eigen::MatrixXd A_constraint;
    Eigen::VectorXd  u_constraint;
    Eigen::VectorXd  l_constraint;

    // MPC约束矩阵
    Eigen::MatrixXd Mju;
    Eigen::MatrixXd Mju_hat;

    Eigen::MatrixXd Mju_hat_hat;
    Eigen::MatrixXd Epsilon;
    Eigen::MatrixXd Epsilon_hat;
    Eigen::MatrixXd Beta;
    Eigen::MatrixXd Beta_Np;
    Eigen::MatrixXd Beta_hat;
    // 约束条件
    double u_min = -2 ,u_max = 2;
    double pos_max = std::numeric_limits<double>::infinity(),pos_min = -std::numeric_limits<double>::infinity();
    double vel_min = -5,vel_max = 5;
    int n;     // Aa.rows()
    int p;     // Ba.cols()
};