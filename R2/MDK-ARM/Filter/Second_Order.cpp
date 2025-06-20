#include <Eigen/Dense>
#include <iostream>

#ifdef __cplusplus
extern "C"
{
#endif
#if MPC

struct Kalman3D
{
    Eigen::Vector3f xk_last; // k-1时刻状态 [位置, 速度, 加速度]
    Eigen::Matrix3f P_last;  // 上一时刻协方差
    Eigen::Vector3f xk_hat;  // 先验估计
    Eigen::Matrix3f P_hat;   // 先验协方差
    Eigen::Matrix3f A;       // 状态转移矩阵
    Eigen::Matrix3f Q;       // 过程噪声
    Eigen::Matrix3f R;       // 观测噪声
    Eigen::Vector3f Z;       // 观测向量
    Eigen::Matrix3f K;       // 卡尔曼增益
};
__attribute__((section(".sram_data"))) struct Kalman3D kalman_3d;
void Kalman3D_Init(void)
{
    float dt = 2;
    // 状态转移矩阵 (匀加速模型)
    kalman_3d.A << 1, dt, 0.5f * dt * dt,
        0, 1, dt,
        0, 0, 1;
    // 初始状态设为0
    kalman_3d.xk_last = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    // 初始协方差
    kalman_3d.P_last = Eigen::Matrix3f::Identity() * 0.1f;
    // 过程噪声 (根据系统特性调整)
    kalman_3d.Q << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.1;
    // 观测噪声 (根据传感器精度调整)
    kalman_3d.R << 0.15, 0.1, 0, // 位置观测噪声
        0.1, 0.2, 0,             // 速度观测噪声
        0, 0, 0.5;               // 加速度观测噪声
}

void Kalman3D_Update(float position, float velocity, float acceleration)
{
    // 设置观测值
    kalman_3d.Z << position, velocity, acceleration;
    // ===== 预测阶段 =====
    // 状态预测: x̂ₖ = A * xₖ₋₁
    kalman_3d.xk_hat = kalman_3d.A * kalman_3d.xk_last;
    // 协方差预测: P̂ₖ = A * Pₖ₋₁ * Aᵀ + Q
    kalman_3d.P_hat = kalman_3d.A * kalman_3d.P_last * kalman_3d.A.transpose() + kalman_3d.Q;
    // 确保协方差矩阵对称 (数值稳定性)
    kalman_3d.P_hat = 0.5f * (kalman_3d.P_hat + kalman_3d.P_hat.transpose());
    // 计算卡尔曼增益: K = P̂ₖ * (P̂ₖ + R)⁻¹
    // 使用LDLT分解提高效率和数值稳定性
    Eigen::Matrix3f S = kalman_3d.P_hat + kalman_3d.R;
    kalman_3d.K = kalman_3d.P_hat * S.ldlt().solve(Eigen::Matrix3f::Identity());
    // 状态更新: xₖ = x̂ₖ + K * (z - x̂ₖ)
    kalman_3d.xk_last = kalman_3d.xk_hat + kalman_3d.K * (kalman_3d.Z - kalman_3d.xk_hat);
    // 协方差更新: Pₖ = (I - K) * P̂ₖ
    kalman_3d.P_last = (Eigen::Matrix3f::Identity() - kalman_3d.K) * kalman_3d.P_hat;
    // 再次确保协方差对称
    kalman_3d.P_last = 0.5f * (kalman_3d.P_last + kalman_3d.P_last.transpose());
}
struct MPC_t
{
    // 转移矩阵A
    Eigen::Matrix3f A;
    // 控制矩阵
    Eigen::Vector3f B;
    //
    Eigen::Vector3f Q;
    //
    float R;

    float U;
};
struct MPC_t mpc;
void MPC_Init(void)
{
    float dt = 2;  
    float mass = 25.0f;
    
    // 修正状态转移矩阵（包含加速度影响）
    mpc.A << 1, dt, 0.5*dt*dt,  // 位置: p_{k+1} = p_k + v_k*dt + 0.5*a_k*dt^2
                    0, 1, dt,         // 速度: v_{k+1} = v_k + a_k*dt
                    0, 0, 1;          // 加速度: a_{k+1} = a_k（假设无外部干扰）
    
    // 控制输入对状态的影响（力 -> 加速度变化）
    mpc.B << 0, 0, dt/mass;  // 控制输入（力）影响加速度
    
    // 权重初始化
    mpc.Q << 1.0, 0.1, 0.01;  // 位置权重 > 速度权重 > 加速度权重
    mpc.R = 0.001;           // 控制输入权重
    mpc.U = 10.0f;            // 控制输入限制（如最大力/电流）
}
float SolveMPC(Eigen::Vector3f current_state, Eigen::Vector3f target_state)
{
		const int state_dim = 3;   // 状态维度 [位置, 速度, 加速度]
		const int control_dim = 1; // 控制输入维度
		const int horizon = 10;    // 预测步长（适当增加）

		// ==================== 1. 构建预测矩阵 ====================
		// 预测方程: X = M * x0 + C * U
		Eigen::MatrixXf state_transition = Eigen::MatrixXf::Zero(state_dim * horizon, state_dim);
		Eigen::MatrixXf control_influence = Eigen::MatrixXf::Zero(state_dim * horizon, control_dim * horizon);

		// 初始状态转移
		state_transition.block(0, 0, state_dim, state_dim) = mpc.A;
		control_influence.block(0, 0, state_dim, control_dim) = mpc.B;

		// 递归构建预测矩阵
		for (int step = 1; step < horizon; step++)
		{
				// 状态转移
				state_transition.block(step * state_dim, 0, state_dim, state_dim) =
						mpc.A * state_transition.block((step - 1) * state_dim, 0, state_dim, state_dim);

				// 历史控制输入的影响
				control_influence.block(step * state_dim, 0, state_dim, step * control_dim) =
						mpc.A * control_influence.block((step - 1) * state_dim, 0, state_dim, step * control_dim);

				// 当前控制输入的影响
				control_influence.block(step * state_dim, step * control_dim, state_dim, control_dim) = mpc.B;
		}

		// ==================== 2. 构建QP问题 ====================
		// Hessian矩阵: H = CᵀQ̄C + R̄
		Eigen::MatrixXf H = Eigen::MatrixXf::Zero(control_dim * horizon, control_dim * horizon);
		// 梯度向量: f = CᵀQ̄(Mx0 - X_ref)
		Eigen::VectorXf f = Eigen::VectorXf::Zero(control_dim * horizon);

		// 构建块对角权重矩阵
		Eigen::MatrixXf Q_full = Eigen::MatrixXf::Zero(state_dim * horizon, state_dim * horizon);
		Eigen::MatrixXf R_full = Eigen::MatrixXf::Zero(control_dim * horizon, control_dim * horizon);

		for (int step = 0; step < horizon; step++)
		{
				// 添加状态权重
				Q_full.block(step * state_dim, step * state_dim, state_dim, state_dim) =
						mpc.Q.asDiagonal();

				// 添加控制输入权重（所有步长都加权）
				R_full(step * control_dim, step * control_dim) = mpc.R;

				// 状态误差
				Eigen::Vector3f state_error = state_transition.block(step * state_dim, 0, state_dim, state_dim) * current_state - target_state;

				// 梯度项
				f += control_influence.block(step * state_dim, 0, state_dim, control_dim * horizon).transpose() * mpc.Q.asDiagonal() * state_error;
		}

		// Hessian矩阵
		H = control_influence.transpose() * Q_full * control_influence + R_full;

		// 添加正则化确保数值稳定
		H += 1e-3 * Eigen::MatrixXf::Identity(control_dim * horizon, control_dim * horizon);

		// ==================== 3. 求解QP ====================
		Eigen::VectorXf optimal_controls = -H.ldlt().solve(f);

		// ==================== 4. 提取并限制控制输入 ====================
		double U = optimal_controls(0);

		U = (U < -mpc.U) ? -mpc.U : (U > mpc.U ? mpc.U : U);
		return U;
}
float MPC_Test(float position,float velocity,float accel){
    Eigen::Vector3f target(position, velocity, accel);
    return SolveMPC(kalman_3d.xk_last,target);
} 





#endif













#ifdef __cplusplus
}
#endif
