#include <Eigen/Cholesky>
#include <Eigen/Core>
#include "Global.h"


#undef Zero 

Eigen::Matrix3f inverse3x3(Eigen::Matrix3f& A);

CPP_BEGIN
using namespace Eigen;
struct Kalman3D{
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
using namespace Eigen;
__attribute__((section(".sram_data"))) struct Kalman3D kalmanx;
__attribute__((section(".sram_data"))) struct Kalman3D kalmany;

 
void Kalman_Init(void){
    float dt = 0.002;
    // 状态转移矩阵 (匀加速模型)
    kalmanx.A << 1, dt, 0.5f * dt * dt,
        0, 1, dt,
        0, 0, 1;
    // 初始状态设为0
    kalmanx.xk_last = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    // 初始协方差
    kalmanx.P_last = Eigen::Matrix3f::Identity() * 0.1f;
    // 过程噪声 (根据系统特性调整)
    kalmanx.Q << 0.1, 0, 0,
									0, 0.1, 0,
									0, 0, 0.1;
    // 观测噪声 (根据传感器精度调整)
    kalmanx.R <<  5, 0.9, 0, // 位置观测噪声
									0.9, 4, 0,             // 速度观测噪声
									0,   0, 4;               // 加速度观测噪声
    /////////////////////////////y方向kalman初始化/////////////////////
    kalmany.A << 1, dt, 0.5f * dt * dt,
									0, 1, dt,
									0, 0, 1;
    // 初始状态设为0
    kalmany.xk_last = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    // 初始协方差
    kalmany.P_last = Eigen::Matrix3f::Identity() * 0.5f;
    // 过程噪声 (根据系统特性调整)
    kalmany.Q << 0.1, 0, 0,
        0, 0.1, 0,
        0, 0, 0.1;
    // 观测噪声 (根据传感器精度调整)
    kalmany.R <<  5, 0.9, 0, // 位置观测噪声
									0.9, 4, 0,             // 速度观测噪声
									0, 0, 4;               // 加速度观测噪声
}

void KalmanX_Update(float position, float velocity, float acceleration,struct status_node_t * statusx){
    // 设置观测值
    kalmanx.Z << position / 1000, velocity, acceleration;
    // ===== 预测阶段 =====
    // 状态预测: x̂ₖ = A * xₖ₋₁
    kalmanx.xk_hat = kalmanx.A * kalmanx.xk_last;
    // 协方差预测: P̂ₖ = A * Pₖ₋₁ * Aᵀ + Q
    kalmanx.P_hat = kalmanx.A * kalmanx.P_last * kalmanx.A.transpose() + kalmanx.Q;
    // 确保协方差矩阵对称 (数值稳定性)
    kalmanx.P_hat = 0.5f * (kalmanx.P_hat + kalmanx.P_hat.transpose());
    // 计算卡尔曼增益: K = P̂ₖ * (P̂ₖ + R)⁻¹
    // 使用LDLT分解提高效率和数值稳定性
    Eigen::Matrix3f S = kalmanx.P_hat + kalmanx.R;
	  
		//kalmanx.K = S.ldlt().solve(kalmanx.P_hat);
		kalmanx.K = kalmanx.P_hat * inverse3x3(S);
    // 状态更新: xₖ = x̂ₖ + K * (z - x̂ₖ)
    kalmanx.xk_last = kalmanx.xk_hat + kalmanx.K * (kalmanx.Z - kalmanx.xk_hat);
    // 协方差更新: Pₖ = (I - K) * P̂ₖ
    kalmanx.P_last = (Eigen::Matrix3f::Identity() - kalmanx.K) * kalmanx.P_hat;
    // 再次确保协方差对称
    kalmanx.P_last = 0.5f * (kalmanx.P_last + kalmanx.P_last.transpose());

    statusx->position = kalmanx.xk_last(0) * 1000;
    statusx->velocity = kalmanx.xk_last(1);
    statusx->accel    = kalmanx.xk_last(2);
}
void KalmanY_Update(float position, float velocity, float acceleration,struct status_node_t * statusy){
    // 设置观测值
    kalmany.Z << position / 1000, velocity, acceleration;
    // ===== 预测阶段 =====
    // 状态预测: x̂ₖ = A * xₖ₋₁
    kalmany.xk_hat = kalmany.A * kalmany.xk_last;
    // 协方差预测: P̂ₖ = A * Pₖ₋₁ * Aᵀ + Q
    kalmany.P_hat = kalmany.A * kalmany.P_last * kalmany.A.transpose() + kalmany.Q;
    // 确保协方差矩阵对称 (数值稳定性)
    kalmany.P_hat = 0.5f * (kalmany.P_hat + kalmany.P_hat.transpose());
    // 计算卡尔曼增益: K = P̂ₖ * (P̂ₖ + R)⁻¹
    // 使用LDLT分解提高效率和数值稳定性
    Eigen::Matrix3f S = kalmany.P_hat + kalmany.R;
		kalmany.K = kalmany.P_hat * inverse3x3(S);
    // 状态更新: xₖ = x̂ₖ + K * (z - x̂ₖ)
    kalmany.xk_last = kalmany.xk_hat + kalmany.K * (kalmany.Z - kalmany.xk_hat);
    // 协方差更新: Pₖ = (I - K) * P̂ₖ
    kalmany.P_last = (Eigen::Matrix3f::Identity() - kalmany.K) * kalmany.P_hat;
    // 再次确保协方差对称
    kalmany.P_last = 0.5f * (kalmany.P_last + kalmany.P_last.transpose());

    statusy->position = kalmany.xk_last(0) * 1000;
    statusy->velocity = kalmany.xk_last(1);
    statusy->accel    = kalmany.xk_last(2);
}
void KalmanFilter_UpDate(struct Point field_position,float car_velocity_x,float car_velocity_y,float car_accel_x,float car_accel_y){





}







constexpr int N = 4;
using Matrix12x3f = Matrix<float, 12, 3>;  // 3*N x 3 = 12x3
using Matrix12x4f = Matrix<float, 12, N>;  // 3*N x N = 12x4
using Vector12f	  = Matrix<float, 12, 1>;

struct MPC_t {
    // 系统矩阵（单精度）
    Matrix3f A;
    Vector3f B;
    
    // 权重参数
    Vector3f Q;  // 状态权重向量
    float R;     // 控制权重
    
    // 约束参数
    float OutLimit;
    
    // 预测矩阵（固定大小）
    Matrix12x3f Sx;  // 状态预测矩阵 12x3
    Matrix12x4f Su;  // 控制预测矩阵 12x4
    
    // Hessian矩阵（固定大小）
    Matrix4f H;      // 4x4
    
    float mass;
    float dt;
};

struct MPC_t mpc;

void MPC_Init(void) {
    mpc.dt = 0.004f;   // 时间步长
    mpc.mass = 25;
    
    // 离散状态空间模型
    mpc.A << 1,  mpc.dt, 0.5f * mpc.dt * mpc.dt,
             0,  1,      mpc.dt,
             0,  0,      1;
    
    mpc.B << 0,  0,  mpc.dt / mpc.mass;
    
    // 权重设置
    mpc.Q << 5.0f, 0.05f, 0.1f;  // 状态权重
    mpc.R =  0.001f;               // 控制权重
    mpc.OutLimit = 2000.0f;       // 控制量限幅

    // 初始化预测矩阵（全部置零）
    mpc.Sx.setZero();
    mpc.Su.setZero();
    
    // 构建预测矩阵
    mpc.Sx.block<3, 3>(0, 0) = mpc.A;
    mpc.Su.block<3, 1>(0, 0) = mpc.B;
    
    // 递推构建预测矩阵（N=4）
    for (int i = 1; i < N; i++) {
        // 状态传递 (3x3 block)
        mpc.Sx.block<3, 3>(3*i, 0) = mpc.A * mpc.Sx.block<3, 3>(3*(i-1), 0);
        
        // 控制传递 (3x4 block)
        mpc.Su.block<3, 4>(3*i, 0) = mpc.Su.block<3, 4>(3*(i-1), 0);
        
        // 添加当前控制
        mpc.Su.block<3, 1>(3*i, i) += mpc.B;
    }
    
    // 构建Hessian矩阵（固定大小4x4）
    mpc.H.setZero();
    Matrix3f Qdiag = mpc.Q.asDiagonal();  // 向量转对角矩阵
    
    for (int i = 0; i < N; i++) {
        // 提取当前控制影响 (3x4 block)
        auto Su_i = mpc.Su.block<3, 4>(3*i, 0);
        
        // H += Su_i^T * Q * Su_i
        mpc.H += Su_i.transpose() * Qdiag * Su_i;
        
        // 对角线添加控制权重
        mpc.H(i, i) += mpc.R;
    }
}

float MPC_OutPut(Vector3f now, Vector3f target) {
    // 计算梯度向量（固定大小4x1）
    Vector4f f = Vector4f::Zero();
    Matrix3f Qdiag = mpc.Q.asDiagonal();
    
    for (int i = 0; i < N; i++) {
        // 状态预测误差
        Vector3f predicted = mpc.Sx.block<3, 3>(3*i, 0) * now;
			
				Vector3f err = predicted - target;
        
        // 梯度项 f += Su_i^T * Q * err
        f += mpc.Su.block<3, 4>(3*i, 0).transpose() * Qdiag * err;
    }
    
    // 求解线性系统 H * u = -f
    Vector4f u = mpc.H.ldlt().solve(-f);
    
    // 返回第一个控制量并限幅
    return Limit(u(0), -mpc.OutLimit, mpc.OutLimit);
}

// 辅助函数：数值安全限幅

void MPC_Calculate(struct Point last, struct Point next,float now_rad, float velocity_target,float* left, float* front) {
    // 1. 创建旋转矩阵
    Matrix2f car_convertmatrix;
    float cos_r = cosf(now_rad);
    float sin_r = sinf(now_rad);
    car_convertmatrix << cos_r, sin_r,
                        -sin_r, cos_r; 
    
    // 2. 获取当前位置（假设已定义kalmanx, kalmany）
    float nowx = kalmanx.xk_last(0);
    float nowy = kalmany.xk_last(0);
    
    // 3. 计算路径方向
    Vector2f path_dir(next.x - last.x, next.y - last.y);
    float path_len = path_dir.norm();
    Vector2f norm_dir = path_dir / (path_len > 1e-6f ? path_len : 1.0f);
    
    // 4. 计算当前状态在路径方向上的投影
    Vector2f current_pos(nowx, nowy);
    Vector2f last_pos(last.x, last.y);
    Vector2f pos_vec = current_pos - last_pos;
    
    // 投影计算
    float position_norm = pos_vec.dot(norm_dir);
    float velocity_norm = Vector2f(kalmanx.xk_last(1), kalmany.xk_last(1)).dot(norm_dir);
    float accel_norm = Vector2f(kalmanx.xk_last(2), kalmany.xk_last(2)).dot(norm_dir);
    
    // 5. 设置目标状态
    Vector3f target(path_len, velocity_target, 0);
    
    // 6. 计算总加速度
    Vector3f current_state(position_norm, velocity_norm, accel_norm);
    float Total_Velocity = MPC_OutPut(current_state, target);
    
    // 7. 确定控制方向
    float death = 200;
    Vector2f to_target(next.x - nowx, next.y - nowy);
    float dist = to_target.norm();
    Vector2f car_dir = (dist < death) ? norm_dir : to_target / dist;
    
    // 8. 创建加速度向量并转换坐标系
    Vector2f accel_vec = Total_Velocity * car_dir;
    Vector2f car_accel = car_convertmatrix * accel_vec;
    
    // 9. 输出结果
    *front = -car_accel(0);
    *left  = -car_accel(1);
}
//shear 切向 normal 法向
//假设每个舵轮角度向前为0度 向左角度增加








CPP_END
Eigen::Matrix3f inverse3x3(Eigen::Matrix3f& A){
    // 计算行列式
    float det = A(0,0) * (A(1,1)*A(2,2) - A(1,2)*A(2,1)) -
                A(0,1) * (A(1,0)*A(2,2) - A(1,2)*A(2,0)) +
                A(0,2) * (A(1,0)*A(2,1) - A(1,1)*A(2,0));
    if (det == 0.0f) 
        return Eigen::Matrix3f::Identity();
    // 计算伴随矩阵
    Eigen::Matrix3f adjugate;
    adjugate(0,0) =  (A(1,1)*A(2,2) - A(1,2)*A(2,1));
    adjugate(0,1) = -(A(0,1)*A(2,2) - A(0,2)*A(2,1));
    adjugate(0,2) =  (A(0,1)*A(1,2) - A(0,2)*A(1,1));
    adjugate(1,0) = -(A(1,0)*A(2,2) - A(1,2)*A(2,0));
    adjugate(1,1) =  (A(0,0)*A(2,2) - A(0,2)*A(2,0));
    adjugate(1,2) = -(A(0,0)*A(1,2) - A(0,2)*A(1,0));
    adjugate(2,0) =  (A(1,0)*A(2,1) - A(1,1)*A(2,0));
    adjugate(2,1) = -(A(0,0)*A(2,1) - A(0,1)*A(2,0));
    adjugate(2,2) =  (A(0,0)*A(1,1) - A(0,1)*A(1,0));

    // 返回逆矩阵：A⁻¹ = adj(A) / det(A)
    return adjugate / det;
}
