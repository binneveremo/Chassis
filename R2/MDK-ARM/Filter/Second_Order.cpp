//#include <Eigen/Dense>
//#include <iostream>
#include "Location.h"
#ifdef __cplusplus
extern "C" {
#endif
//struct Kal_t{
//     //k-1时刻的状态
//     Eigen::Vector2f xk_last;
//     //上一次的协方差
//     Eigen::Matrix2f P_last;
//     //先验
//     Eigen::Vector2f xk_hat;
//     //
//     Eigen::Matrix2f P_hat;
//     //状态空间方程里面的A矩阵与B矩阵
//     Eigen::Matrix2f A;
//     Eigen::Vector2f B;
//     //过程噪声 也就是根据状态方程计算下一时刻状态所产生的噪声
//     Eigen::Matrix2f Q;
//     //观测噪声 是将真正的传感器的值转化后的矩阵
//     Eigen::Matrix2f R;
//     //
//     Eigen::Vector2f Z;
//     Eigen::Matrix2f gain;
//};
//struct Kal_t kal_test;
//void KalamnEigen_Init() {
//    // 初始化模型参数（示例：匀速运动）
//    float dt = 0.1;
//    kal_test.A << 1, dt,
//                  0, 1;  // 状态转移
//    kal_test.Q << 0.01, 0, 
//                  0, 0.01; // 过程噪声
//    kal_test.R << 0.01, 0, 
//                  0, 0.01;         // 观测噪声方差
//    kal_test.P_last = Eigen::Matrix2f::Identity(); // 初始协方差
//}


void Eigen_Test(void)
{
     // 观测值赋值处
//     kal_test.Z << static_cast<float>(site.now.x), 
//                   static_cast<float>(site.field.vx_enc);
//     //根据状态转移矩阵 计算状态先验
//     kal_test.xk_hat = kal_test.A * kal_test.xk_last;
//     //计算协方差先验
//     kal_test.P_hat = kal_test.A * kal_test.P_last * kal_test.A.transpose() + kal_test.Q;
//     //计算卡尔曼增益
//     kal_test.gain = kal_test.P_hat * (kal_test.P_hat + kal_test.R).inverse();
//     //状态更新
//     kal_test.xk_last = kal_test.xk_hat + kal_test.gain * (kal_test.Z - kal_test.xk_hat);
//     //协方差更新
//     kal_test.P_last = (Eigen::Matrix2f::Identity() - kal_test.gain) * kal_test.P_hat;
}

#ifdef __cplusplus
}
#endif




