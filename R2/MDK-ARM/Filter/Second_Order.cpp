#include <Eigen/Dense>
#include <iostream>

#ifdef __cplusplus
extern "C" {
#endif
     float value_check;
     void Eigen_Test(void)
     {
          Eigen::Matrix2f A;
          A << 1, 2,
               3, 4;

          Eigen::Vector2f B;
          B << 5,
               6;

          Eigen::Vector2f result = A * B;

          value_check = result(0, 0);
     }

#ifdef __cplusplus
}
#endif




