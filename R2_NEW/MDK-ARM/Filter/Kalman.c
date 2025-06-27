#include "Kalman.h"
char SecondOrderMatrix_Operate(char ra,char ca, char rb,char cb,char rc,char cc,float a[ra][ca],float b[rb][cb],float c[rc][cc],char * operate){
#define atype (ra * 10 + ca)
#define btype (rb * 10 + cb)
#define ctype (rc * 10 + cc)
	if(strcmp(operate,"Add") == 0)  goto Add;
	if(strcmp(operate,"Sub") == 0)  goto Substract;
	if(atype == 22 && btype == 22 && ctype == 22){
		c[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0];
		c[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1];
		c[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0];
		c[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1];}
	else if(atype == 22 && btype == 21 && ctype == 21){
		c[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0];
		c[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0];}
	else if(atype == 21 && btype == 11 && ctype == 21){
		c[0][0] = a[0][0] * b[0][0];
		c[1][0] = a[1][0] * b[0][0];}
	return true;
Add:
	if(atype == 21 && btype == 21 && ctype == 21){
		c[0][0] = a[0][0] + b[0][0];
		c[1][0] = a[1][0] + b[1][0];}
	else if(atype == 22 && btype == 22 && ctype == 22){
		c[0][0] = a[0][0] + b[0][0];
		c[1][0] = a[1][0] + b[1][0];
		c[0][1] = a[0][1] + b[0][1];
		c[1][1] = a[1][1] + b[1][1];}
	return true;
Substract:
	if(atype == 21 && btype == 21 && ctype == 21){
		c[0][0] = a[0][0] - b[0][0];
		c[1][0] = a[1][0] - b[1][0];}
	else if(atype == 22 && btype == 22 && ctype == 22){
		c[0][0] = a[0][0] - b[0][0];
		c[0][1] = a[0][1] - b[0][1];
		c[1][0] = a[1][0] - b[1][0];
		c[1][1] = a[1][1] - b[1][1];}
	return true;
}
char Matrix_Change(float now[2][2], float tar[2][2],char * operate){
	float det = now[0][0] * now[1][1] - now[0][1] * now[1][0];
	if(strcmp(operate,"Inv") == 0)		goto Inverse;
	tar[0][0] = now[0][0]; 
	tar[0][1] = now[1][0]; 
	tar[1][0] = now[0][1]; 
	tar[1][1] = now[1][1];
	return true;
Inverse:	//计算行列式
	if (det == 0) return false;
	float inv_det = 1.0f / det;
	tar[0][0] =  now[1][1] * inv_det;
	tar[0][1] = -now[0][1] * inv_det;
	tar[1][0] = -now[1][0] * inv_det;
	tar[1][1] =  now[0][0] * inv_det;
	return true;
}
struct Kalman_System_t{
	float A[2][2];
	float B[2][1];
	float Q[2][2];
	float R[2][2];
	float K[2][2];
	struct{
		float P[2][2];
		float state[2][1];
	}Next_;
	struct {
		float P[2][2];
		float state[2][1];
	}last;
	struct {
    float state[2][1];
    float a[1][1]; 
} detect;
	float state[2][1];
};
struct Kalman_System_t sys;
void SecondOrder_KalmanSystemInit(void){
	sys.Q[0][0] = 0.1;
	sys.Q[1][1] = 0.1;
	
	sys.R[0][0] = 0.1;
	sys.R[1][1] = 0.1;
	
	sys.A[0][0] = 1;
	sys.A[0][1] = 2;
	sys.A[1][1] = 1;
	
	sys.B[0][0] = 2;
	sys.B[1][0] = 2;
}


void Kalman_Test(void){
	sys.detect.state[0][0] = site.field.x_enc;
	sys.detect.state[1][0] = site.field.vx_enc;
	sys.detect.a[0][0]     = site.field.ax_gyro;
	Position_Velocity_Accel_SecondOrderKalmanCal();
}


void Position_Velocity_Accel_SecondOrderKalmanCal(void){
	float tempa[2][1],tempb[2][1];
	//x(k=1)~ = A * xk + B * a
	SecondOrderMatrix_Operate(2,2,2,1,2,1,sys.A,sys.last.state,tempa,"Mul");
	SecondOrderMatrix_Operate(2,1,1,1,2,1,sys.A,sys.detect.a,tempb,"Mul");
	SecondOrderMatrix_Operate(2,1,2,1,2,1,tempa,tempb,sys.Next_.state,"Add");
	//中间变量释放
	//p(k+1)~ = A*p(k)*A(T) + Q
	float tempc[2][2],tempd[2][2],tempe[2][2];
	SecondOrderMatrix_Operate(2,2,2,2,2,2,sys.A,sys.last.P,tempc,"Mul");
	Matrix_Change(sys.A, tempe,"Tra");
	SecondOrderMatrix_Operate(2,2,2,2,2,2,tempc,tempe,tempd,"Mul");
	SecondOrderMatrix_Operate(2,2,2,2,2,2,tempd,sys.Q,sys.Next_.P,"Add");
	//中间变量释放
	//卡尔曼增益k = p(k+1)~ * (p(k+1)~ + R)(-1)
	SecondOrderMatrix_Operate(2,2,2,2,2,2,sys.Next_.P,sys.R,tempc,"Add");
	Matrix_Change(tempc, tempd,"Inv");
	SecondOrderMatrix_Operate(2,2,2,2,2,2,sys.Next_.P,tempd,sys.K,"Mul");
	//中间变量释放
	//更新 x(k+1) = x(k)~ + k(z - x(x)~)
	SecondOrderMatrix_Operate(2,1,2,1,2,1,sys.detect.state,sys.Next_.state,tempa,"Sub");
	SecondOrderMatrix_Operate(2,2,2,1,2,1,sys.K,tempa,tempb,"Mul");
	SecondOrderMatrix_Operate(2,2,2,1,2,1,sys.Next_.state,tempb,sys.state,"Add");
	//中间变量释放
	//更新协方差 P(k+1) = (1 - k) * p(k+1)~
	float I[2][2] = {{1,0},{0,1}};
	SecondOrderMatrix_Operate(2,2,2,2,2,2,I,sys.K,tempc,"Sub");
	SecondOrderMatrix_Operate(2,2,2,2,2,2,tempc,sys.Next_.P,sys.last.P,"Sub");
	
}
	

float EKF_Filter(struct EKF * ekf, float input ,float gain){
	ekf->a_hat_prior = ekf->a_hat + gain;
	if (isnan(ekf->a_hat_prior))       ekf->a_hat_prior = 0.0; 
	ekf->p_prior = ekf->p + ekf->q;
	ekf->y = input - ekf->a_hat_prior;
	ekf->k = ekf->p_prior / (ekf->p_prior + ekf->r);
	ekf->a_hat = ekf->a_hat_prior + ekf->k * ekf->y;
	ekf->p = (1 - ekf->k)* ekf->p_prior;
	return ekf->a_hat;
}



















