#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include "stdio.h"
#include "adc.h"
#include "fast_math.h"


#define Udc 13.0f
#define ADC_REF_V                   (float)(1.65f)
#define SAMPLE_RES                  (double)(0.001)
#define AMP_GAIN                    (double)(30.0)
#define SAMPLE_CURR_CON_FACTOR      (double)(ADC_REF_V/2047.5/AMP_GAIN/SAMPLE_RES)

#define _CPR	16384           
#define ENCODER_CPR			 16384
#define ENCODER_CPR_F		(16384.0f)
#define ENCODER_CPR_DIV	(ENCODER_CPR>>1)
#define DT (double)1/PWM_FREQ
#define START_V 					3.5f
#define ABS(x) 					( (x)>0?(x):-(x) )
typedef enum {false = 0 ,true = 1} bool;


typedef enum Calib_State
{
	CAL_NULL = 0,
	CAL_ENCODER_START,	
	CAL_ENCODER_ZERO,
	CAL_ENCODER_CW_LOOP,
	CAL_ENCODER_CCW_LOOP,
	CAL_ENCODER_END,
}CalibState;


typedef struct
{
	int n, n2;
	int loop_count;
	int sample_count;
	float delta, theta_ref, theta_actual, v_cal;
	float offset_temp;
}argCALI;

typedef struct
{	
	//number of pole-pairs
	u8	polePair;
	float vbus;
	//current
	float ia;
	float ib;
	float ic;
	
	//angle 
	float mechOffset, elecOffset;
	float mechAngle, elecAngle, oldMechAngle, position;
	float rotations;
	//speed
	float mechVelocity, mechVelocityFilt, kf_vel;
	float rotate_speed;
	
	//E_ANGLE_COS_SIN
	float Cos;
	float Sin;
	
	//ADC offset
	uint32_t PhaseA_OffSet;
	uint32_t PhaseB_OffSet;
	uint32_t PhaseC_OffSet;
	
	float Res;
	float inductor;
	
}MOTOR;

typedef struct sEncoder {
	int raw;
	int count_in_cpr;
	int count_in_cpr_prev;
	
	int64_t shadow_count;
	
	// pll use
	float pos_cpr_counts;
	float vel_estimate_counts;
	
	float pos;
	float vel;
	
	float phase;
	float phase_vel;
	
	float pll_kp;
	float pll_ki;
	float interpolation;
	float snap_threshold;
} tEncoder;


typedef struct {
  double x;    // 状态向量元素1：速度
  double dx;   // 状态向量元素2：加速度
} state;

typedef struct {
  double z;    // 观测向量元素：速度计算结果
} observation;

// 定义卡尔曼滤波器结构体
typedef struct {
  state x;             // 当前状态向量
  double P[2][2];      // 当前状态协方差矩阵
  double Q[2][2];      // 过程噪声协方差矩阵
  double R;            // 观测噪声方差
  double K[2];         // Kalman增益
} kalman_filter;

typedef struct
{
	float Ta,Tb,Tc;
	float Tx,Ty;
	float t0,t1,t2,t3,t4,t5,t6,t7;
	float ts;
	float K;
	float u1,u2,u3;
	
	u8 sector;
	
}SVPWM;

typedef struct
{
	float Id;
	float Iq;
}CURRENT_DQ;	
	
typedef struct
{
	float Valpha;
	float Vbeta;
}VOLTAGE_ALPHA_BETA;
	
typedef struct
{
	float Vd;
	float Vq;
}VOLTAGE_DQ;

typedef struct
{
  float Ialpha;
  float Ibeta;
}CURRENT_ALPHA_BETA;

typedef struct
{
	float kp;
	float ki;
	float kd;
	
	float I_sum;
	float pre;
	float target;
	float bias;
	float lastBias;
	float error;
	float I_max;
	float outcome;
	float outMax;
}PID_Struct;


typedef struct
{	
	float rate;
	float pre;
	float now;
}LPF;


void board_config(void);
void Clarke_Transf(MOTOR* motor_temp,CURRENT_ALPHA_BETA* Current_alpha_beta_temp);
void Park_Transf(CURRENT_ALPHA_BETA current_alpha_beta_temp,MOTOR motor_temp,CURRENT_DQ* current_dq_temp);
void Rev_Park_Transf(VOLTAGE_DQ v_dq_temp,MOTOR* motor_temp,VOLTAGE_ALPHA_BETA* v_alpha_beta_temp);
void SVPWM_Calc(SVPWM* svpwm_temp,VOLTAGE_ALPHA_BETA v_alpha_beta_temp,float Udc_temp,float Tpwm_temp);
void Angle_To_Cos_Sin(MOTOR* motor_temp);
void FOC_Control_Loop(void);
void Start_Up(void);
void PID_Init(void);
void MOTOR_Init(void);
void Encoder_Init(void);
void Posi_Pid_Calc(PID_Struct* posi_pid_temp);
void PID_Calc(PID_Struct* pid_temp);
void ENCODER_sample(void);
float Low_Pass_Filter(LPF* lowPass_f,float value);
float Get_DCBUS(void);
float Get_angle(void);
void Zero_Current(void);
void kalman_filter_init(kalman_filter* kf, double R);
void kalman_filter_update(kalman_filter* kf, observation z, double dt);

void Disable_Motor(void);
void SYNCHRONIZE_PM(void);

void CALIBRATION_LOOP(void);

#endif