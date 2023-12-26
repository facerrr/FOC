#include "motor.h"
#include "as5047.h"
#include "report.h"
#include <math.h>

MOTOR motor;
SVPWM svpwm;

VOLTAGE_DQ Voltage_DQ;
VOLTAGE_ALPHA_BETA Voltage_Alpha_Beta;
CURRENT_ALPHA_BETA Current_Ialpha_beta;
CURRENT_DQ Current_Idq; 

//三环PID
PID_Struct Current_ID_Pid;
PID_Struct Current_IQ_Pid;
PID_Struct Speed_Pid;
PID_Struct Posi_Pid;

//速度滤波
LPF LPF_vel;

kalman_filter kf;

tEncoder Encoder;

static CalibState mCalibState = CAL_ENCODER_START;
argCALI mArgCali;

extern SystemMode mSystemMode;
extern int stateChange;
extern ControlState ctrlState;
extern starget targetPm; 
extern float pm_buf[14];
uint8_t upload_count = 0;
float value = -3;
float moni_vel;
float moni_angle;
float moni_offset = 5.7;
float target_angle = 3.14;
float target_vel = 30;

float ID_ref;
float IQ_ref;
float DC_BUS;

uint32_t hPhaseA_OffSet=1938;//2037
uint32_t hPhaseB_OffSet=1985;//2043
uint32_t hPhaseC_OffSet=2017;//2043

float error_f[1792] = {0.0f};

void HAL_SYSTICK_Callback(void){
	
	// 1ms update the speed
}

void board_config(void)
{
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,PWM_PERIOD-1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	MOTOR_Init();
	PID_Init();
	Encoder_Init();
	kalman_filter_init(&kf,4);
//	Zero_Current();
//	Calibration(&motor);
//	Start_Up();
}

void PID_Init(void){
	//PID参数初始化
	Speed_Pid.kp = 0.5f;
	Speed_Pid.ki = 0.005; 
	Speed_Pid.kd = 0.0f; 
	Speed_Pid.outMax = 20.0f;
	Speed_Pid.bias = 0.0f;
	Speed_Pid.lastBias = 0.0f;
	Speed_Pid.error = 0.0f;
	Speed_Pid.target = 0.0f;
	Speed_Pid.I_max = 20;
	
	Posi_Pid.kp = -75.0f;   //3.5
	Posi_Pid.ki = -0.0f;
	Posi_Pid.kd = -0.0f;
	Posi_Pid.outMax = 30;
	Posi_Pid.I_max = 10;
	Posi_Pid.bias = 0;
	Posi_Pid.lastBias = 0;
	Posi_Pid.error = 0;
	Posi_Pid.target = 0.0f;
	
	// 4822 : 0.2,0.038,16,4
	// 8312 : 0.03               0.188  0.0157
	// 5010 : 16.2uh  0.084r     0.078  0.039
	// 2312 ：0.1 om   21.15uh    0.133  0.0314
	Current_ID_Pid.kp = 0.1f;
	Current_ID_Pid.ki = 0.03f;
	Current_ID_Pid.outMax = 16;
	Current_ID_Pid.bias = 0;
	Current_ID_Pid.lastBias = 0;
	Current_ID_Pid.error = 0;
	Current_ID_Pid.target = 0.0f;
	Current_ID_Pid.I_max = 4;
	
	Current_IQ_Pid.kp = 0.1f;
	Current_IQ_Pid.ki = 0.03f;
	Current_IQ_Pid.kd = 0.0f;
	Current_IQ_Pid.outMax = 16;
	Current_IQ_Pid.bias = 0;
	Current_IQ_Pid.lastBias = 0;
	Current_IQ_Pid.error = 0;
	Current_IQ_Pid.target = 0.0f;
	Current_IQ_Pid.I_max = 4;
}

void MOTOR_Init(void)
{
	motor.polePair = 14;
	motor.oldMechAngle = 0.0f;
	motor.elecOffset = 3.346f;   //0.167
	motor.mechVelocityFilt = 0.0f;
	motor.rotations = 0.0f;
	motor.vbus = 24.0f;
	motor.Res = 0.084f;
	motor.inductor = 16.2f;
	LPF_vel.now = 0.0f;
	LPF_vel.pre = 0.0f;
	LPF_vel.rate = 0.1f;
//  调试用的
//	mSystemMode = MOTOR_MODE; 
//	ctrlState = CURRENT_LOOP;
}

void Encoder_Init(void)
{
	Encoder.raw = 0;
	Encoder.count_in_cpr = 0;
	Encoder.count_in_cpr_prev = 0;
	Encoder.shadow_count = 0;
	Encoder.pos_cpr_counts = 0;
	Encoder.vel_estimate_counts = 0;
	Encoder.pos = 0;
	Encoder.vel = 0;
	Encoder.phase = 0;
	Encoder.phase_vel = 0;
	
	Encoder.interpolation = 0;
	Encoder.snap_threshold = 0.5f * DT * Encoder.pll_ki;

	int encoder_pll_bw = 2000;
	float bandwidth = encoder_pll_bw;
	Encoder.pll_kp = 2.0f * bandwidth;  				// basic conversion to discrete time
  Encoder.pll_ki = 0.25f * (Encoder.pll_kp * Encoder.pll_kp);  		// Critically damped
}

void kalman_filter_init(kalman_filter* kf, double R) {
  kf->x.x = 0.0;
  kf->x.dx = 0.0;
  kf->P[0][0] = 1.0;
  kf->P[0][1] = 0.0;
  kf->P[1][0] = 0.0;
  kf->P[1][1] = 1.0;
  kf->Q[0][0] = 0.001;
  kf->Q[0][1] = 0.0;
  kf->Q[1][0] = 0.0;
  kf->Q[1][1] = 0.01;
  kf->R = R;
}


void kalman_filter_update(kalman_filter* kf, observation z, double dt) {
  // 预测
  kf->x.x += kf->x.dx * dt;
  kf->P[0][0] += kf->P[1][0] * dt + kf->P[0][1] * dt + kf->P[1][1] * dt * dt + kf->Q[0][0];
  kf->P[0][1] += kf->P[1][1] * dt;
  kf->P[1][0] += kf->P[1][1] * dt;
  kf->P[1][1] += kf->Q[1][1];

  // 更新
  kf->K[0] = kf->P[0][0] / (kf->P[0][0] + kf->R);
  kf->K[1] = kf->P[1][0] / (kf->P[0][0] + kf->R);
  kf->x.x += kf->K[0] * (z.z - kf->x.x);
  kf->x.dx += kf->K[1] * (z.z - kf->x.x);
  kf->P[0][0] *= (1.0 - kf->K[0]);
  kf->P[0][1] *= (1.0 - kf->K[0]);
  kf->P[1][0] -= kf->K[1] * kf->P[0][0];
  kf->P[1][1] -= kf->K[1] * kf->P[0][1];
}

void Start_Up(void)
{
	Voltage_DQ.Vd=START_V;
	Voltage_DQ.Vq=0.0f;
	motor.elecAngle = 0.0f;
	Angle_To_Cos_Sin(&motor); 
	Rev_Park_Transf(Voltage_DQ,&motor,&Voltage_Alpha_Beta); 
	SVPWM_Calc(&svpwm,Voltage_Alpha_Beta,Udc,PWM_PERIOD);	
	HAL_Delay(300);
	motor.elecOffset = 0.0f;
	ENCODER_sample();
	moni_offset = motor.elecAngle;
	motor.elecOffset = motor.elecAngle;
	HAL_Delay(20);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	
//	motor.ib = (int16_t)hadc->Instance->JDR2;
//	motor.ic = (int16_t)hadc->Instance->JDR3;
//	motor.ia = (int16_t)hadc->Instance->JDR1;
	
	motor.ia = (int16_t)(hadc->Instance->JDR1 - hPhaseA_OffSet) * SAMPLE_CURR_CON_FACTOR;
	motor.ic = (int16_t)(hadc->Instance->JDR3 - hPhaseC_OffSet) * SAMPLE_CURR_CON_FACTOR;
	motor.ib = -motor.ia-motor.ic;
	Clarke_Transf(&motor,&Current_Ialpha_beta);
  ENCODER_sample();
	Angle_To_Cos_Sin(&motor);
	Park_Transf(Current_Ialpha_beta,motor,&Current_Idq);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_12);
	
	switch (mSystemMode)
	{
		case UPLOOD_PM_MODE:
			if(stateChange)
			{
				SynControlPM();
				upload_control_parameter();
				stateChange = 0;
			}
			break;
		case SYN_PM_MODE_0:
			if(stateChange)
			{
				logInfo(6);
				stateChange = 0;
			}
			break;
		case SYN_PM_MODE_1:
			if(stateChange)
			{
				SYNCHRONIZE_PM();
				logInfo(5);
				stateChange = 0;
			}
			break;
		case CAlIBRATE_MODE:
			if(stateChange)
			{mCalibState = CAL_ENCODER_START;}
			CALIBRATION_LOOP();
			stateChange = 0;
			break;
		case MOTOR_MODE:
			if(stateChange == 1)
			{
				logInfo(4);
				stateChange = 0;
			}
			FOC_Control_Loop();
			SynStatePM();
			report_state_variable();
			break;
		case REPORT_MODE:
			Disable_Motor();
			SynStatePM();
			report_state_variable();
			break;
	}
//	usb_printf("%f\r\n",motor.me_angle);
//	Cal_Res_Ind_Flux(&motor);	
//	printf("%f,%f,%f\r\n",motor.ia,motor.ib,motor.ic);
//	FOC_Control_Loop();

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器回调函数
{	
	if(htim->Instance == TIM5)
	{
		if(ctrlState==POSITION_LOOP)
		{
			Posi_Pid.pre = Encoder.pos;
			Posi_Pid.target = targetPm.pos_ref;
			Posi_Pid_Calc(&Posi_Pid);
			Speed_Pid.target = -Posi_Pid.outcome;
			Speed_Pid.pre = Encoder.vel;
			PID_Calc(&Speed_Pid);	
		}
		else if(ctrlState==VELOCITY_LOOP)
		{
			Speed_Pid.target = targetPm.vel_ref;
			Speed_Pid.pre = Encoder.vel;
			PID_Calc(&Speed_Pid);	
		}
	}
}

void CALIBRATION_LOOP(void)
{
	switch(mCalibState){
		case CAL_NULL:
			break;
		case CAL_ENCODER_START:
			logInfo(1);
			mArgCali.n = 128*motor.polePair;
			mArgCali.n2 = 40;
			mArgCali.loop_count = 0;
			mArgCali.delta = 2*_PI*motor.polePair/(mArgCali.n * mArgCali.n2);
			mArgCali.theta_ref = 0.0f;
			mArgCali.theta_actual = 0.0f;
			mArgCali.v_cal = 1.0f;
			mCalibState = CAL_ENCODER_ZERO;
			break;
		// 正向旋转
		case CAL_ENCODER_ZERO:
			
			Voltage_DQ.Vd = mArgCali.v_cal;
			Voltage_DQ.Vq = 0.0f;
			motor.elecAngle = 0.0f;
			Angle_To_Cos_Sin(&motor); 
			Rev_Park_Transf(Voltage_DQ,&motor,&Voltage_Alpha_Beta); 
			SVPWM_Calc(&svpwm,Voltage_Alpha_Beta,Udc,PWM_PERIOD);	
			mArgCali.loop_count ++;
			if(mArgCali.loop_count>9999)
			{
				mArgCali.loop_count = 0;
				mCalibState = CAL_ENCODER_CW_LOOP;
			}
			break;
		
		case CAL_ENCODER_CW_LOOP:
			if(mArgCali.sample_count < mArgCali.n)
			{
				mArgCali.theta_ref += mArgCali.delta;	
				Voltage_DQ.Vd=mArgCali.v_cal;
				Voltage_DQ.Vq=0.0f;
				motor.elecAngle = mArgCali.theta_ref;
				Angle_To_Cos_Sin(&motor); 
				Rev_Park_Transf(Voltage_DQ,&motor,&Voltage_Alpha_Beta); 
				SVPWM_Calc(&svpwm,Voltage_Alpha_Beta,Udc,PWM_PERIOD);	
				ENCODER_sample();
				mArgCali.loop_count ++;
				if(mArgCali.loop_count > mArgCali.n2-1)
				{
					mArgCali.theta_actual = motor.mechAngle;
					float count_ref = mArgCali.theta_ref/motor.polePair;
					float error = mArgCali.theta_actual - count_ref;
					error_f[mArgCali.sample_count] = error + _2PI * (error<0);	
					mArgCali.sample_count ++;
					mArgCali.loop_count = 0;
				}
			}
			else
			{
				mArgCali.theta_ref += mArgCali.delta;
				mArgCali.sample_count --;
				mArgCali.loop_count = 0;
				mCalibState = CAL_ENCODER_CCW_LOOP;
			}
			break;
		//反向旋转
		case CAL_ENCODER_CCW_LOOP:
			if(mArgCali.sample_count > -1)
			{
				mArgCali.theta_ref -= mArgCali.delta;	
				Voltage_DQ.Vd=mArgCali.v_cal;
				Voltage_DQ.Vq=0.0f;
				motor.elecAngle = mArgCali.theta_ref;
				Angle_To_Cos_Sin(&motor); 
				Rev_Park_Transf(Voltage_DQ,&motor,&Voltage_Alpha_Beta); 
				SVPWM_Calc(&svpwm,Voltage_Alpha_Beta,Udc,PWM_PERIOD);	
				ENCODER_sample();
				mArgCali.loop_count ++;
				if(mArgCali.loop_count > mArgCali.n2-1)
				{
					mArgCali.theta_actual = motor.mechAngle;
					float count_ref = mArgCali.theta_ref/motor.polePair;
					float error = mArgCali.theta_actual - count_ref;
					error_f[mArgCali.sample_count] += (error + _2PI * (error<0));	
					mArgCali.sample_count --;
					mArgCali.loop_count = 0;
				}
			}
			else
			{
				mArgCali.theta_ref = 0;
				mArgCali.sample_count = 0;
				mArgCali.loop_count = 0;
				mCalibState = CAL_ENCODER_END;
			}
			break;
		case CAL_ENCODER_END:
			mArgCali.offset_temp = 0.0f;   
			for(int i = 0; i<mArgCali.n; i++)
			{
				mArgCali.offset_temp += (error_f[i])/(2.0f*(mArgCali.n));  // 计算平均位置传感器偏移
			}
			mArgCali.offset_temp = fmod(mArgCali.offset_temp*motor.polePair, 2*_PI);
			logInfo(2);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
			mCalibState = CAL_NULL;
			break;
	}
		
}

void Zero_Current(void)
{
	int adc1_offset = 0;
	int adc2_offset = 0;
	int adc3_offset = 0;
	int n = 1024;
	for (int i = 0; i<n; i++)
  {                                             
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
		
    ADC1->CR2  |= ADC_CR2_SWSTART;              // Begin sample and conversion
    delay_us(100);
		adc1_offset += ADC1->JDR1;
    adc2_offset += ADC1->JDR2;
		adc3_offset += ADC1->JDR3;
	}
	motor.PhaseA_OffSet = adc1_offset/n;
	motor.PhaseB_OffSet = adc2_offset/n;
	motor.PhaseC_OffSet = adc3_offset/n;
	printf("%d,%d,%d\r\n",motor.PhaseA_OffSet,motor.PhaseB_OffSet,motor.PhaseC_OffSet);
}

void Disable_Motor(void)
{
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(u16)((TIM1->ARR>>1)*(0.5f)));
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(u16)((TIM1->ARR>>1)*(0.5f)));
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(u16)((TIM1->ARR>>1)*(0.5f)));
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,1050);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,1050);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,1050);
}

void SYNCHRONIZE_PM(void)
{
	Posi_Pid.kp = pm_buf[0];
	Posi_Pid.outMax = pm_buf[1];
	Speed_Pid.kp = pm_buf[2];
	Speed_Pid.ki = pm_buf[3];
	Speed_Pid.I_max = pm_buf[4];
	Speed_Pid.outMax = pm_buf[5];
	Current_ID_Pid.kp = pm_buf[6];
	Current_ID_Pid.ki = pm_buf[7];
	Current_ID_Pid.I_max = pm_buf[8];
	Current_ID_Pid.outMax = pm_buf[9];
	Current_IQ_Pid.kp = pm_buf[6];
	Current_IQ_Pid.ki = pm_buf[7];
	Current_IQ_Pid.I_max = pm_buf[8];
	Current_IQ_Pid.outMax = pm_buf[9];
}

void FOC_Control_Loop(void)
{
//	GetMotorCurrent();
	if(ctrlState == CURRENT_LOOP)
	{
		Current_ID_Pid.pre = Current_Idq.Id;
		Current_IQ_Pid.pre = Current_Idq.Iq;
		Current_ID_Pid.target = 0.0f;
		Current_IQ_Pid.target = targetPm.iq_ref;
		PID_Calc(&Current_ID_Pid);  
		PID_Calc(&Current_IQ_Pid);  	
		Voltage_DQ.Vd = Current_ID_Pid.outcome;
		Voltage_DQ.Vq = Current_IQ_Pid.outcome;
//		Voltage_DQ.Vd = 0;
//		Voltage_DQ.Vq = targetPm.iq_ref;
	}
	else
	{
		Current_ID_Pid.pre = Current_Idq.Id;
		Current_IQ_Pid.pre = Current_Idq.Iq;
		Current_ID_Pid.target = 0.0f;
		Current_IQ_Pid.target = Speed_Pid.outcome;
		PID_Calc(&Current_ID_Pid);  
		PID_Calc(&Current_IQ_Pid);  	
		Voltage_DQ.Vd = Current_ID_Pid.outcome;
		Voltage_DQ.Vq = Current_IQ_Pid.outcome;
	}
	Rev_Park_Transf(Voltage_DQ,&motor,&Voltage_Alpha_Beta); 
	SVPWM_Calc(&svpwm,Voltage_Alpha_Beta,Udc,PWM_PERIOD);
}

void PID_Calc(PID_Struct *pid_temp)
{
	float b_temp;
	pid_temp->bias =  pid_temp->target- pid_temp->pre;
	pid_temp->I_sum += (pid_temp->ki * pid_temp->bias);
	//积分限幅
	if (pid_temp->I_sum > pid_temp->I_max) {
		pid_temp->I_sum = pid_temp->I_max;
	} 
	else if (pid_temp->I_sum < -pid_temp->I_max) {
		pid_temp->I_sum = -pid_temp->I_max;
	}
	
	b_temp = pid_temp->kp * pid_temp->bias + pid_temp->I_sum;
	//输出限幅
	if (b_temp > pid_temp->outMax) {
		pid_temp->outcome= pid_temp->outMax;
	} 
	else if (b_temp < -pid_temp->outMax) {
		pid_temp->outcome = -pid_temp->outMax;
	}
	else pid_temp->outcome = b_temp;
}

void Posi_Pid_Calc(PID_Struct* posi_pid_temp) 	 
{
//	float bias_temp = posi_pid_temp->pre - posi_pid_temp->target;
//	if (bias_temp < -_PI) bias_temp += _2PI;
//	else if (bias_temp > _PI) bias_temp -= _2PI;
//	posi_pid_temp->bias = -bias_temp;
	posi_pid_temp->bias = -(posi_pid_temp->pre - posi_pid_temp->target);
	posi_pid_temp->outcome = posi_pid_temp->kp*posi_pid_temp->bias ;
	posi_pid_temp->lastBias = posi_pid_temp->bias;
	if (posi_pid_temp->outcome > posi_pid_temp->outMax) {
		posi_pid_temp->outcome = posi_pid_temp->outMax;
	} 
	if (posi_pid_temp->outcome < -posi_pid_temp->outMax) {
		posi_pid_temp->outcome = -posi_pid_temp->outMax;
	}
	
}

float Low_Pass_Filter(LPF* lpf_temp,float value)
{
	lpf_temp->now = lpf_temp->rate * value + (1-lpf_temp->rate)*lpf_temp->pre;
	lpf_temp->pre=lpf_temp->now;
	return lpf_temp->now;
}

float Get_DCBUS(void)
{
	float temp;
	HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1,50);
	temp = HAL_ADC_GetValue(&hadc1);
	
	return temp;
}

void ENCODER_sample(void)
{
	/* Linearization */
	int count = SPI_AS5047_ReadData();
	Encoder.raw = count;
	/*  Wrap in ENCODER_CPR */
	while(count > ENCODER_CPR) count -= ENCODER_CPR;
    while(count < 0          ) count += ENCODER_CPR;
	Encoder.count_in_cpr = count;
	
	/* Delta count */
	int delta_count = Encoder.count_in_cpr - Encoder.count_in_cpr_prev;
	Encoder.count_in_cpr_prev = Encoder.count_in_cpr;
	while(delta_count > +ENCODER_CPR_DIV) delta_count -= ENCODER_CPR;
    while(delta_count < -ENCODER_CPR_DIV) delta_count += ENCODER_CPR;
	
	/* Add measured delta to encoder count */
    Encoder.shadow_count += delta_count;  

	/* Run vel PLL */
    Encoder.pos_cpr_counts += DT * Encoder.vel_estimate_counts;
	float delta_pos_cpr_counts = (float)(Encoder.count_in_cpr - (int)Encoder.pos_cpr_counts);
	while(delta_pos_cpr_counts > +ENCODER_CPR_DIV) delta_pos_cpr_counts -= ENCODER_CPR_F;
    while(delta_pos_cpr_counts < -ENCODER_CPR_DIV) delta_pos_cpr_counts += ENCODER_CPR_F;
    Encoder.pos_cpr_counts += DT * Encoder.pll_kp * delta_pos_cpr_counts;
	while(Encoder.pos_cpr_counts > ENCODER_CPR) Encoder.pos_cpr_counts -= ENCODER_CPR_F;
    while(Encoder.pos_cpr_counts < 0          ) Encoder.pos_cpr_counts += ENCODER_CPR_F;
    Encoder.vel_estimate_counts += DT * Encoder.pll_ki * delta_pos_cpr_counts;
	motor.mechAngle = (float)Encoder.pos_cpr_counts/ ENCODER_CPR_F * _2PI;
	float elec_temp = motor.mechAngle*motor.polePair - motor.elecOffset;
	while(elec_temp>_2PI)
	{
		elec_temp -= _2PI;
	}
	motor.elecAngle = elec_temp;
	// align delta-sigma on zero to prevent jitter
	bool snap_to_zero_vel = false;
    if (ABS(Encoder.vel_estimate_counts) < Encoder.snap_threshold) {
        Encoder.vel_estimate_counts = 0.0f;
        snap_to_zero_vel = true;
    }
	
	// run encoder count interpolation
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel) {
        Encoder.interpolation = 0.5f;
    // reset interpolation if encoder edge comes
    } else if (delta_count > 0) {
        Encoder.interpolation = 0.0f;
    } else if (delta_count < 0) {
        Encoder.interpolation = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        Encoder.interpolation += DT * Encoder.vel_estimate_counts;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (Encoder.interpolation > 1.0f) Encoder.interpolation = 1.0f;
        if (Encoder.interpolation < 0.0f) Encoder.interpolation = 0.0f;
    }
    float interpolated_enc = Encoder.count_in_cpr+ Encoder.interpolation;
	while(interpolated_enc > ENCODER_CPR) interpolated_enc -= ENCODER_CPR;
    while(interpolated_enc < 0          ) interpolated_enc += ENCODER_CPR;
	
	float shadow_count_f = Encoder.shadow_count;
    float turns = shadow_count_f / ENCODER_CPR_F;
    float residual = shadow_count_f - turns * ENCODER_CPR_F;
	
	/* Outputs from Encoder for Controller */
	Encoder.pos = turns + residual / ENCODER_CPR_F;
//	Encoder.vel = Encoder.vel_estimate_counts / ENCODER_CPR_F;
	Encoder.vel = 0.99f*Encoder.vel + 0.01*(Encoder.vel_estimate_counts / ENCODER_CPR_F);
	Encoder.phase = (interpolated_enc * _2PI * motor.polePair) / ENCODER_CPR_F;
	Encoder.phase_vel = Encoder.vel * _2PI * motor.polePair;
}

void Clarke_Transf(MOTOR* motor_temp,CURRENT_ALPHA_BETA* Current_alpha_beta_temp)
{
	Current_alpha_beta_temp->Ialpha = motor_temp->ia ;
	Current_alpha_beta_temp->Ibeta = (motor_temp->ia + 2*motor_temp->ib) * 0.57735026918963F;
}																				   

void Angle_To_Cos_Sin(MOTOR* motor_temp)
{
	motor_temp ->Cos = cos_f32(motor_temp ->elecAngle);
	motor_temp ->Sin = sin_f32(motor_temp ->elecAngle);
}

void Park_Transf(CURRENT_ALPHA_BETA current_alpha_beta_temp,MOTOR motor_temp,CURRENT_DQ* current_dq_temp)
{
	current_dq_temp->Id = current_alpha_beta_temp.Ialpha * motor_temp.Cos + current_alpha_beta_temp.Ibeta * motor_temp.Sin;
	current_dq_temp->Iq = -current_alpha_beta_temp.Ialpha * motor_temp.Sin + current_alpha_beta_temp.Ibeta * motor_temp.Cos;
}


void Rev_Park_Transf(VOLTAGE_DQ v_dq_temp,MOTOR* motor_temp,VOLTAGE_ALPHA_BETA* v_alpha_beta_temp)
{
  v_alpha_beta_temp->Valpha = motor_temp->Cos * v_dq_temp.Vd - motor_temp->Sin * v_dq_temp.Vq;
  v_alpha_beta_temp->Vbeta  = motor_temp->Sin * v_dq_temp.Vd + motor_temp->Cos * v_dq_temp.Vq;
}


void SVPWM_Calc(SVPWM* svpwm_temp,VOLTAGE_ALPHA_BETA v_alpha_beta_temp,float Udc_temp,float Tpwm_temp)
{
	float f_temp,Tcmp1,Tcmp2,Tcmp3;
	svpwm_temp->sector = 0;
	
  if (v_alpha_beta_temp.Vbeta > 0.0F) {  // U1
    svpwm_temp->sector = 1;
  }
	if ((1.73205078F * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F) {  //U2
    svpwm_temp->sector += 2;
  }
  
  if ((-1.73205078F * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F) {   //U3
    svpwm_temp->sector += 4;
  }
  
  switch (svpwm_temp->sector) {
  case 1:  //  secotr 2
    svpwm_temp->Tx = (-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);	//	-Sqrt(3)*U2
    svpwm_temp->Ty = (1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);		//	-Sqrt(3)*U3
    break;
    
  case 2:  //  secotr 6
    svpwm_temp->Tx = (1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);  	//	-Sqrt(3)*U3
    svpwm_temp->Ty = -(1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);		//	-Sqrt(3)*U1
    break;
    
  case 3:  // sector 1
    svpwm_temp->Tx = -((-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));   // Sqrt(3)* U2 
    svpwm_temp->Ty = 1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;   // sqrt(3)*U1
    break;
    
  case 4:  // sector 4
    svpwm_temp->Tx = -(1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);  //	-Sqrt(3)*U1
    svpwm_temp->Ty = (-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);   //	-Sqrt(3)*U2
    break;
    
  case 5:  // sector 3
    svpwm_temp->Tx = 1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;  //	Sqrt(3)*U1
    svpwm_temp->Ty = -((1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));   	//	Sqrt(3)*U3
    break;
    
  default:  // sector 5
    svpwm_temp->Tx = -((1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));    //  sqrt(3)*U3
    svpwm_temp->Ty = -((-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));		//	sqrt(3)*U2
    break;
  }
  
  f_temp = svpwm_temp->Tx + svpwm_temp->Ty;
  if (f_temp > Tpwm_temp) {
    svpwm_temp->Tx /= f_temp;
    svpwm_temp->Ty /= svpwm_temp->Tx + svpwm_temp->Ty;		
  }
  
  Tcmp1 = (Tpwm_temp - (svpwm_temp->Tx + svpwm_temp->Ty)) / 4.0F;
  Tcmp2 = svpwm_temp->Tx / 2.0F + Tcmp1;
  Tcmp3 = svpwm_temp->Ty / 2.0F + Tcmp2;
  switch (svpwm_temp->sector) {
  case 1:
    svpwm_temp->Tb = Tcmp2;
    svpwm_temp->Ta = Tcmp1;
    svpwm_temp->Tc = Tcmp3;
    break;
    
  case 2:
    svpwm_temp->Tb = Tcmp1;
    svpwm_temp->Ta = Tcmp3;
    svpwm_temp->Tc = Tcmp2;
    break;
    
  case 3:
    svpwm_temp->Tb = Tcmp1;
    svpwm_temp->Ta = Tcmp2;
    svpwm_temp->Tc = Tcmp3;
    break;
    
  case 4:
    svpwm_temp->Tb = Tcmp3;
    svpwm_temp->Ta = Tcmp2;
    svpwm_temp->Tc = Tcmp1;
    break;
    
  case 5:
    svpwm_temp->Tb = Tcmp3;
    svpwm_temp->Ta = Tcmp1;
    svpwm_temp->Tc = Tcmp2;
    break;
    
  case 6:
    svpwm_temp->Tb = Tcmp2;
    svpwm_temp->Ta = Tcmp3;
    svpwm_temp->Tc = Tcmp1;
    break;
  }
	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(u16)svpwm_temp->Ta);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(u16)svpwm_temp->Tb);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(u16)svpwm_temp->Tc);

}


