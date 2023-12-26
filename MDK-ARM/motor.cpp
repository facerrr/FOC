#include "motor.h"
#include "as5047.h"
#include "report.h"
MOTOR motor;
SVPWM svpwm;
VOLTAGE_DQ Voltage_DQ;
VOLTAGE_ALPHA_BETA Voltage_Alpha_Beta;
CURRENT_ALPHA_BETA Current_Ialpha_beta;
CURRENT_DQ Current_Idq; 
PI_Struct Current_ID_Pid;
PID_Struct Current_ID_Pid_2;
PI_Struct Current_IQ_Pid;
PID_Struct Current_IQ_Pid_2;
PI_Struct Speed_Pid;
PID_Struct Speed_Pid_2;
PID_Struct Posi_Pid;
LPF LPF_vel;
LPF LPF_ID;
LPF LPF_IQ;
LPF LPF_IA;
LPF LPF_IB;
LPF LPF_IC;
float value = -3;
float moni_vel;
float moni_angle;
float moni_offset = 5.7;
float target_angle = 3.14;
float target_vel = 50;
float ID_ref;
float IQ_ref;
float DC_BUS;
float moni_iq;
float moni_id;
float moni_id_out;
uint32_t hPhaseA_OffSet=1938;//2037
uint32_t hPhaseB_OffSet=1985;//2043
uint32_t hPhaseC_OffSet=2017;//2043

static u16 Vel_Timebase = SPEED_SAMPLING_TIME;   // value = 1
static u16 Posi_Timebase = POSI_SAMPLING_TIME;
static CalibState mCalibState = CAL_NULL;
uint8_t send_buf[12] = {0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d, 0x0a};

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
	moni_offset = Calibration(&motor);
//	Start_Up();
}

void PID_Init(void){
	Speed_Pid.kp = 0.003f;
	Speed_Pid.ki = 0.0005f; 
	Speed_Pid.outMax = 7.0f;
	Speed_Pid.bias = 0.0f;
	Speed_Pid.lastBias = 0.0f;
	Speed_Pid.error = 0.0f;
	Speed_Pid.target = 0.0f;
	
	Speed_Pid_2.kp = 0.95;
	Speed_Pid_2.ki = 0.016f; 
	Speed_Pid_2.kd = 0.0f; 
	Speed_Pid_2.outMax = 16.0f;
	Speed_Pid_2.bias = 0.0f;
	Speed_Pid_2.lastBias = 0.0f;
	Speed_Pid_2.error = 0.0f;
	Speed_Pid_2.target = 0.0f;
	Speed_Pid_2.I_max = 4;
	
	Posi_Pid.kp = -17.0f;
	Posi_Pid.ki = -0.0f;
	Posi_Pid.kd = -0.0f;
	Posi_Pid.outMax = 100;
	Posi_Pid.I_max = 10;
	Posi_Pid.bias = 0;
	Posi_Pid.lastBias = 0;
	Posi_Pid.error = 0;
	Posi_Pid.target = 0.0f;
	
	Current_ID_Pid.kp = 0.000f;
	Current_ID_Pid.ki = 0.00005f;
	Current_ID_Pid.outMax = 6;
	Current_ID_Pid.bias = 0;
	Current_ID_Pid.lastBias = 0;
	Current_ID_Pid.error = 0;
	Current_ID_Pid.target = 0.0f;
	
	Current_IQ_Pid.kp = 0.0f;
	Current_IQ_Pid.ki = 0.0f;
	Current_IQ_Pid.outMax = 4;
	Current_IQ_Pid.bias = 0;
	Current_IQ_Pid.lastBias = 0;
	Current_IQ_Pid.error = 0;
	Current_IQ_Pid.target = 0.0f;
	
	// 4822 : 0.2,0.038,16,4
	// 8312 : 0.03
	// 5010 : 16.2uh  0.084r
	Current_ID_Pid_2.kp = 0.1f;
	Current_ID_Pid_2.ki = 0.026f;
	Current_ID_Pid_2.outMax = 16;
	Current_ID_Pid_2.bias = 0;
	Current_ID_Pid_2.lastBias = 0;
	Current_ID_Pid_2.error = 0;
	Current_ID_Pid_2.target = 0.0f;
	Current_ID_Pid_2.I_max = 4;
	
	Current_IQ_Pid_2.kp = 0.1f;
	Current_IQ_Pid_2.ki = 0.026f;
	Current_IQ_Pid_2.kd = 0.0f;
	Current_IQ_Pid_2.outMax = 16;
	Current_IQ_Pid_2.bias = 0;
	Current_IQ_Pid_2.lastBias = 0;
	Current_IQ_Pid_2.error = 0;
	Current_IQ_Pid_2.target = 0.0f;
	Current_IQ_Pid_2.I_max = 4;
}

void MOTOR_Init(void)
{
	motor.polePair = 14;
	motor.pre_angle = 0.0f;
	motor.encoder_offset = 0.065f;   //0.6f
	LPF_vel.now = 0.0f;
	LPF_vel.pre = 0.0f;
	LPF_vel.rate = 0.1f;
	LPF_ID.now = 0.0f;
	LPF_IQ.pre = 0.0f;
	LPF_ID.now = 0.0f;
	LPF_IQ.pre = 0.0f;
	LPF_ID.rate = 0.99f;
	LPF_IQ.rate = 0.99f;
	
	LPF_IA.now = 0.0f;
	LPF_IB.now = 0.0f;
	LPF_IC.now = 0.0f;
	
	LPF_IA.pre = 0.0f;
	LPF_IB.pre = 0.0f;
	LPF_IC.pre = 0.0f;
	
	LPF_IA.rate = 0.09f;
	LPF_IB.rate = 0.09f;
	LPF_IC.rate = 0.09f;
	
}

void Start_Up(void)
{
	Voltage_DQ.Vd=START_V;
	Voltage_DQ.Vq=0.0f;
	motor.el_angle = 0.0f;
	motor.encoder_offset = 0.0f;
	Angle_To_Cos_Sin(&motor); 
	Rev_Park_Transf(Voltage_DQ,&motor,&Voltage_Alpha_Beta); 
	SVPWM_Calc(&svpwm,Voltage_Alpha_Beta,Udc,PWM_PERIOD);	
	HAL_Delay(300);
	ENC_Get_Electrical_Angle(&motor);
	moni_offset = motor.el_angle;
	motor.encoder_offset = motor.el_angle;
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
//	motor.ia = (int16_t)(hadc->Instance->JDR1 - hPhaseA_OffSet) * SAMPLE_CURR_CON_FACTOR;
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_12);

//	CDC_Transmit_FS(send_buf,12);
	report_angle_speed();
//	usb_printf("%f\r\n",motor.me_angle);
//	Cal_Res_Ind_Flux(&motor);	
//	printf("%f,%f,%f\r\n",motor.ia,motor.ib,motor.ic);
//	FOC_Control_Loop();
//	TEST_FUN();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器回调函数
{	
	if(htim->Instance == TIM5)
	{
		
		GetMotorSpeed(&motor);
		Posi_Timebase = SPEED_SAMPLING_TIME;
		moni_vel = motor.velocity;
		Posi_Pid.pre = motor.me_angle;
		Posi_Pid.target = target_angle;
		Posi_Pid_Calc(&Posi_Pid);
		Speed_Pid_2.target = -Posi_Pid.outcome;
//		Speed_Pid_2.target = target_vel;
		Speed_Pid_2.pre = moni_vel;
		Speed_Pid_Calc_2(&Speed_Pid_2);		
	}
}
void my_delay_ms(u16 time)
{    
   u16 i=0;  
   while(time--)
   {
      i=12000;
      while(i--) ;    
   }
}

void TEST_FUN(void)
{
	static int count=0;
	if (count<80000){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0.04*PWM_PERIOD);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
		count ++;
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
	}
	
	
}

float Calibration(MOTOR* motor_temp)
{
	int NPP = 14;
	int n = 128*NPP;
	int n2 = 40;   
	float error_f[n];													  //正旋转记录的旋转角偏移量
  float error_b[n];
	float delta = 2*_PI*NPP/(n*n2);   
	printf("%f\r\n",delta);
	float error[n];
	float raw_f[n];
	float raw_b[n];
	float theta_ref = 0;
	float theta_actual = 0;		
	float v_d = 0.5;                              //把所有的电压放在D轴上
  float v_q = 0.0f;		
	Voltage_DQ.Vd=0.15f;
	Voltage_DQ.Vq=0.0f;
	motor.el_angle = theta_ref;
	motor.encoder_offset = 0.0f;
	Angle_To_Cos_Sin(&motor); 
	Rev_Park_Transf(Voltage_DQ,&motor,&Voltage_Alpha_Beta); 
	SVPWM_Calc(&svpwm,Voltage_Alpha_Beta,Udc,PWM_PERIOD);	
	for(int i = 0; i<n; i++)	
	{                                                   
		for(int j = 0; j<n2; j++) 					
		{  
			theta_ref += delta;	
			Voltage_DQ.Vd=0.5f;
			Voltage_DQ.Vq=0.0f;
			motor.el_angle = theta_ref;
			motor.encoder_offset = 0.0f;
			Angle_To_Cos_Sin(&motor); 
			Rev_Park_Transf(Voltage_DQ,&motor,&Voltage_Alpha_Beta); 
			SVPWM_Calc(&svpwm,Voltage_Alpha_Beta,Udc,PWM_PERIOD);	
			delay_us(100);
		}
		ENC_Get_Electrical_Angle(&motor);
		theta_actual = motor.me_angle;
		error_f[i] = theta_ref/NPP - theta_actual;		
	}
	for(int i = 0; i<n; i++)	
	{                                                   
		for(int j = 0; j<n2; j++) 					
		{  
			theta_ref -= delta;	
			Voltage_DQ.Vd=0.5f;
			Voltage_DQ.Vq=0.0f;
			motor.el_angle = theta_ref;
			motor.encoder_offset = 0.0f;
			Angle_To_Cos_Sin(&motor); 
			Rev_Park_Transf(Voltage_DQ,&motor,&Voltage_Alpha_Beta); 
			SVPWM_Calc(&svpwm,Voltage_Alpha_Beta,Udc,PWM_PERIOD);	
			delay_us(100);
		}
		ENC_Get_Electrical_Angle(&motor);
		theta_actual = motor.me_angle;
		error_b[i] = theta_ref/NPP - theta_actual;		
	}
	
	float offset = 0.0f;   
//	for(int i = 0; i<n; i++)
//	{
//		offset += (error_f[i] + error_b[1791-i]);           // 计算平均位置传感器偏移
//	}
//	
	return offset;
//	while(offset>_2PI)
//	{
//		offset-=_2PI;
//	}
//		offset-=_2PI;
}

void Cal_Res_Ind_Flux(MOTOR* motor_temp)
{
	static float duty;
	static unsigned int x=0, k=0, y=0;
	static float i[500];
	static float Ir_b;
	static float Ir_c;
	static float U_bus;
	static int cnt=0;
	
	switch (mCalibState){
		case CAL_NULL:
			mCalibState = CAL_RES_START;
			printf("CAL_RES_Prepare\r\n");
			break;
		case CAL_RES_START:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
			my_delay_ms(2000);
			printf("CAL_RES_START\r\n");
			mCalibState = CAL_RES;
			break;
		case CAL_RES:
			Ir_b = motor_temp->ib;
			Ir_c = motor_temp->ic;
			U_bus = 24*duty;
			cnt ++;
			if(Ir_b + Ir_c<6)
			{	
				if (cnt>200)
				{
					duty += 0.001;
					cnt = 0 ;
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(u16)duty*PWM_PERIOD);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
				}
				if (duty>=0.8)
				{
					printf("res.Ir = %f\r\n",Ir_b+Ir_c);
					printf("Check the maximum of current\r\n");
					mCalibState = CAL_RES_END;
				}
				if (Ir_b + Ir_c<-2)
				{
					printf("Current direction is false\r\n");
					mCalibState = CAL_RES_END;
				}
			}
			else
			{
				motor_temp->Res = (U_bus/(Ir_b+Ir_c))*2/3;
				cnt = 0;
				mCalibState = CAL_RES_END;
				printf("Calculate Res Sucessfully\r\n");
				printf("The Res of motor is %f\r\n",motor_temp->Res);
				
			}
			break;
		case CAL_RES_END:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
			break;
	}
	
}
void FOC_Control_Loop(void)
{
//	GetMotorCurrent();
	Clarke_Transf(&motor,&Current_Ialpha_beta);
	ENC_Get_Electrical_Angle(&motor);
	Angle_To_Cos_Sin(&motor);
	Park_Transf(Current_Ialpha_beta,motor,&Current_Idq);
	moni_iq = Current_Idq.Iq;
	moni_id = Current_Idq.Id;
	Current_ID_Pid_2.pre = Current_Idq.Id;
	Current_IQ_Pid_2.pre = Current_Idq.Iq;
	Current_ID_Pid_2.target = 0.0f;
	Current_IQ_Pid_2.target = Speed_Pid_2.outcome;
	Current_Pid_Calc_2(&Current_ID_Pid_2);  
	Current_Pid_Calc_2(&Current_IQ_Pid_2);  	
	moni_id_out = Current_ID_Pid_2.outcome;
	Voltage_DQ.Vd = 0;
	Voltage_DQ.Vq = 2.5f;
	Rev_Park_Transf(Voltage_DQ,&motor,&Voltage_Alpha_Beta); 
	SVPWM_Calc(&svpwm,Voltage_Alpha_Beta,Udc,PWM_PERIOD);
}

void Posi_Pid_Calc(PID_Struct* posi_pid_temp) 	 
{
	float bias_temp = posi_pid_temp->pre - posi_pid_temp->target;
	if (bias_temp < -_PI) bias_temp += _2PI;
	else if (bias_temp > _PI) bias_temp -= _2PI;
	posi_pid_temp->bias = -bias_temp;
	
	posi_pid_temp->I_sum += (posi_pid_temp->ki * posi_pid_temp->bias);
	if (posi_pid_temp->I_sum > posi_pid_temp->I_max) {
		posi_pid_temp->I_sum = posi_pid_temp->I_max;
	} 
	else if (posi_pid_temp->I_sum < -posi_pid_temp->I_max) {
		posi_pid_temp->I_sum = -posi_pid_temp->I_max;
	}
	
	posi_pid_temp->outcome = posi_pid_temp->kp*posi_pid_temp->bias + posi_pid_temp->I_sum + posi_pid_temp->kd * (posi_pid_temp->bias - posi_pid_temp->lastBias);
	posi_pid_temp->lastBias = posi_pid_temp->bias;
	if (posi_pid_temp->outcome > posi_pid_temp->outMax) {
		posi_pid_temp->outcome = posi_pid_temp->outMax;
	} 
	if (posi_pid_temp->outcome < -posi_pid_temp->outMax) {
		posi_pid_temp->outcome = -posi_pid_temp->outMax;
	}
	
}

void Current_Pid_Calc_2(PID_Struct* current_pid_temp)
{
	float b_temp;
	current_pid_temp->bias =  current_pid_temp->target- current_pid_temp->pre;
	current_pid_temp->I_sum += (current_pid_temp->ki * current_pid_temp->bias);
	
	if (current_pid_temp->I_sum > current_pid_temp->I_max) {
		current_pid_temp->I_sum = current_pid_temp->I_max;
	} 
	else if (current_pid_temp->I_sum < -current_pid_temp->I_max) {
		current_pid_temp->I_sum = -current_pid_temp->I_max;
	}
	b_temp = current_pid_temp->kp * current_pid_temp->bias + current_pid_temp->I_sum;
	
	if (b_temp > current_pid_temp->outMax) {
		current_pid_temp->outcome= current_pid_temp->outMax;
	} 
	else if (b_temp < -current_pid_temp->outMax) {
		current_pid_temp->outcome = -current_pid_temp->outMax;
	}
	else current_pid_temp->outcome = b_temp;
	
	
	
}


void Speed_Pid_Calc_2(PID_Struct* speed_pid_temp)
{
	float b_temp;
	speed_pid_temp->bias =  speed_pid_temp->target- speed_pid_temp->pre;
	speed_pid_temp->I_sum += (speed_pid_temp->ki * speed_pid_temp->bias);
	
	if (speed_pid_temp->I_sum > speed_pid_temp->I_max) {
		speed_pid_temp->I_sum = speed_pid_temp->I_max;
	} 
	else if (speed_pid_temp->I_sum < -speed_pid_temp->I_max) {
		speed_pid_temp->I_sum = -speed_pid_temp->I_max;
	}
	b_temp = speed_pid_temp->kp * speed_pid_temp->bias + speed_pid_temp->I_sum + speed_pid_temp->kd * (speed_pid_temp->bias - speed_pid_temp->lastBias);
	
	if (b_temp > speed_pid_temp->outMax) {
		speed_pid_temp->outcome= speed_pid_temp->outMax;
	} 
	else if (b_temp < -speed_pid_temp->outMax) {
		speed_pid_temp->outcome = -speed_pid_temp->outMax;
	}
	else speed_pid_temp->outcome = b_temp;
	speed_pid_temp->lastBias = speed_pid_temp->bias;
}

void ENC_Get_Electrical_Angle(MOTOR* motor_temp)
{
	float el_temp;
	motor_temp ->me_angle = (float)SPI_AS5047_ReadData()/ 16384.0f * _2PI;
	moni_angle = motor_temp->me_angle;
//	motor_temp ->me_angle += 0.01;
	el_temp = motor_temp->me_angle * motor_temp->polePair ;
	while(el_temp>_2PI)
		el_temp-=_2PI;
	
	motor_temp ->el_angle = el_temp;
}

void GetMotorSpeed(MOTOR* motor_temp)
{	
	float vel_temp;
	vel_temp = motor_temp->me_angle - motor_temp->pre_angle;
	if (vel_temp < -_PI){
		vel_temp += _2PI;
	}
	else if (vel_temp > _PI){
		vel_temp -= _2PI;
	}
	motor_temp->pre_angle = motor_temp->me_angle;
	vel_temp = Low_Pass_Filter(&LPF_vel,vel_temp);
	vel_temp = vel_temp*1000;
	
	motor_temp->velocity = vel_temp;
}

void GetMotorCurrent()
{
	
	
//	motor.ic =(int16_t)(ADC1->JDR3 - hPhaseA_OffSet) * SAMPLE_CURR_CON_FACTOR;
//	motor.ic = Low_Pass_Filter(&LPF_IC,motor.ic);
	CurrentReconstruction(&motor);
}

float Low_Pass_Filter(LPF* lpf_temp,float value)
{
	lpf_temp->now = lpf_temp->rate * value + (1-lpf_temp->rate)*lpf_temp->pre;
	lpf_temp->pre=lpf_temp->now;
	return lpf_temp->now;
}

void CurrentReconstruction(MOTOR* motor_temp)
{
	switch (svpwm.sector) {
        case 3:
            motor_temp->ia =0.0f - motor_temp->ib - motor_temp->ic;
            break;
        case 1:
            motor_temp->ib =0.0f - motor_temp->ia - motor_temp->ic;
            break;
        case 5:
            motor_temp->ib =0.0f - motor_temp->ia - motor_temp->ic;
            break;
        case 4:
            motor_temp->ic =0.0f - motor_temp->ia - motor_temp->ib;
            break;
        case 6:
            motor_temp->ic =0.0f - motor_temp->ia - motor_temp->ib;
            break;
        case 2:
            motor_temp->ia =0.0f - motor_temp->ib - motor_temp->ic;
            break;
        default:
            break;
    }
	
}

float get_DCBUS(void)
{
	float temp;
	HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1,50);
	temp = HAL_ADC_GetValue(&hadc1);
	
	return temp;
}

void Clarke_Transf(MOTOR* motor_temp,CURRENT_ALPHA_BETA* Current_alpha_beta_temp)
{
	Current_alpha_beta_temp->Ialpha = motor_temp->ia ;
	Current_alpha_beta_temp->Ibeta = (motor_temp->ia + 2*motor_temp->ib) * 0.57735026918963F;
}																				   

void Angle_To_Cos_Sin(MOTOR* motor_temp)
{
	motor_temp ->Cos = cos_f32(motor_temp ->el_angle - motor_temp->encoder_offset);
	motor_temp ->Sin = sin_f32(motor_temp ->el_angle - motor_temp->encoder_offset);
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


