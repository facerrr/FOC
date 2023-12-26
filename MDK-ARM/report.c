#include "report.h"
extern MOTOR motor;
extern tEncoder Encoder;
extern CURRENT_DQ Current_Idq; 
extern PID_Struct Current_ID_Pid;
extern PID_Struct Current_IQ_Pid;
extern PID_Struct Speed_Pid;
extern PID_Struct Posi_Pid;
extern argCALI mArgCali;

uint8_t state_pm_buffer[32] = {0x00};
uint8_t control_pm_buffer[60] = {0x00};
uint8_t information[8] = {0x00};
float control_pm[14] = {0.0f};
float state_pm[7] = {0.0f};

unsigned int floatToInt32(float data0) {
    unsigned int uintp32 = (*((unsigned int *) (&data0)));
    return uintp32;
}
uint32_t float2hex( float HEX )
{     
     return *( uint32_t *)&HEX;
}

uint32_t int2hex( int value )
{     
     return *( uint32_t *)&value;
}


void SynStatePM(void)
{
	state_pm[0] = Encoder.pos;
	state_pm[1] = Encoder.vel;
	state_pm[2] = motor.ia;
	state_pm[3] = motor.ib;
	state_pm[4] = motor.ic;
	state_pm[5] = Current_Idq.Id;
	state_pm[6] = Current_Idq.Iq;
}

void SynControlPM(void)
{
	control_pm[0] = Posi_Pid.kp;
	control_pm[1] = Posi_Pid.outMax;
	control_pm[2] = Speed_Pid.kp;
	control_pm[3] = Speed_Pid.ki;
	control_pm[4] = Speed_Pid.I_max;
	control_pm[5] = Speed_Pid.outMax;
	control_pm[6] = Current_ID_Pid.kp;
	control_pm[7] = Current_ID_Pid.ki;
	control_pm[8] = Current_ID_Pid.I_max;
	control_pm[9] = Current_ID_Pid.outMax;
	control_pm[10] = motor.vbus;
	control_pm[11] = (float)motor.polePair;
	control_pm[12] = motor.Res;
	control_pm[13] = motor.inductor;
}

void upload_control_parameter(void)
{
	control_pm_buffer[0] = 0x2f;
	for(int i = 0; i<14; i++)
	{	
		uint32_t pm = float2hex(control_pm[i]);
		control_pm_buffer[4*i+4] = (uint8_t) ((pm >> 24UL) & 0x000000ffUL);
    control_pm_buffer[4*i+3] = (uint8_t) ((pm >> 16UL) & 0x000000ffUL);
    control_pm_buffer[4*i+2] = (uint8_t) ((pm >> 8UL) & 0x000000ffUL);
    control_pm_buffer[4*i+1] = (uint8_t) (pm & 0x000000ffUL);
	}
	control_pm_buffer[57] = 0x00;
	control_pm_buffer[58] = 0x80;
	control_pm_buffer[59] = 0x7f;
	CDC_Transmit_FS(control_pm_buffer,60);
}

void report_state_variable(void) {
//    /* converts velocity and angle floating point data to an integer */
	state_pm_buffer[0] = 0x1f;  // Êý¾Ýid
	for(int i = 0; i<7; i++)
	{	
		uint32_t pm = float2hex(state_pm[i]);
		state_pm_buffer[4*i+4] = (uint8_t) ((pm >> 24UL) & 0x000000ffUL);
    state_pm_buffer[4*i+3] = (uint8_t) ((pm >> 16UL) & 0x000000ffUL);
    state_pm_buffer[4*i+2] = (uint8_t) ((pm >> 8UL) & 0x000000ffUL);
    state_pm_buffer[4*i+1] = (uint8_t) (pm & 0x000000ffUL);
	}
	state_pm_buffer[29] = 0x00;
	state_pm_buffer[30] = 0x80;
	state_pm_buffer[31] = 0x7f;
	CDC_Transmit_FS(state_pm_buffer,32);
}

void logInfo(uint8_t infoID)
{
	switch(infoID)
	{
		case 1:
			//info : calibration start
			information[0] = 0x3f;
			information[1] = 0xff;
			break;
		case 2:
			//info : calibration over
			information[0] = 0x4f;
			information[1] = 0xff;
			uint32_t offset = float2hex(mArgCali.offset_temp);
			information[5] = (uint8_t) ((offset >> 24UL) & 0x000000ffUL);
			information[4] = (uint8_t) ((offset >> 16UL) & 0x000000ffUL);
			information[3] = (uint8_t) ((offset >> 8UL) & 0x000000ffUL);
			information[2] = (uint8_t) (offset & 0x000000ffUL);
			break;
		case 3:
			//info : upload parameter
			information[0] = 0x5f;
			information[1] = 0xff;
			break;
		
		case 4: 
			//info : Run Mode
			information[0] = 0x6f;
			information[1] = 0xff;
			break;
		
		case 5:
			//info : Synchronize PM Sucessfully
			information[0] = 0x7f;
			information[1] = 0x1f;
			break;
		
		case 6:
			//info : Fail To Synchronize PM
			information[0] = 0x7f;
			information[1] = 0x2f;
			break;
	}
	
	CDC_Transmit_FS(information,8);
	
}



