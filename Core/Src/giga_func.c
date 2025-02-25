#include "giga_func.h"
#include "led.h"
#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include <stdlib.h>


#define IN_LINE 0
#define OUT_LINE 1
#define STOP_LINE 2

extern volatile uint16_t analog_val[10];
extern uint8_t but_mode;
extern uint8_t but_select;
extern uint8_t cnt_mode;
extern uint8_t cnt_select;

uint16_t analog_val_low[10];
uint16_t analog_val_high[10];
uint16_t analog_val_mid[10];

uint8_t lock_mode = 0;
uint8_t lock_select = 0;
uint8_t lock_low = 0;
uint8_t lock_high = 0;
uint8_t lock_show = 0;

float err_left = 0;
float err_right = 0;
int speed_left = 0;
int speed_right = 0;
char show_char = ' ';
int current_err, last_err;
int real_cal = 0;
int line_status = 0;

uint8_t counter = 0;
uint32_t led7Time = 0;
uint32_t ledTime = 0;
uint8_t bzState = 0;
uint32_t bzTime = 0;

void stop_motor(void) {
	TIM2 ->CCR1 = 0;
	TIM2 ->CCR2 = 0;	
	TIM4 ->CCR1 = 0;
	TIM4 ->CCR2 = 0;		
}

void set_motor_right(int pwm) {
	pwm = (pwm < 0) ? 0 : pwm;
	pwm = (pwm > 500) ? 500 : pwm;
	TIM4 ->CCR1 = pwm;
	TIM4 ->CCR2 = 0;	
}

void set_motor_left(int pwm) {
	pwm = (pwm < 0) ? 0 : pwm;
	pwm = (pwm > 500) ? 500 : pwm;
	TIM2 ->CCR1 = pwm;
	TIM2 ->CCR2 = 0;	
}

void stop_motor_left(void) {
	TIM2 ->CCR1 = 0;
	TIM2 ->CCR2 = 0;	
}

void stop_motor_right(void) {
	TIM4 ->CCR1 = 0;
	TIM4 ->CCR2 = 0;	
}

void bzEnable(void) {
	bzState = 1;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, bzState);
	bzTime = HAL_GetTick() + 100;
}

void loop_7seg(void) {
	if ((uint32_t)(HAL_GetTick() - led7Time) > 100) {
		led7Time = HAL_GetTick();
		Show_1Seg(counter);
		counter++;
		if (counter >= 7)
			counter = 1;
	}
}

void read_button(void) {
	but_mode = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
	but_select = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
	
	if (but_select == 0 && lock_select == 0) {
		lock_select = 1;
		cnt_select++;
		if (cnt_select % 2 == 0)
			HAL_GPIO_WritePin(ledPC13_GPIO_Port, ledPC13_Pin, GPIO_PIN_RESET);
	} else if (but_select == 1) {
		lock_select = 0;
		stop_motor();
		if ((uint32_t)(HAL_GetTick() - ledTime) > 400) {
			ledTime = HAL_GetTick();
			HAL_GPIO_TogglePin(ledPC13_GPIO_Port, ledPC13_Pin);
		}
	}
	
	if(but_mode == 0 && lock_mode == 0) {
		//osDelay(20);
		bzEnable();
		if (cnt_select % 2 == 1)
			cnt_mode ++;
		lock_mode = 1;
		if(cnt_mode > 4)
			cnt_mode = 0;
		Show_7Seg(cnt_mode);
	} else if (but_mode == 1) lock_mode = 0;
	
	if (bzState == 1 && bzTime < HAL_GetTick()) {
		bzState = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, bzState);	
	}
}

void mqc_Calc_Speed(int Kp, int Ki, int Speed) {
  speed_left = Speed + err_left * Kp - err_right*Ki ;
  speed_right = Speed + err_right * Kp - err_left*Ki;	
	if(analog_val[1] >= analog_val_mid[1] && analog_val[8] >= analog_val_mid[8]){speed_left = 0; speed_right = 0;}		
	set_motor_left(speed_left);
	set_motor_right(speed_right);	
}

void mqc_PID(int Kp, int Ki, int Kd, int Speed) {
  // SS1	
	if(analog_val[8] >= analog_val_mid[8])                                             {current_err = 6; line_status = IN_LINE;}	
	// SS1 SS2
	if(analog_val[8] >= analog_val_mid[8] && analog_val[7] >= analog_val_mid[7])  {current_err = 5; line_status = IN_LINE;}	
	// SS2
	if(analog_val[7] >= analog_val_mid[7])                                             {current_err = 4; line_status = IN_LINE;}
	// SS2 SS3
  if(analog_val[7] >= analog_val_mid[7] && analog_val[6] >= analog_val_mid[6])  {current_err = 3; line_status = IN_LINE;}		
	//SS3
	if(analog_val[6] >= analog_val_mid[6])                                             {current_err = 2; line_status = IN_LINE;}
	//SS3 SS4
	if(analog_val[6] >= analog_val_mid[6] && analog_val[4] >= analog_val_mid[4])  {current_err = 1; line_status = IN_LINE;}		
	// SS4
	if(analog_val[4] >= analog_val_mid[4])                                             {current_err = 0; line_status = IN_LINE;}	
	// SS4 SS5 
	if(analog_val[4] >= analog_val_mid[4] && analog_val[3] >= analog_val_mid[3])  {current_err = -1;line_status = IN_LINE;}			
	// SS5
	if(analog_val[3] >= analog_val_mid[3])                                             {current_err = -2;line_status = IN_LINE;}	
	// SS5 SS6
	if(analog_val[3] >= analog_val_mid[3] && analog_val[2] >= analog_val_mid[2])  {current_err = -3;line_status = IN_LINE;}				
	// SS6
	if(analog_val[2] >= analog_val_mid[2])                                             {current_err = -4;line_status = IN_LINE;}
  // SS6 SS7
	if(analog_val[2] >= analog_val_mid[2] && analog_val[1] >= analog_val_mid[1])  {current_err = -5;line_status = IN_LINE;}				
	// SS7
	if(analog_val[1] >= analog_val_mid[1])                                             {current_err = -6;line_status = IN_LINE;}
	 
	if(analog_val[1] >= analog_val_mid[1] && analog_val[4] >= analog_val_mid[4] && analog_val[8] >= analog_val_mid[8])   {line_status = OUT_LINE;}
	
  real_cal = Kp * current_err + Kd * (current_err - last_err);
	last_err = current_err;
	
	if(current_err < 0)
		{
			if(line_status== 1)
				{ 
					/*
					For out of line, the robot need to turn with high rotational acceleration 
					if robot can't go back to line, you can set the Right motor to revert by using 
					mqc_Set_MotorA(0,speed_right) or mqc_Set_MotorA(0,speed_left) funtion
					*/
					speed_right = Speed + abs(real_cal);
					speed_left  = 0;
				}	
		  else
			  {
					speed_right = Speed + abs(real_cal);
					speed_left  = Speed - abs(real_cal)/Ki;
			  }		
		}

	if(current_err == 0)
		{
			speed_right = Speed;
			speed_left  = Speed;
		}
		
	if(current_err > 0)
		{ 
			if(line_status== 1)
			 {
				  /*
    				For out of line, the robot need to turn with high rotational acceleration 
				    if robot can't go back to line, you can set the Right motor to revert by using 
						mqc_Set_MotorA(0,speed_right) or mqc_Set_MotorA(0,speed_left) funtion
				  */
					speed_right = 0;
					speed_left  = Speed + abs(real_cal);			 
			 }	 
			 else
			 {
					speed_right = Speed - abs(real_cal)/Ki;
					speed_left  = Speed + abs(real_cal);			 
			 }

		}	
		
		if(speed_right < 0 ) speed_right = 0;
		if(speed_left < 0) speed_left = 0;
		if(speed_right > 500 ) speed_right = 500;
		if(speed_left > 500) speed_left = 500;		
	
	set_motor_left(speed_left);
	set_motor_right(speed_right);		
}

	