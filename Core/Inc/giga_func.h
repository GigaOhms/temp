
/*Control Motor Left
  pwm min = 0, pwm max = 500
*/

void set_motor_left(int pwm);

void set_motor_right(int pwm);

/*Stop Motor left*/
void stop_motor_left(void);

/*Stop Motor right*/
void stop_motor_right(void);

void stop_motor(void);

void run_motor(void);

void turn_left(void);

void turn_right(void);

void bzEnable(void);

void read_button(void);

void loop_7seg(void);

void calc_speed(int Kp,int Ki,int Speed);

void calc_PID(int Kp, int Ki, int Kd, int Speed);

void operating_mode(void);
