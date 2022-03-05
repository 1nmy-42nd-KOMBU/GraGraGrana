#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#define PORT_LIGHT NXT_PORT_S3  /* 入出力ポートの定義 */
#define PORT_TOUCH NXT_PORT_S2
#define L_MOTOR NXT_PORT_B
#define R_MOTOR NXT_PORT_C

#define BLACK 700
#define WHITE 500

DeclareTask(Task1);				/* Task1を宣言 */

void ecrobot_device_initialize(){		/* OSEK起動時の処理 */
	nxt_motor_set_speed(L_MOTOR,0,0);
	nxt_motor_set_speed(R_MOTOR,0,0);
	ecrobot_set_light_sensor_active(PORT_LIGHT);
}

void ecrobot_device_terminate(){		/* OSEK終了時の処理 */
	nxt_motor_set_speed(L_MOTOR,0,0);
	nxt_motor_set_speed(R_MOTOR,0,0);
	ecrobot_set_light_sensor_inactive(PORT_LIGHT);
}

void user_1ms_isr_type2(void){}


void sound_beep(){			/* ビープ音を鳴らすユーザ関数 */
	ecrobot_sound_tone(600, 2, 80);
	systick_wait_ms(20);
	ecrobot_sound_tone(500, 5, 80);
	systick_wait_ms(50);
}

TASK(Task1)
{
	int speed=70;
	float Kp = 0.4; //Pゲイン
	float Kd = 1.6; //Dゲイン
	int black,white,gray,light;	
	int err, err_prev;
	float turn;

        black = BLACK;
        white = WHITE;
        gray = (black + white) / 2;


	while(1){

		err_prev  = 0;

		while(ecrobot_get_touch_sensor(PORT_TOUCH) == 0){		/* TSが押されるまでループする */
			display_clear(0);
			display_goto_xy(0, 1);
			display_string("PUSH START");
			display_update();
			systick_wait_ms(10);
		}
	
		systick_wait_ms(500);		/* 500msec待つ */
		//light_tmp = ecrobot_get_light_sensor(PORT_LIGHT);
	
		while(ecrobot_get_touch_sensor(PORT_TOUCH) == 0){ // TSが押されるまでループする

			//PD制御による旋回
			light = ecrobot_get_light_sensor(PORT_LIGHT);
			err = light - gray;
			turn = Kp * err  + Kd * (err - err_prev);
			nxt_motor_set_speed(L_MOTOR,speed-turn,1);
			nxt_motor_set_speed(R_MOTOR,speed+turn,1);
			err_prev = err;		/* 一つ前の誤差を格納 */

			systick_wait_ms(10); /* wait 10msec*/

		}

		//停止
		nxt_motor_set_speed(L_MOTOR,0,1);
		nxt_motor_set_speed(R_MOTOR,0,1);

		systick_wait_ms(1000);		/* 1秒待つ */
	}


	TerminateTask();					/* 処理終了 */
}