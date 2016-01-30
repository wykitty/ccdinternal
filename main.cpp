/*
 * main.cpp
 *
 * Author: Kitty
 *
 */

#include <cassert>
#include <cstring>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/led.h>
#include <libsc/encoder.h>
#include <libsc/button.h>
#include <libbase/k60/gpio.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libsc/dir_motor.h>
#include <libsc/servo.h>
#include <libsc/motor.h>
#include <libsc/lcd_console.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/joystick.h>
#include <libsc/st7735r.h>
#include <libsc/alternate_motor.h>
#include <libsc/tsl1401cl.h>
#include <libsc/futaba_s3010.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <libbase/k60/gpio.h>
#include <math.h>
#include <stdio.h>
#include "libsc/lcd.h"
//#include <kalman.h>

//R11 C6 R22

#define BLUE	0x001F
#define CYAN	0x07FF
#define BLACK	0x0000
#define WHITE	0xFFFF
#define SERVO_MID_DEGREE 900

namespace libbase
{
namespace k60
{

Mcg::Config Mcg::GetMcgConfig()
{
	Mcg::Config config;
	config.external_oscillator_khz = 50000;
	config.core_clock_khz = 150000;
	return config;
}

}
}

using namespace libsc;
using namespace libbase::k60;

#define BLUE	0x001F
#define CYAN	0x07FF
#define BLACK	0x0000
#define WHITE	0xFFFF
#define SERVO_MID_DEGREE 900

char CCD;
Gpo buzzer;
Gpi jdown, jleft, jright, jcenter, jup;
St7735r *lcd = 0;
St7735r::Rect rect_1, rect_2, rect_3, rect_4;
LcdTypewriter *writ = 0;
LcdConsole *console = 0;
FutabaS3010 *servo = 0;     //500-1300
Tsl1401cl *ccd_up = 0;
Tsl1401cl *ccd_down = 0;
libsc::k60::JyMcuBt106 *bluetooth = 0;
uint16_t mid_angle = 900; //[500,1300]
uint16_t angle = 500;
Motor *mot=0;
AlternateMotor *altmotor =0;
std::array<uint16_t, Tsl1401cl::kSensorW> Data;
std::array<uint16_t, Tsl1401cl::kSensorW> copyData;
std::array<uint16_t, Tsl1401cl::kSensorW> prepreData;
std::array<uint16_t, Tsl1401cl::kSensorW> prevData;
uint16_t copdata[Tsl1401cl::kSensorW]={0};
uint16_t ccd_count;
uint16_t speed;
uint16_t errors, feedbkerror;
int errl[2]={0,0};
int xr[2] = {0,0};
int xl[2] = {0,0};
int errr[2]={0,0};
double kp,ki,kd;
bool lflag, rflag;
uint16_t lline, rline;
uint16_t color = 0xFFFF;
uint16_t mid;
uint16_t color2 = 0xFFE0;
uint16_t feedback, preError;
int16_t sumerror=0;
int16_t prev_err_val = 0;
int16_t dt=0, integral_val=0;
int16_t error=0;
int16_t preverror=0;

bool ispressed = false;
bool motorenabled = false;

//libsc::Lcd::Rect,int, uint16_t,int,int
bool listener(const Byte *, const size_t);
void region_set(Lcd::Rect,int, uint16_t,int,int);
void turn(void);
void print_info(uint16_t);
void print_ccd(uint16_t);
void changespeed(uint16_t);
void configuration(void);
void ccdbuffer(void);
void check_joystick(void);
void coppydata(void);
void checkangle(void);
uint16_t otsu(uint16_t *);
double calculate_error(void);
void speedPID(void);
void turnPID(void);
void PID(int16_t, int16_t);
void detmid(void);
void ledge(void);
void redge(void);


int main(void)
{
	System::Init();
	System::Time();
	const char *words = "hahahahah234567";

	configuration();

	//LED INIT TEST:
	libsc::Led::Config a;
	a.id = 3;
	a.is_active_low = 0;
	libsc::Led leda(a);
	libsc::Led::Config b;
	b.id = 0;
	b.is_active_low = 0;
	libsc::Led ledb(b);
	//

	int t;
	char buffer[33];
	char ccd_buf[50];
	char ano[30];
	//libsc::Lcd::Rect ha;
	ccd_count=0;

	kp = 0;
	ki = 0;
	kd = 0;

	while(1){
		leda.SetEnable(1);
		//writ->WriteString("haha");

		if(t != System::Time()){
			t=System::Time();
			//leda.SetEnable(1);
			//sprintf(buffer, "%d",System::Time());


			if (t%10==0){
				//check_joystick();
				ccd_up->StartSample();
				while(!ccd_up->SampleProcess()){};
				Data = ccd_up->GetData();
//				sprintf(ccd_buf, "%d", ledge);
//				writ->WriteString("midno:");
//				writ->WriteString(ccd_buf);

//				if(ccd_count==0){
//					break;
//				}
				//Data = ccd_up->GetData();
//				if(ccd_count<3){
//					ccd_count+=1;
//				}
				for(uint16_t i=0; i<Tsl1401cl::kSensorW; i++){
					lcd->SetRegion(Lcd::Rect(i, (255-Data[i])/2.0,1,1));
					lcd->FillPixel(&color,1);
				}
				ledge();
				redge();
				detmid();
				PID();
				sprintf(ano, "%d", errl[0]);
				writ->WriteString(ano);
				sprintf(ccd_buf, "%d", errr[0]);
				writ->WriteString(ccd_buf);

				/*
				for (uint16_t q=0; q<Tsl1401cl::kSensorW; q++){
					Data[q] = (prevData[q]*0.3 + Data[q]*0.7);
				}
				*/
				//print_ccd(Lcd::kWhite);

				for(uint16_t i=0; i<Tsl1401cl::kSensorW; i++){
					lcd->SetRegion(Lcd::Rect(i, (255-Data[i])/2.0,1,1));
					lcd->FillPixel(&color,1);
				}

				/*
				if(ccd_count<3){
					if(ccd_count<2){
						for (uint16_t i=0; i<Tsl1401cl::kSensorW; i++){
							prepreData[i]=0;
							prevData[i]=0;
						}
					}else{
						for (uint16_t i=0; i<Tsl1401cl::kSensorW; i++){
							prepreData[i]=0;
							prevData[i]=Data[i];
						}
					}
				}else{
					prepreData = prevData;
					prevData = Data;
				}
				*/

				System::DelayMs(5);

				for(uint16_t i=0; i<Tsl1401cl::kSensorW; i++){
					lcd->SetRegion(Lcd::Rect(i, (255-Data[i])/2.0,1,1));
					lcd->FillColor(libsc::Lcd::kBlack);
				}
			}

			//leda.SetEnable(1);

			if(t%1000 == 0){
				//console->Clear(1);
				//leda.SetEnable(1);
				//leda.Switch();
				ledb.Switch();
			}
		}

	}
	return 0;
}

bool listener(const Byte *data, const size_t size){
	char msg[128];
	switch(data[0]){
	case '5':
		motorenabled = true;
		if(speed >400){
			speed = 400;
			altmotor->SetPower(speed);
			bluetooth->SendStrLiteral("Motor on with speed 400 \n");
		}else{
			altmotor->SetPower(speed);
			sprintf(msg, "Motor on with power = %d\n", speed);
			bluetooth->SendStr(msg);
		}
		break;
	case '6':
		if (motorenabled == true){
			speed = 0;
			altmotor->SetPower(speed);
			sprintf(msg,"Motor turned off");
			bluetooth->SendStr(msg);
			motorenabled = false;
		}else{
			sprintf(msg, "Motor is already off");
			bluetooth->SendStr(msg);
		}
		break;
	case '1':
		sprintf(msg, "Status of Motor: %d\n Speed of Motor: %d\n",motorenabled, speed);
		bluetooth->SendStr(msg);
		writ->WriteString(msg);
		////**Delete the delay when writ has shown sth
		//System::DelayMs(500);
		break;
	default:
	;

	}
	return true;
}


void print_ccd(uint16_t color){
	for(uint16_t i=0; i<Tsl1401cl::kSensorW; i++){
		lcd->SetRegion(Lcd::Rect(i, (255-Data[i])/2.0 ,1,1));
		lcd->FillPixel(&color,1);
	}
	//lcd->SetRegion(Lcd::Rect(0,0,128,160));
	//lcd->FillPixel(&Lcd::kPurple,10);
}

void print_info(uint16_t color2){
	for (uint16_t i = 0; i<128; i++){
		for (uint16_t j=0; j<255/2; j++){
			//lcd->WriteString("haha");
			//lcd->SetRegion(Lcd::Rect(i,))
		}
	}
}


void check_joystick(void){
	if(!ispressed){
		console->Clear(1);
		//jdown, jleft, jright, jcenter, jup;
		if (!jdown.Get()){
			console->WriteString("joystick down");
			//bluetooth->SendStrLiteral("joystick down \n");
			//lcd->SetRegion(Lcd::Rect(0,0,128,50));
			//lcd->FillColor(Lcd::kYellow);
			ispressed = true;
		}
		else if (!jleft.Get()){
			console->WriteString("joystick left");
			//bluetooth->SendStrLiteral("joystick left \n");
			ispressed = true;
		}
		else if (!jright.Get()){
			console->WriteString("joystick right");
			//bluetooth->SendStrLiteral("joystick right \n");
			ispressed = true;
		}
		else if (!jcenter.Get()){
			console->WriteString("joystick center");
			//bluetooth->SendStrLiteral("joystick center \n");
			ispressed = true;
		}
		else if (!jup.Get()){
			console->WriteString("joystick up");
			//bluetooth->SendStrLiteral("joystick up \n");
			ispressed = true;
		}
	}else{
		if(jdown.Get() && jleft.Get() && jright.Get() && jcenter.Get() && jup.Get()){
			ispressed = false;
		}
	}
}

void checkangle(void){
	angle = (angle < 500) ? 500 : (angle > 1250) ? 1250 : angle;
}

void coppydata(void){
	for(uint16_t i=0; i<Tsl1401cl::kSensorW; i++){
		copdata[i]=Data[i];
	}
}

void ccdbuffer(void){
	prevData=Data;

}


uint16_t otsu(uint16_t *p){
	uint32_t g,max=0;
	uint16_t tt=0, ttlow=0;
	uint8_t u0=0, u1=0, count=0, cnt=0;
	uint16_t tr=0;
	uint8_t pc[256] = {0};
	uint8_t j;
	for(j=5; j<=122; j++){
		pc[*(p+j)]++;
		tt+=*(p+j);
	}
	for(j=0; j<=254; j++){
		cnt=pc[j];
		if(cnt==0) continue;
		count+=pc[j];
		ttlow+=cnt*j;
		u0=ttlow/count;
		u1=(tt-ttlow)/(118-count);
		g=((uint32_t)(u0-u1)*(u0-u1))*((count*(118-count)))/16384;
		if(g>max){
			max=g;
			tr=j;
		}
		if(count>=118) break;
	}
	return tr;
}


void ledge(void){
	lflag=0;
	errl[0]=0;
	int j,k,l,m;
	char edg[30];
	for(uint16_t i=0; i<Tsl1401cl::kSensorW/2; i++){
		if ((Data[i] + 4)<= Data[i+1]){
			if((Data[i+1]+4) <= Data[i+2]){
				if((Data[i+2]+4) <= Data[i+3]){
					coppydata();
					uint16_t mline = otsu(copdata);
					if(Data[i+3]-Data[i]>= mline){
						lflag = true;
					}
				}
			}
		}
		errl[1]=errl[0];
		j=i+4;
		xl[1]=j;
		if((Data[j]-Data[i])>errl[0]){
			errl[0] = Data[j]-Data[i];
			xl[0]=j-2;
		}
	}
	if(lflag==true){
		lline=xl[0];
	}

}

void redge(void){
	errr[0]=0;
	char edg[30];
	for(uint16_t i= Tsl1401cl::kSensorW/2; i<Tsl1401cl::kSensorW; i++){
		int j = i+4;
		errr[1]=errr[0];
		xr[1]=j;
		if((Data[j]-Data[i])>errr[0]){
			errr[0] = Data[j]-Data[i];
			xr[0]=j+2;
		}
	}

}

void detmid(void){
	char mil[30];
	mid=(xr[0]+xl[0])/2;
	//sprintf(mil, "%d", mid);
	//console->WriteString(mil);
}

double calculate_error(void){
	uint16_t ccd_min_val, ccd_max_val;

	ccd_min_val = ccd_max_val = Data[0];

	// Find out the upper and lower boundary of current batch of the signal.
	for(int i = 1; i < Tsl1401cl::kSensorW; i++) {
		if(Data[i] > ccd_max_val)
			ccd_max_val = Data[i];
		if(Data[i] < ccd_min_val)
			ccd_min_val = Data[i];
	}

	uint16_t threshold = (ccd_min_val + ccd_max_val) / 2;

	int left_pos = -1, right_pos = -1;
	bool state = (Data[0] < threshold);

	for(int i = 1; i < Tsl1401cl::kSensorW; i++) {
		if(state ^ (Data[i] < threshold)) {
			// State change.
			// Note: Record the first change as left side,
			//        while the last change as right side.
			if(left_pos == -1)
				left_pos = i;
			else
				right_pos = i;
		}
	}
	return (left_pos + right_pos) / 2.0;
}


void PID(int16_t target_val, int16_t curr_val){
	//Error
	int16_t curr_err_val = target_val - curr_val;

	//Proportional
	int16_t p_out = kp * error;

	//Integral
	integra_val += curr_err_val * dt;
	int16_t i_out = ki * su;

	//Derivative
	int16_t derivative = (curr_err_val - prev_err_val) / dt;
	int16_t d_out = kd * dervivative;

	//Calculate total output
	int16_t output = p_out + i_out + d_out;

	//Check power
	changespeed(output);

	kp=0.6;
	ki=0.2;

	uint16_t turn = angle;

	//preverror=
	if(mid>64){
		turn = 900 + (uint16_t)(kp*error + ki*sumerror + kd*(preverror-error));
	}else{
		turn = 900 - (uint16_t)(kp*error + ki*sumerror + kd*(preverror-error));
	}
	angle = turn;
	checkangle();
	servo->SetDegree(angle);
	//sprintf(ang, "%d", angle);
	//console->WriteString("turn");
	//console->WriteString(ang);
}

void region_set(libsc::Lcd::Rect &ha,int x, uint16_t y,int w,int h){
	ha.x = x;
	ha.y= y;
	ha.w = w;
	ha.h = h;
}

void turn(void){
	if(Data[64]>90){
		servo->SetDegree(mid_angle);
	}
	if(Data[30]<80){
		angle = 1200;
		servo->SetDegree(angle);
	}
	else if(Data[100]<80){
		angle = 800;
		servo->SetDegree(angle);
	}
}

void speedPID(void){

}

void changespeed(uint16_t speed){
	speed = (speed > 400) ? 400: speed;
	if (motorenabled){
		altmotor->SetPower(speed);
	}
}

void configuration(void){

	//CCD
	ccd_up = new Tsl1401cl(0);

	//BUZZER
	Gpo::Config buzzer_config;
	buzzer_config.pin = Pin::Name::kPta8;
	buzzer_config.is_high = false;
	buzzer = Gpo(buzzer_config);

	//JOYSTICK
	Gpi::Config joystick_config;
	joystick_config.pin = Pin::Name::kPtc4;
	jdown = Gpi(joystick_config);
	joystick_config.pin = Pin::Name::kPtc5;
	jleft = Gpi(joystick_config);
	joystick_config.pin = Pin::Name::kPtc6;
	jright = Gpi(joystick_config);
	joystick_config.pin = Pin::Name::kPtc7;
	jcenter = Gpi(joystick_config);
	joystick_config.pin = Pin::Name::kPtc8;
	jup = Gpi(joystick_config);

	//LCD
	St7735r::Config config;
	config.is_revert = true;
	config.is_bgr = false;
	lcd = new St7735r(config);
	//lcd->SetRegion(Lcd::Rect(0,0,128,160));
	lcd->FillColor(Lcd::kBlack);

	////LCD CONSOLE
	libsc::LcdConsole::Config tryy;
	tryy.bg_color = 0;
	tryy.lcd = lcd;
	tryy.text_color = 0xFFFF;
	//Lcd *ylcd;
	tryy.lcd = lcd;
	console = new LcdConsole(tryy);

	//LcdConsole console(tryy);		//this one s not pointer

	////LCD TYPEWRITER
	LcdTypewriter::Config writert;
	//writert.text_color = WHITE;
	writert.lcd= lcd;
	writert.is_text_wrap = true;
	writ = new LcdTypewriter(writert);
	//LcdTypewriter type(writert);

	//SERVO
	FutabaS3010::Config serv_config;
	serv_config.id=0;
	servo = new FutabaS3010(serv_config);
	servo->SetDegree(mid_angle);

	//ALTMOTOR
	AlternateMotor::Config altmotor_config;
	altmotor_config.id = 1;
	altmotor = new AlternateMotor(altmotor_config);
	altmotor->SetClockwise(false);
	speed=0;
	altmotor->SetPower(speed);
	motorenabled = false;

	/*
	//BLUETOOTH
	libsc::k60::JyMcuBt106::Config bluetooth_config;
	bluetooth_config.id = 0;
	bluetooth_config.baud_rate = Uart::Config::BaudRate::k115200;
	bluetooth_config.tx_buf_size = 200;
	bluetooth_config.rx_isr = listener;
	bluetooth = new libsc::k60::JyMcuBt106(bluetooth_config);
	*/
}
