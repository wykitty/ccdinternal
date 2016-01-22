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
FutabaS3010 *servo = 0;
Tsl1401cl *ccd_up = 0;
Tsl1401cl *ccd_down = 0;
libsc::k60::JyMcuBt106 *bluetooth = 0;
uint16_t mid_angle=1050; //[600,1500]
uint16_t angle =1050;;
Motor *mot=0;
AlternateMotor *altmotor =0;
std::array<uint16_t, Tsl1401cl::kSensorW> Data;
std::array<uint16_t, Tsl1401cl::kSensorW> preprepreData;
std::array<uint16_t, Tsl1401cl::kSensorW> prepreData;
std::array<uint16_t, Tsl1401cl::kSensorW> prevData;
uint16_t ccd_count;
uint16_t speed;
uint16_t color;
uint16_t color2 = 0xFFE0;
uint16_t feedback, preError;
int16_t error;
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
void check_joystick(void);
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
	libsc::Lcd::Rect ha;
	ccd_count=0;

	if(angle!=1300){
		angle=1300;
		servo->SetDegree(angle);
	}


	while(1){
		if(t != System::Time()){
			t=System::Time();
			//lcd->Clear();
			//sprintf(buffer, "%d",System::Time());
			if (t%10 == 0){
				check_joystick();
				//**check if need inline here
			}

			if (t%20==0){
				lcd->Clear();
				ccd_up->StartSample();
				while(!ccd_up->SampleProcess()){};
				Data = ccd_up->GetData();
				//turn();
//				if (ccd_count<10){
//					ccd_count+=1;
//				}
				sprintf(ccd_buf, "%d",Data[64]);
				//print_ccd(Lcd::kGreen);
				for(uint16_t i=0; i<Tsl1401cl::kSensorW; i++){
					lcd->SetRegion(Lcd::Rect(i, (255-Data[i])/2,1,1));
					lcd->FillPixel(&color,1);
				}
				//prevData=Data;
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
			}
			if(t%1000 == 0){
				leda.Switch();
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
		System::DelayMs(500);
		break;
	default:
	;

	}
	return true;
}


inline void print_ccd(uint16_t color){
	for(uint16_t i=0; i<Tsl1401cl::kSensorW; i++){
		lcd->SetRegion(Lcd::Rect(i, (255-Data[i])/2,1,1));
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
		//jdown, jleft, jright, jcenter, jup;
		if (!jdown.Get()){
			console->WriteString("joystick down");
			//bluetooth->SendStrLiteral("joystick down \n");
			lcd->SetRegion(Lcd::Rect(0,0,128,50));
			lcd->FillColor(Lcd::kYellow);
			ispressed = true;
		}
		else if (!jleft.Get()){
			writ->WriteString("joystick left");
			bluetooth->SendStrLiteral("joystick left \n");
			ispressed = true;
		}
		else if (!jright.Get()){
			writ->WriteString("joystick right");
			bluetooth->SendStrLiteral("joystick right \n");
			ispressed = true;
		}
		else if (!jcenter.Get()){
			writ->WriteString("joystick center");
			bluetooth->SendStrLiteral("joystick center \n");
			ispressed = true;
		}
		else if (!jup.Get()){
			writ->WriteString("joystick up");
			bluetooth->SendStrLiteral("joystick up \n");
			ispressed = true;
		}
	}else{
		if(jdown.Get() && jleft.Get() && jright.Get() && jcenter.Get() && jup.Get()){
			ispressed = false;
		}
	}
}

void ledge(void){

}

void redge(void){

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
	lcd->SetRegion(Lcd::Rect(0,0,128,160));
	lcd->FillColor(Lcd::kBlack);

	////LCD CONSOLE
	libsc::LcdConsole::Config tryy;
	for(uint16_t i=0; i<128; i++){
		tryy.region = Lcd::Rect(i,80,1,1);
		//tryy.region(Lcd::Rect(i,80,1,1));
	}
	tryy.bg_color = 0;
	tryy.lcd = lcd;
	tryy.text_color = -1;
	//Lcd *ylcd;
	tryy.lcd = lcd;
	console = new LcdConsole(tryy);

	//LcdConsole console(tryy);		//this one s not pointer

	////LCD TYPEWRITER
	LcdTypewriter::Config writert;
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

	//BLUETOOTH
	libsc::k60::JyMcuBt106::Config bluetooth_config;
	bluetooth_config.id = 0;
	bluetooth_config.baud_rate = Uart::Config::BaudRate::k115200;
	bluetooth_config.tx_buf_size = 200;
	bluetooth_config.rx_isr = listener;
	bluetooth = new libsc::k60::JyMcuBt106(bluetooth_config);
}

