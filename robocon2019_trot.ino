/*****************************************
	
	！　ピンの番号が一致しているか要確認　！
	
*****************************************/



/*****************************************
	include
*****************************************/
#include "Arduino.h"
#include "math.h"
#include "MsTimerTPU3.h"
#include "AMT203V.h"
#include "PIDclass.h"
#include "legClass.h"
#include "mySaberClass.h"
#include "SoftwareSerial.h"
#include "Sabertooth.h"
#include "phaseCounter.h"
#include "reboot.h"
#include "VL53L0X.h"
#include "Wire.h"

/*****************************************
	define
*****************************************/
#define INT_TIME	(0.01)
#define PAIR1	(1)
#define PAIR2	(2)
#define LINEAR_CMD	(1)
#define ANGLE_CMD	(2)
// #define PI		(3.14159)

//#define PITCH	(3) // mmだと3，mだと0.003
//#define HASU	(25)
#define HANKEI	(0.015)
#define PITCH_HASU	(0.075) // ピッチ(m) * 歯数
#define ENC_RES	(1000)
// AMT203用
#define PIN_15	(33)
#define PIN_51	(25)
#define PIN_13	(31)
#define PIN_12	(30)
#define PIN_FLSW	(46)//(A2)//
#define PIN_BLSW	(28)//(27)//
#define PIN_FRSW	(27)//(A5)//
#define PIN_BRSW	(47)//(A4)//
#define PIN_START	(29)//(28)//
#define PIN_ASHISAKI_FL	(44)
#define PIN_ASHISAKI_FR	(45)
#define PIN_GEREGE	(1)

// mode
#define STATE_INIT_1ST	(0)
#define STATE_INIT_2ND	(1)

// height_changeTimingで使用
#define CHANGE_HEIGHT	(0.1)
#define CHANGE_HEIGHT_SHORT	(0.095)
#define CHANGE_HEIGHT_VERYSHORT	(0.03)
#define MAE		(0)
#define ATO		(1)

// 左右のみ
#define LEFT	(0)
#define RIGHT	(1)

// 前後
#define FRONT	(0)
#define BACK	(1)

// 四脚すべて
#define FL		(0)
#define BL		(1)
#define FR		(2)
#define BR		(3)

//----------------------------------------
//	距離センサ
//----------------------------------------

/* #define SENSOR_NUM  2 // 使用するセンサーの数
#define ADDRESS_DEFALUT 0b0101001 // 0x29
#define ADDRESS_00 (ADDRESS_DEFALUT + 2) */

/*****************************************
	class
*****************************************/
legClass frontLeft;
legClass BackLeft;
legClass frontRight;
legClass BackRight;


phaseCounter FLlinearEnc(1);
phaseCounter BLlinearEnc(4);
phaseCounter FRlinearEnc(3);
phaseCounter BRlinearEnc(2);

AMT203V FLangleEnc(&SPI, PIN_15);
AMT203V BLangleEnc(&SPI, PIN_51);
AMT203V FRangleEnc(&SPI, PIN_13);
AMT203V BRangleEnc(&SPI, PIN_12);

PID FLLichiPID(24.0, 0.0, 0.0, INT_TIME);
PID FLAichiPID(15.0, 0.0, 0.0, INT_TIME);
PID FLLsokudoPID(32.0, 0.0, 60.0, INT_TIME);
PID FLAsokudoPID(2.5, 0.0, 4.0, INT_TIME);

PID BLLichiPID(23.0, 0.0, 0.0, INT_TIME);
PID BLAichiPID(12.0, 0.0, 0.0, INT_TIME);
PID BLLsokudoPID(30.0, 0.0, 60.0, INT_TIME);
PID BLAsokudoPID(2.0, 0.0, 0.5, INT_TIME);

PID FRLichiPID(24.0, 0.0, 0.0, INT_TIME);
PID FRAichiPID(12.0, 0.0, 0.0, INT_TIME);
PID FRLsokudoPID(30.0, 0.0, 60.0, INT_TIME);
PID FRAsokudoPID(2.0, 0.0, 4.0, INT_TIME);

PID BRLichiPID(24.0, 0.0, 0.0, INT_TIME);
PID BRAichiPID(15.0, 0.0, 0.0, INT_TIME);
PID BRLsokudoPID(30.0, 0.0, 60.0, INT_TIME);
PID BRAsokudoPID(2.5, 0.0, 4.0, INT_TIME);

SoftwareSerial SWSerial1(26, 24);// rx, tx
Sabertooth ST1(129, SWSerial1);//FL
Sabertooth ST2(130, SWSerial1);//BL
Sabertooth ST3(128, SWSerial1);//FR
Sabertooth ST4(131, SWSerial1);//BR

mySaberClass saber1(&ST1);
mySaberClass saber2(&ST2);
mySaberClass saber3(&ST3);
mySaberClass saber4(&ST4);

/*****************************************
	宣言
*****************************************/
double kataSokudo = 0.2;//0.25;//0.2;//2;
double kata_sokudo[10 + 1] = { 0.2, 0.16, 0.06, 0.08, 0.1, 0.1, 0.1, 0.07, 0.1, 0.1 };//{ 0.2, 0.18, 0.14, 0.1, 0.15 };//{ 0.2, 0.2, 0.2, 0.2, 0.2 };//{ 0.2, 0.2, 0.2, 0.2, 0.2 };//{ 0.25, 0.23, 0.26, 0.23, 0.27 };

double stroke[10 + 1][2] = { // left, right
	/* 0 */{ 0.2, 0.2 },//{ 0.18, 0.20 }, 
	/* 1 */{ 0.1, 0.23 },//{ 0.16, 0.16 },//{ 0.16, 0.23 }, 
	/* 2 */{ 0.08, 0.08 },//{ 0.12, 0.12 },//{ 0.1, 0.1 },//{ 0.13, 0.13 }, 
	/* 3 */{ 0.085, 0.085 },//{ 0.11, 0.11 }, 
	/* 4 */{ 0.115, 0.115 },//{ 0.2, 0.2 }
	/* 5 */{ 0.1, 0.23 },//{ 0.07, 0.15 },
	/* 6 */{ 0.2, 0.0 },//{ 0.14, 0.0 },//{ 0.12, 0.0 },
	/* 7 */{ 0.08, 0.08 },
	/* 8 */{ 0.15, 0.15 },
	/* 9 */{ 0.15, 0.15 }
};

double x_height[4] = { 0.32, 0.32, 0.32, 0.32 };//{ 0.33, 0.33, 0.33, 0.33 };//{ 0.34, 0.34, 0.34, 0.34 };// FL, BL, FR, BR

/* double x_height[4 + 1][4] = { // FL, BL, FR, BR
	{ 0.33, 0.33, 0.33, 0.33 },//{ 0.33, 0.33 },
	{ 0.33, 0.33, 0.33, 0.33 },//{ 0.33, 0.33 },
	{ 0.33, 0.33, 0.33, 0.33 },//{ 0.33, 0.33 },
	{ 0.33, 0.33, 0.33, 0.33 },//{ 0.33, 0.33 },
	{ 0.33, 0.33, 0.33, 0.33 }//{ 0.33, 0.33 }
}; */

double ex_height = 0.0;

double FLheight_changeTiming[2] = { 30.0, 40.0 };	// change前, change後

double BLheight_changeTiming[2] = { 30.0, 40.0 };	// change前, change後

double FRheight_changeTiming[2] = { 30.0, 40.0 };	// change前, change後

double BRheight_changeTiming[2] = { 30.0, 40.0 };	// change前, change後

double shift[10 + 1][4] = { // FL, BL, FR, BR
	{0.00, 0.03, 0.00, 0.03},
	{0.00, 0.03, 0.00, 0.03},
	{-0.02, 0.03, -0.02, 0.03},
	{-0.02, 0.03, -0.02, 0.03},
	{-0.02, 0.03, -0.02, 0.03},
	{-0.02, 0.03, -0.02, 0.03},
	{-0.02, 0.03, -0.02, 0.03},
	{-0.02, 0.03, -0.02, 0.03},
	{-0.02, 0.03, -0.02, 0.03},
	{-0.02, 0.03, -0.02, 0.03},
};

double height[4] = { 0.03, 0.03, 0.03, 0.03 };//{ 0.03, 0.03 };//{ 0.03, 0.03 };// front, back

/* double height[4 + 1][2] = { // front, back
	{ 0.1, 0.1 },
	{ 0.05, 0.05 },
	{ 0.05, 0.05 },
	{ 0.05, 0.05 },
	{ 0.05, 0.05 }
}; */

double TmpStroke[4] = { stroke[0][LEFT], stroke[0][LEFT], stroke[0][RIGHT], stroke[0][RIGHT] };
double TmpShift[4] = { shift[0][FL], shift[0][BL], shift[0][FR], shift[0][BR] };

double kataSokudo_FL = 2.0 * kataSokudo * stroke[0][LEFT] / ( stroke[0][RIGHT] + stroke[0][LEFT] );//Y_left / ( Y_right + Y_left );
double kataSokudo_BL = 2.0 * kataSokudo * stroke[0][LEFT] / ( stroke[0][RIGHT] + stroke[0][LEFT] );
double kataSokudo_FR = 2.0 * kataSokudo * stroke[0][RIGHT] / ( stroke[0][LEFT] + stroke[0][RIGHT] );//Y_right / ( Y_left + Y_right );
double kataSokudo_BR = 2.0 * kataSokudo * stroke[0][RIGHT] / ( stroke[0][LEFT] + stroke[0][RIGHT] );

#define X_BACK_CONST	(0.4)

double constNum = 2.0 * kataSokudo * INT_TIME / ( stroke[0][RIGHT] + stroke[0][LEFT] );

double FLlinear = 0.0, FLangle = 0.0, preFLlinear = 0.0, preFLangle = 0.0, refFLlinear = 0.0, refFLangle = 0.0;
double FLLsokudo = 0.0, refFLLsokudo = 0.0, FLAsokudo = 0.0, refFLAsokudo = 0.0, preFLAsokudo = 0.0;
int FLAenc = 0, preFLAenc = 0, FLLenc = 0, preFLLenc = 0;
double FLLcmd = 0.0, FLAcmd = 0.0;

double BLlinear = 0.0, BLangle = 0.0, preBLlinear = 0.0, preBLangle = 0.0, refBLlinear = 0.0, refBLangle = 0.0;
double BLLsokudo = 0.0, refBLLsokudo = 0.0, BLAsokudo = 0.0, refBLAsokudo = 0.0, preBLAsokudo = 0.0;
int BLAenc = 0, preBLAenc = 0, BLLenc = 0, preBLLenc = 0;
double BLLcmd = 0.0, BLAcmd = 0.0;

double FRlinear = 0.0, FRangle = 0.0, preFRlinear = 0.0, preFRangle = 0.0, refFRlinear = 0.0, refFRangle = 0.0;
double FRLsokudo = 0.0, refFRLsokudo = 0.0, FRAsokudo = 0.0, refFRAsokudo = 0.0, preFRAsokudo = 0.0;
int FRAenc = 0, preFRAenc = 0, FRLenc = 0, preFRLenc = 0;
double FRLcmd = 0.0, FRAcmd = 0.0;

double BRlinear = 0.0, BRangle = 0.0, preBRlinear = 0.0, preBRangle = 0.0, refBRlinear = 0.0, refBRangle = 0.0;
double BRLsokudo = 0.0, refBRLsokudo = 0.0, BRAsokudo = 0.0, refBRAsokudo = 0.0, preBRAsokudo = 0.0;
int BRAenc = 0, preBRAenc = 0, BRLenc = 0, preBRLenc = 0;
double BRLcmd = 0.0, BRAcmd = 0.0;

double addFL = 0.0, addBL = 0.0, addFR = 0.0, addBR = 0.0;

double incrNum = 0.0;

// まっすぐから左旋回の時間， 左旋回からまっすぐに戻るまでの時間， まっすぐから右旋回までの時間， 右旋回からまっすぐの時間
double change_timing[10] = { 4.5, 8.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0, 900.0, 1000.0  };//{4.5, 8.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0, 900.0, 1000.0 };//{4.0, 7.5, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0, 900.0, 1000.0 };

int count = 0; // 時間計測するために使ってるだけ
int wait_count = 0;
double  step_count = 0.0;
int amt203_count = 0;

int ledcount = 0;

int mode = 0;

int n = 0;

int height_count = 0;

int kari = 0;

// 距離センサ
/* const int VL53L0X_GPIO[SENSOR_NUM] = {A2, A3};
VL53L0X gSensor[SENSOR_NUM]; // 使用するセンサークラス配列
double sensVal_left[4] = { 0.0, 0.0, 0.0, 0.0};
double sensVal_right[4] = { 0.0, 0.0, 0.0, 0.0};
unsigned char sensVal_count = 0;
unsigned char sensVal_leftCount = 0;
unsigned char sensVal_rightCount = 0; */

// FLA:FLL / BLA:BLL / FRA:FRL / BRA:BRL
byte init_1st = B00000000;//B11111111;//
byte init_2nd = B00000000;//B11111111;//

boolean a = false;
boolean change_shift_flag = false;
boolean change_height_flag = false;

boolean changeStroke_flag_FL = false;
boolean changeStroke_flag_BL = false;
boolean changeStroke_flag_FR = false;
boolean changeStroke_flag_BR = false;

boolean halfStep = false;
boolean pre_halfStep = halfStep;

boolean sakyuMode = false;
boolean sakyuMode_left = false;
boolean sakyuMode_right = false;

boolean FLheighet_flag = false;
boolean BLheighet_flag = false;
boolean FRheighet_flag = false;
boolean BRheighet_flag = false;

double height_long_timing = 100.0;

boolean rope_FL = false;
boolean rope_BL = false;
boolean rope_FR = false;
boolean rope_BR = false;

double stepRope_FL = 0.0;
double stepRope_BL = 0.0;
double stepRope_FR = 0.0;
double stepRope_BR = 0.0;


void timerWarikomi_10ms() {
	if(ledcount >= 25) {
		if(digitalRead(PIN_LED0) == LOW){
			digitalWrite(PIN_LED0, HIGH);
		} else {
			digitalWrite(PIN_LED0, LOW);
		}
		ledcount = 0;
	}
	ledcount++;
	
	// 直動のカウント取得
	FLLenc = FLlinearEnc.getCount();
	FLAenc = FLangleEnc.getEncount();
	BLLenc = BLlinearEnc.getCount();
	BLAenc = BLangleEnc.getEncount();
	FRLenc = FRlinearEnc.getCount();
	FRAenc = FRangleEnc.getEncount();
	BRLenc = BRlinearEnc.getCount();
	BRAenc = BRangleEnc.getEncount();
	if( FLAenc == -1 ){
		amt203_count++;
		FLAenc = preFLAenc;
	}
	if( BLAenc == -1 ){
		amt203_count++;
		BLAenc = preBLAenc;
	}
	if( FRAenc == -1 ){
		amt203_count++;
		FRAenc = preFRAenc;
	}
	if( BRAenc == -1 ){
		amt203_count++;
		BRAenc = preBRAenc;
	}
	preFLAenc = FLAenc;
	preBLAenc = BLAenc;
	preFRAenc = FRAenc;
	preBRAenc = BRAenc;
	
	// ベルトのピッチ * プーリーの歯数 * カウント / (エンコーダの分解能 * 4)
	// 長さ = (double)Encount * 2 * PI * 半径 / (4逓倍 * 分解能);
	FLlinear = ( -FLLenc * 2 * PI * HANKEI / ( 4.0 * ENC_RES ) ) + addFL;// + 0.39;
	FLangle = ( -FLAenc * 2.0 * PI / 4096.0 ) * ( 1.0 / 2.0 ) + ( PI / 2 );
	BLlinear = ( -BLLenc * 2 * PI * HANKEI / ( 4.0 * ENC_RES ) ) + addBL;// + 0.39;
	BLangle = ( -BLAenc * 2.0 * PI / 4096.0 ) * ( 1.0 / 2.0 ) + ( PI / 2 );
	FRlinear = ( FRLenc * 2 * PI * HANKEI / ( 4.0 * ENC_RES ) ) + addFR;// + 0.39;
	FRangle = -(( -FRAenc * 2.0 * PI / 4096.0 ) * ( 1.0 / 2.0 ) + ( PI / 2 ));
	BRlinear = ( -BRLenc * 2 * PI * HANKEI / ( 4.0 * ENC_RES ) ) + addBR;// + 0.39;
	BRangle = -(( -BRAenc * 2.0 * PI / 4096.0 ) * ( 1.0 / 2.0 ) + ( PI / 2 ));

	if( !a ){
		preFLangle = FLangle;
		preBLangle = BLangle;
		preFRangle = FRangle;
		preBRangle = BRangle;
		a = true;
	}
	
	/* double predict_FLangle = preFLangle + FLAsokudo * INT_TIME;
	
	if( fabs( FLangle - predict_FLangle ) > 0.1 ){
		FLangle = predict_FLangle;
	} */
	
	
	//-----------------------------------------------------------------
	//linear
	FLLsokudo = ( FLlinear - preFLlinear ) / (INT_TIME);
	BLLsokudo = ( BLlinear - preBLlinear ) / (INT_TIME);
	FRLsokudo = ( FRlinear - preFRlinear ) / (INT_TIME);
	BRLsokudo = ( BRlinear - preBRlinear ) / (INT_TIME);
	
	preFLlinear = FLlinear;
	preBLlinear = BLlinear;
	preFRlinear = FRlinear;
	preBRlinear = BRlinear;
	//-----------------------------------------------------------------
	//-----------------------------------------------------------------
	//angle
	FLAsokudo = ( FLangle - preFLangle ) / (INT_TIME);
	if( fabs( FLAsokudo ) > 10.0 ){
		FLAsokudo = preFLAsokudo;
	}
	preFLAsokudo = FLAsokudo;
	BLAsokudo = ( BLangle - preBLangle ) / (INT_TIME);
	if( fabs( BLAsokudo ) > 10.0 ){
		BLAsokudo = preBLAsokudo;
	}
	preBLAsokudo = BLAsokudo;
	FRAsokudo = ( FRangle - preFRangle ) / (INT_TIME);
	if( fabs( FRAsokudo ) > 10.0 ){
		FRAsokudo = preFRAsokudo;
	}
	preFRAsokudo = FRAsokudo;
	BRAsokudo = ( BRangle - preBRangle ) / (INT_TIME);
	if( fabs( BRAsokudo ) > 10.0 ){
		BRAsokudo = preBRAsokudo;
	}
	preBRAsokudo = BRAsokudo;
	
	preFLangle = FLangle;
	preBLangle = BLangle;
	preFRangle = FRangle;
	preBRangle = BRangle;
	//-----------------------------------------------------------------
	
	if( mode == STATE_INIT_1ST ){
		int readPinFL = digitalRead(PIN_FLSW);
		int readPinBL = digitalRead(PIN_BLSW);
		int readPinFR = digitalRead(PIN_FRSW);
		int readPinBR = digitalRead(PIN_BRSW);
		
		if( ( init_1st & B10000000 ) == 0 ){
			refFLangle = 0.0;
			
			if( fabs( refFLangle - FLangle ) <= 0.03 ){
				init_1st |= B10000000;
			}
			
			if( refFLangle > FLangle ){
				saber1.saberCmd(ANGLE_CMD, -0.06);
			}else if( refFLangle < FLangle ){
				saber1.saberCmd(ANGLE_CMD, 0.06);
			}
		}else{
			saber1.saberCmd(ANGLE_CMD, 0.0);
		}
		
		if( ( ( init_1st & B01000000 ) == 0 ) && readPinFL == 0 ){
			addFL = 0.456 - FLlinear;//0.5030 - FLlinear;
			saber1.saberCmd(LINEAR_CMD, 0.0);
			init_1st |= B01000000;
		}else if( ( init_1st & B01000000 ) == 64 ){
			saber1.saberCmd(LINEAR_CMD, 0.0);
		}else{
			saber1.saberCmd(LINEAR_CMD, -0.1);
		}
		
		if( ( init_1st & B00100000 ) == 0 ){
			refBLangle = 0.0;
			
			if( fabs( refBLangle - BLangle ) <= 0.03 ){
				init_1st |= B00100000;
			}
			
			if( refBLangle > BLangle ){
				saber2.saberCmd(ANGLE_CMD, -0.06);
			}else if( refBLangle < BLangle ){
				saber2.saberCmd(ANGLE_CMD, 0.06);
			}
		}else{
			saber2.saberCmd(ANGLE_CMD, 0.0);
		}
		
		if( ( ( init_1st & B00010000 ) == 0 ) &&  readPinBL == 0 ){
			addBL = 0.45 - BLlinear;//0.44 - BLlinear;//0.5070 - BLlinear;
			saber2.saberCmd(LINEAR_CMD, 0.0);
			init_1st |= B00010000;
		}else if( ( init_1st & B00010000 ) == 16 ){
			saber2.saberCmd(LINEAR_CMD, 0.0);
		}else{
			saber2.saberCmd(LINEAR_CMD, 0.1);
		}
		
		if( ( init_1st & B00001000 ) == 0 ){
			refFRangle = 0.0;
			
			if( fabs( refFRangle - FRangle ) <= 0.03){
				init_1st |= B00001000;
			}
			
			if( refFRangle > FRangle ){
				saber3.saberCmd(ANGLE_CMD, 0.06);
			}else if( refFRangle < FRangle ){
				saber3.saberCmd(ANGLE_CMD, -0.06);
			}
		}else{
			saber3.saberCmd(ANGLE_CMD, 0.0);
		}
		
		if( ( ( init_1st & B00000100 ) == 0 ) &&  readPinFR == 0 ){
			addFR = 0.45 - FRlinear;//0.44 - FRlinear;//0.5100 - FRlinear;
			saber3.saberCmd(LINEAR_CMD, 0.0);
			init_1st |= B00000100;
		}else if( ( init_1st & B00000100 ) == 4 ){
			saber3.saberCmd(LINEAR_CMD, 0.0);
		}else{
			saber3.saberCmd(LINEAR_CMD, 0.1);
		}
		
		if( ( init_1st & B00000010 ) == 0 ){
			refBRangle = 0.0;
			
			if( fabs( refBRangle - BRangle ) <= 0.03 ){
				init_1st |= B00000010;
			}
			
			if( refBRangle > BRangle ){
				saber4.saberCmd(ANGLE_CMD, 0.06);
			}else if( refBRangle < BRangle ){
				saber4.saberCmd(ANGLE_CMD, -0.06);
			}
		}else{
			saber4.saberCmd(ANGLE_CMD, 0.0);
		}
		
		if( ( ( init_1st & B00000001 ) == 0 ) &&  readPinBR == 0 ){
			addBR = 0.46 - BRlinear;//0.47 - BRlinear;//0.5110 - BRlinear;
			saber4.saberCmd(LINEAR_CMD, 0.0);
			init_1st |= B00000001;
		}else if( ( init_1st & B00000001 ) == 1 ){
			saber4.saberCmd(LINEAR_CMD, 0.0);
		}else{
			saber4.saberCmd(LINEAR_CMD, -0.1);
		}
		
		if( init_1st == B11111111 ){
			mode = STATE_INIT_2ND;
		}
		
		Serial.print(init_1st, BIN);
		Serial.print("\t");
		Serial.print(FLlinear, 4);
		Serial.print("\t");
		Serial.print(BLlinear, 4);
		Serial.print("\t");
		Serial.print(FRlinear, 4);
		Serial.print("\t");
		Serial.println(BRlinear, 4);
	}else if( mode == STATE_INIT_2ND ){
		
		// angle init
		if( ( init_2nd & B10000000 ) == 0 ){
			refFLangle = frontLeft.calcAngle( frontLeft.calcSayu( incrNum ), frontLeft.calcJoge( incrNum ) );
			
			if( fabs( refFLangle - FLangle ) <= 0.03 ){
				init_2nd |= B10000000;
			}
			
			if( refFLangle > FLangle ){
				saber1.saberCmd(ANGLE_CMD, -0.06);
			}else if( refFLangle < FLangle ){
				saber1.saberCmd(ANGLE_CMD, 0.06);
			}
			
		}else{
			saber1.saberCmd(ANGLE_CMD, 0.0);
		}
	
		//delayMicroseconds(50);
	
		// linear init
		if( ( init_2nd & B01000000 ) == 0 ){
			refFLlinear = frontLeft.calcLinear( frontLeft.calcSayu( incrNum ), frontLeft.calcJoge( incrNum ) );
			
			if( fabs( refFLlinear - FLlinear ) <= 0.003 ){
				init_2nd |= B01000000;
			}
			
			saber1.saberCmd(LINEAR_CMD, 0.1);
		}else{
			saber1.saberCmd(LINEAR_CMD, 0.0);
		}
		
		//delayMicroseconds(50);
		
		// angle init
		if( ( init_2nd & B00100000 ) == 0 ){
			refBLangle = BackLeft.calcAngle( BackLeft.calcSayu( incrNum ), BackLeft.calcJoge( incrNum ) );
			
			if( fabs( refBLangle - BLangle ) <= 0.03 ){
				init_2nd |= B00100000;
			}
			
			if( refBLangle > BLangle ){
				saber2.saberCmd(ANGLE_CMD, -0.06);// プラス方向に動いた
			}else if( refBLangle < BLangle ){
				saber2.saberCmd(ANGLE_CMD, 0.06);
			}
			
		}else{
			saber2.saberCmd(ANGLE_CMD, 0.0);
		}
	
		//delayMicroseconds(50);
	
	
		// linear init
		if( ( init_2nd & B00010000 ) == 0 ){
			refBLlinear = BackLeft.calcLinear( BackLeft.calcSayu( incrNum ), BackLeft.calcJoge( incrNum ) );
			
			if( fabs( refBLlinear - BLlinear ) <= 0.003 ){
				init_2nd |= B00010000;
			}		
			
			saber2.saberCmd(LINEAR_CMD, -0.1);
		}else{
			saber2.saberCmd(LINEAR_CMD, 0.0);
		}
		
		//delayMicroseconds(50);
		
		// angle init
		if( ( init_2nd & B00001000 ) == 0 ){
			refFRangle = frontRight.calcAngle( frontRight.calcSayu( incrNum ), frontRight.calcJoge( incrNum ) );
			
			if( fabs( refFRangle - FRangle ) <= 0.03){//0.05 ){
				init_2nd |= B00001000;
			}
			
			if( refFRangle > FRangle ){
				saber3.saberCmd(ANGLE_CMD, 0.06);
			}else if( refFRangle < FRangle ){
				saber3.saberCmd(ANGLE_CMD, -0.06);
			}
		}else{
			saber3.saberCmd(ANGLE_CMD, 0.0);
		}
		
		//delayMicroseconds(50);
		
		// linear init
		if( ( init_2nd & B00000100 ) == 0 ){
			refFRlinear = frontRight.calcLinear( frontRight.calcSayu( incrNum ), frontRight.calcJoge( incrNum ) );
			
			if( fabs( refFRlinear - FRlinear ) <= 0.003 ){
				init_2nd |= B00000100;
			}
			
			saber3.saberCmd(LINEAR_CMD, -0.1);
		}else{
			saber3.saberCmd(LINEAR_CMD, 0.0);
		}
		
		//delayMicroseconds(50);
		
		// angle init
		if( ( init_2nd & B00000010 ) == 0 ){
			refBRangle = BackRight.calcAngle( BackRight.calcSayu( incrNum ), BackRight.calcJoge( incrNum ) );
			
			if( fabs( refBRangle - BRangle ) <= 0.03 ){
				init_2nd |= B00000010;
			}
			
			if( refBRangle > BRangle ){
				saber4.saberCmd(ANGLE_CMD, 0.06);// プラス方向に動いた
			}else if( refBRangle < BRangle ){
				saber4.saberCmd(ANGLE_CMD, -0.06);
			}
			
		}else{
			saber4.saberCmd(ANGLE_CMD, 0.0);
		}
	
		//delayMicroseconds(50);
	
		// linear init
		if( ( init_2nd & B00000001 ) == 0 ){
			refBRlinear = BackRight.calcLinear( BackRight.calcSayu( incrNum ), BackRight.calcJoge( incrNum ) );
			
			if( fabs( refBRlinear - BRlinear ) <= 0.003 ){
				init_2nd |= B00000001;
			}
			
			saber4.saberCmd(LINEAR_CMD, 0.1);
		}else{
			saber4.saberCmd(LINEAR_CMD, 0.0);
		}
		
		if( init_2nd == B11111111 ){
			mode = 10;
		}
		
		Serial.print(init_2nd, BIN);
		Serial.print("\t");
		Serial.print(FLlinear, 4);
		Serial.print("\t");
		Serial.print(BLlinear, 4);
		Serial.print("\t");
		Serial.print(FRlinear, 4);
		Serial.print("\t");
		Serial.println(BRlinear, 4);
		
	}else if( mode == 10 ){ // 待ち

		refFLangle = frontLeft.calcAngle( frontLeft.calcSayu( incrNum ), frontLeft.calcJoge( incrNum ) );
		refFLlinear = frontLeft.calcLinear( frontLeft.calcSayu( incrNum ), frontLeft.calcJoge( incrNum ) );
		refBLangle = BackLeft.calcAngle( BackLeft.calcSayu( incrNum ), BackLeft.calcJoge( incrNum ) );
		refBLlinear = BackLeft.calcLinear( BackLeft.calcSayu( incrNum ), BackLeft.calcJoge( incrNum ) );
		refFRangle = frontRight.calcAngle( frontRight.calcSayu( incrNum ), frontRight.calcJoge( incrNum ) );
		refFRlinear = frontRight.calcLinear( frontRight.calcSayu( incrNum ), frontRight.calcJoge( incrNum ) );
		refBRangle = BackRight.calcAngle( BackRight.calcSayu( incrNum ), BackRight.calcJoge( incrNum ) );
		refBRlinear = BackRight.calcLinear( BackRight.calcSayu( incrNum ), BackRight.calcJoge( incrNum ) );

		// 目標速度
		refFLAsokudo = FLAichiPID.getCmd(refFLangle, FLangle, 6.0);
		refFLLsokudo = FLLichiPID.getCmd(refFLlinear, FLlinear, 4.0);
		refBLAsokudo = BLAichiPID.getCmd(refBLangle, BLangle, 6.0);
		refBLLsokudo = BLLichiPID.getCmd(refBLlinear, BLlinear, 4.0);
		refFRAsokudo = FRAichiPID.getCmd(refFRangle, FRangle, 6.0);
		refFRLsokudo = FRLichiPID.getCmd(refFRlinear, FRlinear, 4.0);
		refBRAsokudo = BRAichiPID.getCmd(refBRangle, BRangle, 7.0);
		refBRLsokudo = BRLichiPID.getCmd(refBRlinear, BRlinear, 4.0);//1.0);
		
		// sabertoothに送るコマンド
		FLAcmd += FLAsokudoPID.getCmd(refFLAsokudo, FLAsokudo, 30.0) * ( -1.0 / 127.0 ); // 20より大きくするとcmdの値がいきなり大きく変わる
		FLLcmd += FLLsokudoPID.getCmd(refFLLsokudo, FLLsokudo, 76.0) * ( -1.0 / 127.0 );//127.0) * ( 1.0 / 127.0 );
		BLAcmd += BLAsokudoPID.getCmd(refBLAsokudo, BLAsokudo, 30.0) * ( -1.0 / 127.0 );
		BLLcmd += BLLsokudoPID.getCmd(refBLLsokudo, BLLsokudo, 127.0) * ( 1.0 / 127.0 );
		FRAcmd += FRAsokudoPID.getCmd(refFRAsokudo, FRAsokudo, 30.0) * ( 1.0 / 127.0 );
		FRLcmd += FRLsokudoPID.getCmd(refFRLsokudo, FRLsokudo, 127.0) * ( 1.0 / 127.0 );
		BRAcmd += BRAsokudoPID.getCmd(refBRAsokudo, BRAsokudo, 30.0) * ( 1.0 / 127.0 );
		BRLcmd += BRLsokudoPID.getCmd(refBRLsokudo, BRLsokudo, 127.0) * ( -1.0 / 127.0 );
		
		// コマンドの値が大きくなり過ぎないように制限
		if( FLAcmd < -1.0 ){
			FLAcmd = -1.0;
		}else if( 1.0 < FLAcmd ){
			FLAcmd = 1.0;
		}
		if( FLLcmd < -1.0 ){
			FLLcmd = -1.0;
		}else if( 1.0 < FLLcmd ){
			FLLcmd = 1.0;
		}
		if( BLAcmd < -1.0 ){
			BLAcmd = -1.0;
		}else if( 1.0 < BLAcmd ){
			BLAcmd = 1.0;
		}
		if( BLLcmd < -1.0 ){
			BLLcmd = -1.0;
		}else if( 1.0 < BLLcmd ){
			BLLcmd = 1.0;
		}
		if( FRAcmd < -1.0 ){
			FRAcmd = -1.0;
		}else if( 1.0 < FRAcmd ){
			FRAcmd = 1.0;
		}
		if( FRLcmd < -1.0 ){
			FRLcmd = -1.0;
		}else if( 1.0 < FRLcmd ){
			FRLcmd = 1.0;
		}
		if( BRAcmd < -1.0 ){
			BRAcmd = -1.0;
		}else if( 1.0 < BRAcmd ){
			BRAcmd = 1.0;
		}
		if( BRLcmd < -1.0 ){
			BRLcmd = -1.0;
		}else if( 1.0 < BRLcmd ){
			BRLcmd = 1.0;
		}
		
		saber1.saberCmd(ANGLE_CMD, FLAcmd);
		saber1.saberCmd(LINEAR_CMD, FLLcmd);
		saber2.saberCmd(ANGLE_CMD, BLAcmd);
		saber2.saberCmd(LINEAR_CMD, BLLcmd);
		saber3.saberCmd(ANGLE_CMD, FRAcmd);
		saber3.saberCmd(LINEAR_CMD, FRLcmd);
		saber4.saberCmd(ANGLE_CMD, BRAcmd);
		saber4.saberCmd(LINEAR_CMD, BRLcmd);

		if( !digitalRead(PIN_GEREGE) ){//if( !digitalRead(PIN_START) ){
			mode = 2;
		}
		
		Serial.println("mode = 2");
	}else if( mode == 2 ){
		
		incrNum += constNum;

		if ( !halfStep && incrNum >= 0.5 ) {
			step_count += 0.5;
			halfStep = !halfStep;
			
			/* sensVal_left[ 3 ] = gSensor[0].readRangeSingleMillimeters() * 0.001;
			sensVal_right[ 3 ] = gSensor[1].readRangeSingleMillimeters() * 0.001;
			for( int mk = 0; mk < 4; mk++ ){
				if( sensVal_left[ mk ] < 2.0 ){
					sensVal_leftStroke += sensVal_left[ mk ];
					sensVal_leftCount++;
				}
				if( sensVal_right[ mk ] < 2.0 ){
					sensVal_rightStroke += sensVal_right[ mk ];
					sensVal_rightCount++;
				}
			}
			sensVal_leftStroke = sensVal_leftStroke / (double)( sensVal_leftCount + 1 );
			sensVal_rightStroke = sensVal_rightStroke / (double)( sensVal_rightCount + 1 );
			sensVal_leftCount = 0;
			sensVal_rightCount = 0; */
		}else if ( halfStep && incrNum >= 1.0 ) {
			step_count += 0.5;
			incrNum -= 1.0;
			halfStep = !halfStep;
		}

		/* if( step_count == change_timing[ 0 ] || step_count == change_timing[ 1 ] || step_count == change_timing[ 2 ] || step_count == change_timing[ 3 ] ){
			if( sensVal_leftStroke <= 0.2 + SAKYU_KYORI ){
				// sensValで返ってきた値分strokeにする(+αする必要あり?)
				stroke[(int)step_count][LEFT] = sensVal_leftStroke - SAKYU_KYORI;
			}else if( ( ( 0.2 + SAKYU_KYORI ) < sensVal_leftStroke ) && ( sensVal_leftStroke <= ( 0.4 + SAKYU_KYORI ) ){
				// ( sensValで返ってきた値 / 2 )strokeにする(+αする必要あり?)
				stroke[(int)step_count][LEFT] = ( sensVal_leftStroke - SAKYU_KYORI ) / 2.0;
			}else if( ( ( 0.4 + SAKYU_KYORI ) < sensVal_leftStroke ) && ( sensVal_leftStroke <= ( 2.0 + SAKYU_KYORI ) )){
				//stroke[(int)step_count][LEFT] = 0.2;
			}else{
				//stroke[(int)step_count][LEFT] = 0.2;
			}

			if( sensVal_rightStroke <= 0.2 + SAKYU_KYORI ){
				// sensValで返ってきた値分strokeにする(+αする必要あり?)
				stroke[(int)step_count][RIGHT] = sensVal_rightStroke - SAKYU_KYORI;
			}else if( ( ( 0.2 + SAKYU_KYORI ) < sensVal_rightStroke ) && ( sensVal_rightStroke <= ( 0.4 + SAKYU_KYORI ) ){
				// ( sensValで返ってきた値 / 2 )strokeにする(+αする必要あり?)
				stroke[(int)step_count][RIGHT] = ( sensVal_rightStroke - SAKYU_KYORI ) / 2.0;
			}else if( ( ( 0.4 + SAKYU_KYORI ) < sensVal_rightStroke ) && ( sensVal_rightStroke <= ( 2.0 + SAKYU_KYORI ) )){
				// とりあえず0.2strokeにするとか
			}else{
				// とりあえず0.2strokeにするとか
			}
		} */

		// リミットスイッチ当たったら(砂丘)
		if( !digitalRead( PIN_ASHISAKI_FL ) && !sakyuMode){
			sakyuMode_left = true;
		}
		if( !digitalRead( PIN_ASHISAKI_FR ) && !sakyuMode){
			sakyuMode_right = true;
		}

		// リミットスイッチ当たったら(ロープ)
		/* if( !digitalRead( PIN_ASHISAKI_FL ) && ( n > 6 ) ){
			if( !rope_FL && ( step_count > stepRope_FL ) ){
				rope_FL = true;
				stepRope_FL = step_count + 1.5;

				if( halfStep ){
					FLheight_changeTiming[MAE] = step_count + 0.5;
					//FLheight_changeTiming[ATO] = step_count + 1.5;

					//FRheight_changeTiming[MAE] = step_count + 1.0;
					//FRheight_changeTiming[ATO] = step_count + 2.0;
				}else{
					FLheight_changeTiming[MAE] = step_count + 1.0;
					//FLheight_changeTiming[ATO] = step_count + 2.0;

					//FRheight_changeTiming[MAE] = step_count + 0.5;
					//FRheight_changeTiming[ATO] = step_count + 1.5;
				}
			}
		} */

		/* if( !digitalRead( PIN_ASHISAKI_FR ) && ( n > 6 ) ){
			if( !rope_FR && ( step_count > stepRope_FR ) ){
				rope_FR = true;
				stepRope_FR = step_count + 1.5;

				if( halfStep ){
					//FLheight_changeTiming[MAE] = step_count + 0.5;
					//FLheight_changeTiming[ATO] = step_count + 1.5;

					FRheight_changeTiming[MAE] = step_count + 1.0;
					//FRheight_changeTiming[ATO] = step_count + 2.0;
				}else{
					//FLheight_changeTiming[MAE] = step_count + 1.0;
					//FLheight_changeTiming[ATO] = step_count + 2.0;

					FRheight_changeTiming[MAE] = step_count + 0.5;
					//FRheight_changeTiming[ATO] = step_count + 1.5;
				}
			}
		} */

		if( rope_FL == true && ( stepRope_FL <= step_count ) ){
			rope_FL = false;
		}else if( rope_BL == true && ( stepRope_BL <= step_count ) ){
			rope_BL = false;
		}else if( rope_FR == true && ( stepRope_FR <= step_count ) ){
			rope_FR = false;
		}else if( rope_BR == true && ( stepRope_BR <= step_count ) ){
			rope_BR = false;
		}



		if( sakyuMode_left && sakyuMode_right && !sakyuMode ){
			sakyuMode = true;
			change_timing[ n ] = step_count + 1.0;
			change_timing[ n + 1 ] = step_count + 3.5;// 後ろ足が段差登る時の歩幅

			//change_timing[  ] = step_count + 4.0; // 砂丘を抜けたあと旋回
			if( halfStep ){//0~0.5
				change_timing[ n + 2 ] = step_count + 7.0;// 左旋回
				change_timing[ n + 3 ] = step_count + 9.5;//10.5;// 右旋回のタイミング
				change_timing[ n + 4 ] = step_count + 13.5;//14.5;// 元の歩幅に戻るタイミング

				height_long_timing = change_timing[ n + 3 ];

				FLheight_changeTiming[MAE] = step_count + 0.5;
				FLheight_changeTiming[ATO] = step_count + 2.5;

				BLheight_changeTiming[MAE] = step_count + 4.0;
				BLheight_changeTiming[ATO] = step_count + 6.0;

				FRheight_changeTiming[MAE] = step_count + 1.0;
				FRheight_changeTiming[ATO] = step_count + 3.0;

				BRheight_changeTiming[MAE] = step_count + 4.5;
				BRheight_changeTiming[ATO] = step_count + 5.5;
			}else{//0.5~1
				change_timing[ n + 2 ] = step_count + 7.5;// 左旋回
				change_timing[ n + 3 ] = step_count + 10.0;//11.5;// 右旋回のタイミング
				change_timing[ n + 4 ] = step_count + 14.0;//15.0;// 元の歩幅に戻るタイミング

				height_long_timing = change_timing[ n + 3 ];

				FLheight_changeTiming[MAE] = step_count + 1.0;
				FLheight_changeTiming[ATO] = step_count + 3.0;

				BLheight_changeTiming[MAE] = step_count + 4.5;
				BLheight_changeTiming[ATO] = step_count + 6.5;

				FRheight_changeTiming[MAE] = step_count + 1.5;
				FRheight_changeTiming[ATO] = step_count + 3.5;

				BRheight_changeTiming[MAE] = step_count + 5.0;
				BRheight_changeTiming[ATO] = step_count + 6.0;
				/* FLheight_changeTiming[MAE] = step_count + 1.0;
				FLheight_changeTiming[ATO] = step_count + 3.0;//2.0;

				BLheight_changeTiming[MAE] = step_count + 4.5;//2.5;//1.5;
				BLheight_changeTiming[ATO] = step_count + 6.5;//4.5;//2.5;

				FRheight_changeTiming[MAE] = step_count + 0.5;
				FRheight_changeTiming[ATO] = step_count + 2.5;//1.5;

				BRheight_changeTiming[MAE] = step_count + 5.0;//3.0;//2.0;
				BRheight_changeTiming[ATO] = step_count + 7.0;//4.0;//3.0; */				
			}
		}


		if( height_long_timing == step_count ){
			ex_height = CHANGE_HEIGHT_VERYSHORT / ( 0.5 / constNum );
			x_height[ FL ] += ex_height;
			frontLeft.changeJoge( x_height[ FL ] );

			ex_height = CHANGE_HEIGHT_VERYSHORT / ( 0.5 / constNum );
			x_height[ BL ] += ex_height;
			BackLeft.changeJoge( x_height[ BL ] );

			ex_height = CHANGE_HEIGHT_VERYSHORT / ( 0.5 / constNum );
			x_height[ FR ] += ex_height;
			frontRight.changeJoge( x_height[ FR ] );

			ex_height = CHANGE_HEIGHT_VERYSHORT / ( 0.5 / constNum );
			x_height[ BR ] += ex_height;
			BackRight.changeJoge( x_height[ BR ] );
		}

		if( n < 7 ){
			// 砂丘・前
			if( FLheight_changeTiming[MAE] == step_count ){
				ex_height = CHANGE_HEIGHT / ( 0.5 / constNum );
				x_height[ FL ] -= ex_height;
				if( !FLheighet_flag ){
					height[ FL ] += 0.05;
					FLheighet_flag = true;
				}
				
				frontLeft.changeJoge( x_height[ FL ] );
				frontLeft.changeLegHeight( height[ FL ] );
				height_count++;
			}else if( ( FLheight_changeTiming[MAE] + 0.5 ) == step_count ){
				if( FLheighet_flag ){
					height[ FL ] -= 0.05;
					FLheighet_flag = false;
				}
				frontLeft.changeLegHeight( height[ FL ] );
			}

			if( BLheight_changeTiming[MAE] == step_count ){
				ex_height = CHANGE_HEIGHT / ( 0.5 / constNum );
				x_height[ BL ] -= ex_height;
				if( !BLheighet_flag ){
					height[ BL ] += 0.06;//0.05;
					BLheighet_flag = true;
				}
				BackLeft.changeJoge( x_height[ BL ] );
				BackLeft.changeLegHeight( height[ BL ] );
			}else if( ( BLheight_changeTiming[MAE] + 0.5 ) == step_count ){
				if( BLheighet_flag ){
					height[ BL ] -= 0.06;//0.05;
					BLheighet_flag = false;
				}
				BackLeft.changeLegHeight( height[ BL ] );
			}

			if( FRheight_changeTiming[MAE] == step_count ){
				ex_height = CHANGE_HEIGHT / ( 0.5 / constNum );
				x_height[ FR ] -= ex_height;
				if( !FRheighet_flag ){
					height[ FR ] += 0.06;//0.05;
					FRheighet_flag = true;
				}
				frontRight.changeJoge( x_height[ FR ] );
				frontRight.changeLegHeight( height[ FR ] );
			}else if( ( FRheight_changeTiming[MAE] + 0.5 ) == step_count ){
				if( FRheighet_flag ){
					height[ FR ] -= 0.06;//0.05;
					FRheighet_flag = false;
				}
				frontRight.changeLegHeight( height[ FR ] );
			}

			if( BRheight_changeTiming[MAE] == step_count ){
				ex_height = CHANGE_HEIGHT / ( 0.5 / constNum );
				x_height[ BR ] -= ex_height;
				if( !BRheighet_flag ){
					height[ BR ] += 0.05;
					BRheighet_flag = true;
				}
				BackRight.changeJoge(x_height[ BR ]);
				BackRight.changeLegHeight( height[ BR ] );
			}else if( ( BRheight_changeTiming[MAE] + 0.5 ) == step_count ){
				if( !BRheighet_flag ){
					height[ BR ] -= 0.05;
					BRheighet_flag = false;
				}
				BackRight.changeLegHeight( height[ BR ] );
			}


			// 砂丘・後
			if( FLheight_changeTiming[ATO] == step_count ){
				ex_height = CHANGE_HEIGHT_SHORT / ( 0.5 / constNum );
				x_height[ FL ] += ex_height;
				if( !FLheighet_flag ){
					height[ FL ] += 0.05;
					FLheighet_flag = true;
				}
				frontLeft.changeJoge( x_height[ FL ] );
				frontLeft.changeLegHeight( height[ FL ] );
			}else if( ( FLheight_changeTiming[ATO] + 0.5 ) == step_count ){
				if( FLheighet_flag ){
					height[ FL ] -= 0.05;
					FLheighet_flag = false;
				}
				frontLeft.changeLegHeight( height[ FL ] );
			}

			if( BLheight_changeTiming[ATO] == step_count ){
				ex_height = CHANGE_HEIGHT / ( 0.5 / constNum );
				x_height[ BL ] += ex_height;
				if( !BLheighet_flag ){
					height[ BL ] += 0.05;
					BLheighet_flag = true;
				}
				BackLeft.changeJoge( x_height[ BL ] );
				BackLeft.changeLegHeight( height[ BL ] );
			}else if( ( BLheight_changeTiming[ATO] + 0.5 ) == step_count ){
				if( BLheighet_flag ){
					height[ BL ] -= 0.05;
					BLheighet_flag = false;
				}
				BackLeft.changeLegHeight( height[ BL ] );
			}

			if( FRheight_changeTiming[ATO] == step_count ){
				ex_height = CHANGE_HEIGHT_SHORT / ( 0.5 / constNum );
				x_height[ FR ] += ex_height;
				if( !FRheighet_flag ){
					height[ FR ] += 0.05;
					FRheighet_flag = true;
				}
				frontRight.changeJoge(x_height[ FR ]);
				frontRight.changeLegHeight( height[ FR ] );
			}else if( ( FRheight_changeTiming[ATO] + 0.5 ) == step_count ){
				if( FRheighet_flag ){
					height[ FR ] -= 0.05;
					FRheighet_flag = false;
				}
				frontRight.changeLegHeight( height[ FR ] );
			}

			if( BRheight_changeTiming[ATO] == step_count ){
				ex_height = CHANGE_HEIGHT / ( 0.5 / constNum );
				x_height[ BR ] += ex_height;
				if( !BRheighet_flag ){
					height[ BR ] += 0.05;
					BRheighet_flag = true;
				}
				BackRight.changeJoge(x_height[ BR ]);
				BackRight.changeLegHeight( height[ BR ] );
			}else if( ( BRheight_changeTiming[ATO] + 0.5 ) == step_count ){
				if( BRheighet_flag ){
					height[ BR ] -= 0.05;
					BRheighet_flag = false;
				}
				BackRight.changeLegHeight( height[ BR ] );
			}
		} else { // n >= 8
			// ロープ・前
			if( !FLheighet_flag ){
				height[ FL ] += 0.135;
				FLheighet_flag = true;
			}
			
			frontLeft.changeLegHeight( height[ FL ] );
			/* if( FLheight_changeTiming[MAE] == step_count ){
				//ex_height = CHANGE_HEIGHT / ( 0.25 / constNum );
				//x_height[ FL ] -= ex_height;
				if( !FLheighet_flag ){
					height[ FL ] += 0.08;
					FLheighet_flag = true;
				}
				
				//frontLeft.changeJoge( x_height[ FL ] );
				frontLeft.changeLegHeight( height[ FL ] );
			}else if( ( FLheight_changeTiming[MAE] + 0.5 ) == step_count ){
				if( FLheighet_flag ){
					height[ FL ] -= 0.08;
					FLheighet_flag = false;
				}
				frontLeft.changeLegHeight( height[ FL ] );
			} */

			if( !BLheighet_flag ){
				height[ BL ] += 0.135;
				BLheighet_flag = true;
			}

			BackLeft.changeLegHeight( height[ BL ] );
			/* if( BLheight_changeTiming[MAE] == step_count ){
				//ex_height = CHANGE_HEIGHT / ( 0.25 / constNum );
				//x_height[ BL ] -= ex_height;
				if( !BLheighet_flag ){
					height[ BL ] += 0.08;
					BLheighet_flag = true;
				}
				//BackLeft.changeJoge( x_height[ BL ] );
				BackLeft.changeLegHeight( height[ BL ] );
			}else if( ( BLheight_changeTiming[MAE] + 0.5 ) == step_count ){
				if( BLheighet_flag ){
					height[ BL ] -= 0.08;
					BLheighet_flag = false;
				}
				BackLeft.changeLegHeight( height[ BL ] );
			} */

			if( !FRheighet_flag ){
				height[ FR ] += 0.135;
				FRheighet_flag = true;
			}
			
			frontRight.changeLegHeight( height[ FR ] );

			/* if( FRheight_changeTiming[MAE] == step_count ){
				//ex_height = CHANGE_HEIGHT / ( 0.25 / constNum );
				//x_height[ FR ] -= ex_height;
				if( !FRheighet_flag ){
					height[ FR ] += 0.08;
					FRheighet_flag = true;
				}
				//frontRight.changeJoge( x_height[ FR ] );
				frontRight.changeLegHeight( height[ FR ] );
			}else if( ( FRheight_changeTiming[MAE] + 0.5 ) == step_count ){
				if( FRheighet_flag ){
					height[ FR ] -= 0.08;
					FRheighet_flag = false;
				}
				frontRight.changeLegHeight( height[ FR ] );
			} */

			if( !BRheighet_flag ){
				height[ BR ] += 0.135;
				BRheighet_flag = true;
			}
			
			BackRight.changeLegHeight( height[ BR ] );

			/* if( BRheight_changeTiming[MAE] == step_count ){
				//ex_height = CHANGE_HEIGHT / ( 0.25 / constNum );
				//x_height[ BR ] -= ex_height;
				if( !BRheighet_flag ){
					height[ BR ] += 0.08;
					BRheighet_flag = true;
				}
				//BackRight.changeJoge(x_height[ BR ]);
				BackRight.changeLegHeight( height[ BR ] );
			}else if( ( BRheight_changeTiming[MAE] + 0.5 ) == step_count ){
				if( !BRheighet_flag ){
					height[ BR ] -= 0.08;
					BRheighet_flag = false;
				}
				BackRight.changeLegHeight( height[ BR ] );
			} */


			// ロープ・後
			/* if( FLheight_changeTiming[ATO] == step_count ){
				//ex_height = CHANGE_HEIGHT_SHORT / ( 0.25 / constNum );
				//x_height[ FL ] += ex_height;
				frontLeft.changeJoge(x_height[ FL ]);
				if( !FLheighet_flag ){
					height[ FL ] += 0.08;
					FLheighet_flag = true;
				}
				//frontLeft.changeJoge( x_height[ FL ] );
				frontLeft.changeLegHeight( height[ FL ] );
			}else if( ( FLheight_changeTiming[ATO] + 0.5 ) == step_count ){
				if( FLheighet_flag ){
					height[ FL ] -= 0.08;
					FLheighet_flag = false;
				}
				frontLeft.changeLegHeight( height[ FL ] );
			} */

			/* if( BLheight_changeTiming[ATO] == step_count ){
				//ex_height = CHANGE_HEIGHT / ( 0.25 / constNum );
				//x_height[ BL ] += ex_height;
				if( !BLheighet_flag ){
					height[ BL ] += 0.08;
					BLheighet_flag = true;
				}
				//BackLeft.changeJoge( x_height[ BL ] );
				BackLeft.changeLegHeight( height[ BL ] );
			}else if( ( BLheight_changeTiming[ATO] + 0.5 ) == step_count ){
				if( BLheighet_flag ){
					height[ BL ] -= 0.08;
					BLheighet_flag = false;
				}
				BackLeft.changeLegHeight( height[ BL ] );
			} */

			/* if( FRheight_changeTiming[ATO] == step_count ){
				//ex_height = CHANGE_HEIGHT_SHORT / ( 0.25 / constNum );
				//x_height[ FR ] += ex_height;
				if( !FRheighet_flag ){
					height[ FR ] += 0.08;
					FRheighet_flag = true;
				}
				//frontRight.changeJoge(x_height[ FR ]);
				frontRight.changeLegHeight( height[ FR ] );
			}else if( ( FRheight_changeTiming[ATO] + 0.5 ) == step_count ){
				if( FRheighet_flag ){
					height[ FR ] -= 0.08;
					FRheighet_flag = false;
				}
				frontRight.changeLegHeight( height[ FR ] );
			} */

			/* if( BRheight_changeTiming[ATO] == step_count ){
				//ex_height = CHANGE_HEIGHT / ( 0.25 / constNum );
				//x_height[ BR ] += ex_height;
				if( !BRheighet_flag ){
					height[ BR ] += 0.08;
					BRheighet_flag = true;
				}
				//BackRight.changeJoge(x_height[ BR ]);
				BackRight.changeLegHeight( height[ BR ] );
			}else if( ( BRheight_changeTiming[ATO] + 0.5 ) == step_count ){
				if( BRheighet_flag ){
					height[ BR ] -= 0.08;
					BRheighet_flag = false;
				}
				BackRight.changeLegHeight( height[ BR ] );
			} */
		} // else end( n >= 8 )


		if (pre_halfStep != halfStep) {
			
			static double div = 2.0 * kataSokudo / ( stroke[n][LEFT] + stroke[n][RIGHT] );
			
			/* if ( step_count == ( change_timing[n] - 0.5 ) ) {
				kataSokudo = ( kata_sokudo[n] + kata_sokudo[n + 1] ) * 0.5;
			}else if( step_count == change_timing[1] ){ 
				kataSokudo = kata_sokudo[n + 1];
			} */
			
			if ( step_count == ( change_timing[n] - 0.5 ) ) {
				kataSokudo = ( kata_sokudo[n] + kata_sokudo[n] + kata_sokudo[n + 1] ) / 3.0;
			}else if( step_count == change_timing[1] ){ 
				kataSokudo = ( kata_sokudo[n] + kata_sokudo[n + 1] + kata_sokudo[n + 1] ) / 3.0;
			}else if( step_count == ( change_timing[n] + 0.5 ) ){
				kataSokudo = kata_sokudo[n + 1];
			}

			if ( step_count == ( change_timing[n] - 0.5 ) ) {
				if ( halfStep) { // 0.5~1.0
					// pair2 の歩幅とシフトを調節
					TmpStroke[BL] =	( stroke[n][LEFT] + stroke[n + 1][LEFT] ) * 0.5 + ( shift[n][BL] - shift[n + 1][BL] );
					TmpStroke[FR] = ( stroke[n][RIGHT] + stroke[n + 1][RIGHT] ) * 0.5 + ( shift[n][FR] - shift[n + 1][FR] );
					
					TmpShift[BL] = ( stroke[n][LEFT] - stroke[n + 1][LEFT] + 2.0 * ( shift[n][BL] + shift[n + 1][BL] ) ) * 0.25;
					TmpShift[FR] = ( stroke[n][RIGHT] - stroke[n + 1][RIGHT] + 2.0 * ( shift[n][FR] + shift[n + 1][FR] ) ) * 0.25;
					
					// pair1 の歩幅とシフトは前のまま
					TmpStroke[FL] = stroke[n][LEFT];
					TmpStroke[BR] = stroke[n][RIGHT];
					
					TmpShift[FL] = shift[n][FL];
					TmpShift[BR] = shift[n][BR];
					
					// pair1(立脚) の歩幅を利用して肩速度とconstNumを求める
					/* kataSokudo_FL = 2.0 * kataSokudo * TmpStroke[FL] / ( TmpStroke[FR] + TmpStroke[FL] );
					kataSokudo_BL = 2.0 * kataSokudo * TmpStroke[BL] / ( TmpStroke[BR] + TmpStroke[BL] );
					kataSokudo_FR = 2.0 * kataSokudo * TmpStroke[FR] / ( TmpStroke[FL] + TmpStroke[FR] );
					kataSokudo_BR = 2.0 * kataSokudo * TmpStroke[BR] / ( TmpStroke[BL] + TmpStroke[BR] ); */
					div = 2.0 * kataSokudo / ( TmpStroke[FL] + TmpStroke[BR] );
					constNum = INT_TIME * div;// kataSokudo / ( TmpStroke[FL] + TmpStroke[BR] );
					
				}else { // 0.0~0.5
					// pair1 の歩幅とシフトを調節
					TmpStroke[FL] = ( stroke[n][LEFT] + stroke[n + 1][LEFT] ) * 0.5 + ( shift[n][FL] - shift[n + 1][FL] );
					TmpStroke[BR] = ( stroke[n][RIGHT] + stroke[n + 1][RIGHT] ) * 0.5 + ( shift[n][BR] - shift[n + 1][BR] );
					
					TmpShift[FL] = ( stroke[n][LEFT] - stroke[n + 1][LEFT] + 2.0 * ( shift[n][FL] + shift[n + 1][FL] ) ) * 0.25;
					TmpShift[BR] = ( stroke[n][RIGHT] - stroke[n + 1][RIGHT] + 2.0 * ( shift[n][BR] + shift[n + 1][BR] ) ) * 0.25;
					
					// pair2 の歩幅とシフトは前のまま
					TmpStroke[BL] =	stroke[n][LEFT];
					TmpStroke[FR] = stroke[n][RIGHT];
					
					TmpShift[BL] = shift[n][BL];
					TmpShift[FR] = shift[n][FR];
					
					// pair2(立脚) の歩幅を利用して肩速度とconstNumを求める
					/* kataSokudo_FL = 2.0 * kataSokudo * stroke[0][LEFT] / ( stroke[0][RIGHT] + stroke[0][LEFT] );
					kataSokudo_BL = 2.0 * kataSokudo * stroke[0][LEFT] / ( stroke[0][RIGHT] + stroke[0][LEFT] );
					kataSokudo_FR = 2.0 * kataSokudo * stroke[0][RIGHT] / ( stroke[0][LEFT] + stroke[0][RIGHT] );
					kataSokudo_BR = 2.0 * kataSokudo * stroke[0][RIGHT] / ( stroke[0][LEFT] + stroke[0][RIGHT] ); */
					
					div = 2.0 * kataSokudo / ( TmpStroke[BL] + TmpStroke[FR] );
					constNum = INT_TIME * div;//kataSokudo / ( TmpStroke[BL] + TmpStroke[FR] );
					
				}
			}else if (step_count == change_timing[n]) {
				wait_count++;
				if (halfStep) {
					// pair2 の歩幅とシフトを調節
					TmpStroke[BL] =	( stroke[n][LEFT] + stroke[n + 1][LEFT] ) * 0.5 + ( shift[n][BL] - shift[n + 1][BL] );
					TmpStroke[FR] = ( stroke[n][RIGHT] + stroke[n + 1][RIGHT] ) * 0.5 + ( shift[n][FR] - shift[n + 1][FR] );
					
					TmpShift[BL] = ( stroke[n][LEFT] - stroke[n + 1][LEFT] + 2.0 * ( shift[n][BL] + shift[n + 1][BL] ) ) * 0.25;
					TmpShift[FR] = ( stroke[n][RIGHT] - stroke[n + 1][RIGHT] + 2.0 * ( shift[n][FR] + shift[n + 1][FR] ) ) * 0.25;
					
					// pair1 の歩幅とシフトは新しいやつ
					TmpStroke[FL] = stroke[n + 1][LEFT];
					TmpStroke[BR] = stroke[n + 1][RIGHT];
					
					TmpShift[FL] = shift[n + 1][FL];
					TmpShift[BR] = shift[n + 1][BR];
					
					// pair2(立脚) の歩幅を利用して肩速度とconstNumを求める
					div = 2.0 * kataSokudo / ( TmpStroke[FL] + TmpStroke[BR] );
					constNum = INT_TIME * div;//kataSokudo / ( TmpStroke[BL] + TmpStroke[FR] );
					
				}else {
					// pair1 の歩幅とシフトを調節
					TmpStroke[FL] = ( stroke[n][LEFT] + stroke[n + 1][LEFT] ) * 0.5 + ( shift[n][FL] - shift[n + 1][FL] );
					TmpStroke[BR] = ( stroke[n][RIGHT] + stroke[n + 1][RIGHT] ) * 0.5 + ( shift[n][BR] - shift[n + 1][BR] );
					
					TmpShift[FL] = ( stroke[n][LEFT] - stroke[n + 1][LEFT] + 2.0 * ( shift[n][FL] + shift[n + 1][FL] ) ) * 0.25;
					TmpShift[BR] = ( stroke[n][RIGHT] - stroke[n + 1][RIGHT] + 2.0 * ( shift[n][BR] + shift[n + 1][BR] ) ) * 0.25;
					
					// pair2 の歩幅とシフトは新しいやつ
					TmpStroke[BL] =	stroke[n + 1][LEFT];
					TmpStroke[FR] = stroke[n + 1][RIGHT];
					
					TmpShift[BL] = shift[n + 1][BL];
					TmpShift[FR] = shift[n + 1][FR];
					
					// pair1(立脚) の歩幅を利用して肩速度とconstNumを求める
					div = 2.0 * kataSokudo / ( TmpStroke[BL] + TmpStroke[FR] );
					constNum = INT_TIME * div;//kataSokudo / ( TmpStroke[FL] + TmpStroke[BR] );
					
				}
			}else if (step_count == (change_timing[n] + 0.5)) {
				if (halfStep) {
					// pair1 の歩幅とシフトは新しいやつ
					TmpStroke[FL] = stroke[n + 1][LEFT];
					TmpStroke[BR] = stroke[n + 1][RIGHT];
					
					TmpShift[FL] = shift[n + 1][FL];
					TmpShift[BR] = shift[n + 1][BR];
					
					// pair1(立脚) の歩幅を利用して肩速度とconstNumを求める
					div = 2.0 * kataSokudo / ( TmpStroke[FL] + TmpStroke[BR] );
					constNum = INT_TIME * div;//* kataSokudo / ( TmpStroke[FL] + TmpStroke[BR] );
					
				}else {
					// pair2 の歩幅はとシフト新しいやつ
					TmpStroke[BL] =	stroke[n + 1][LEFT];
					TmpStroke[FR] = stroke[n + 1][RIGHT];
					
					TmpShift[BL] = shift[n + 1][BL];
					TmpShift[FR] = shift[n + 1][FR];

					// pair2(立脚) の歩幅を利用して肩速度とconstNumを求める
					div = 2.0 * kataSokudo / ( TmpStroke[BL] + TmpStroke[FR] );
					constNum = INT_TIME * div;//kataSokudo / ( TmpStroke[BL] + TmpStroke[FR] );
					
				}	
				
				n++;
			}
			
			kataSokudo_FL = TmpStroke[FL] * div;// / ( TmpStroke[BL] + TmpStroke[FR] );//( TmpStroke[FR] + TmpStroke[FL] );
			kataSokudo_BL = TmpStroke[BL] * div;// / ( TmpStroke[BL] + TmpStroke[FR] );//( TmpStroke[BR] + TmpStroke[BL] );
			kataSokudo_FR = TmpStroke[FR] * div;// / ( TmpStroke[BL] + TmpStroke[FR] );//( TmpStroke[FL] + TmpStroke[FR] );
			kataSokudo_BR = TmpStroke[BR] * div;// / ( TmpStroke[BL] + TmpStroke[FR] );//( TmpStroke[BL] + TmpStroke[BR] );
			
			//changepara する
			frontLeft.changeSayu( TmpStroke[FL] );
			frontLeft.changeLegShift( TmpShift[FL] );
			BackLeft.changeSayu( TmpStroke[BL] );
			BackLeft.changeLegShift( TmpShift[BL] );
			frontRight.changeSayu( TmpStroke[FR] );
			frontRight.changeLegShift( TmpShift[FR] );
			BackRight.changeSayu( TmpStroke[BR] );
			BackRight.changeLegShift( TmpShift[BR] );
			
			frontLeft.changeConstNum( constNum );
			frontLeft.changeKataSokudo( kataSokudo_FL );
			BackLeft.changeConstNum( constNum );
			BackLeft.changeKataSokudo( kataSokudo_BL );
			frontRight.changeConstNum( constNum );
			frontRight.changeKataSokudo( kataSokudo_FR );
			BackRight.changeConstNum( constNum );
			BackRight.changeKataSokudo( kataSokudo_BR );	
		}

		pre_halfStep = halfStep;

		//calcAngle する
		// 目標位置生成
		refFLangle = frontLeft.calcAngle( frontLeft.calcSayu( incrNum ), frontLeft.calcJoge( incrNum ) );
		refFLlinear = frontLeft.calcLinear( frontLeft.calcSayu( incrNum ), frontLeft.calcJoge( incrNum ) );
		refBLangle = BackLeft.calcAngle( BackLeft.calcSayu( incrNum ), BackLeft.calcJoge( incrNum ) );
		refBLlinear = BackLeft.calcLinear( BackLeft.calcSayu( incrNum ), BackLeft.calcJoge( incrNum ) );
		refFRangle = frontRight.calcAngle( frontRight.calcSayu( incrNum ), frontRight.calcJoge( incrNum ) );
		refFRlinear = frontRight.calcLinear( frontRight.calcSayu( incrNum ), frontRight.calcJoge( incrNum ) );
		refBRangle = BackRight.calcAngle( BackRight.calcSayu( incrNum ), BackRight.calcJoge( incrNum ) );
		refBRlinear = BackRight.calcLinear( BackRight.calcSayu( incrNum ), BackRight.calcJoge( incrNum ) );
		
		// 目標速度
		refFLAsokudo = FLAichiPID.getCmd(refFLangle, FLangle, 6.0);
		refFLLsokudo = FLLichiPID.getCmd(refFLlinear, FLlinear, 4.0);
		if(fabs(refFLLsokudo) > 0.05){
			//cmd_base = cmd_base + cmd_base * G;
			refFLLsokudo = refFLLsokudo + refFLLsokudo * 0.2;//BLLgain;
		}
		
		refBLAsokudo = BLAichiPID.getCmd(refBLangle, BLangle, 6.0);
		refBLLsokudo = BLLichiPID.getCmd(refBLlinear, BLlinear, 4.0);
		refFRAsokudo = FRAichiPID.getCmd(refFRangle, FRangle, 6.0);
		refFRLsokudo = FRLichiPID.getCmd(refFRlinear, FRlinear, 4.0);
		refBRAsokudo = BRAichiPID.getCmd(refBRangle, BRangle, 7.0);
		refBRLsokudo = BRLichiPID.getCmd(refBRlinear, BRlinear, 4.0);//1.0);
		
		// sabertoothに送るコマンド
		FLAcmd += FLAsokudoPID.getCmd(refFLAsokudo, FLAsokudo, 30.0) * ( -1.0 / 127.0 ); // 20より大きくするとcmdの値がいきなり大きく変わる
		FLLcmd += FLLsokudoPID.getCmd(refFLLsokudo, FLLsokudo, 76.0) * ( -1.0 / 127.0 );//127.0) * ( 1.0 / 127.0 );
		BLAcmd += BLAsokudoPID.getCmd(refBLAsokudo, BLAsokudo, 30.0) * ( -1.0 / 127.0 );
		BLLcmd += BLLsokudoPID.getCmd(refBLLsokudo, BLLsokudo, 127.0) * ( 1.0 / 127.0 );
		FRAcmd += FRAsokudoPID.getCmd(refFRAsokudo, FRAsokudo, 30.0) * ( 1.0 / 127.0 );
		FRLcmd += FRLsokudoPID.getCmd(refFRLsokudo, FRLsokudo, 127.0) * ( 1.0 / 127.0 );
		BRAcmd += BRAsokudoPID.getCmd(refBRAsokudo, BRAsokudo, 30.0) * ( 1.0 / 127.0 );
		BRLcmd += BRLsokudoPID.getCmd(refBRLsokudo, BRLsokudo, 127.0) * ( -1.0 / 127.0 );
		
		// コマンドの値が大きくなり過ぎないように制限
		if( FLAcmd < -1.0 ){
			FLAcmd = -1.0;
		}else if( 1.0 < FLAcmd ){
			FLAcmd = 1.0;
		}
		if( FLLcmd < -1.0 ){
			FLLcmd = -1.0;
		}else if( 1.0 < FLLcmd ){
			FLLcmd = 1.0;
		}
		/* if( FLLcmd < -0.6 ){
			FLLcmd = -0.6;
		}else if( 0.6 < FLLcmd ){
			FLLcmd = 0.6;
		} */
		if( BLAcmd < -1.0 ){
			BLAcmd = -1.0;
		}else if( 1.0 < BLAcmd ){
			BLAcmd = 1.0;
		}
		if( BLLcmd < -1.0 ){
			BLLcmd = -1.0;
		}else if( 1.0 < BLLcmd ){
			BLLcmd = 1.0;
		}
		if( FRAcmd < -1.0 ){
			FRAcmd = -1.0;
		}else if( 1.0 < FRAcmd ){
			FRAcmd = 1.0;
		}
		if( FRLcmd < -1.0 ){
			FRLcmd = -1.0;
		}else if( 1.0 < FRLcmd ){
			FRLcmd = 1.0;
		}
		if( BRAcmd < -1.0 ){
			BRAcmd = -1.0;
		}else if( 1.0 < BRAcmd ){
			BRAcmd = 1.0;
		}
		if( BRLcmd < -1.0 ){
			BRLcmd = -1.0;
		}else if( 1.0 < BRLcmd ){
			BRLcmd = 1.0;
		}
	
		saber1.saberCmd(ANGLE_CMD, FLAcmd);
		saber1.saberCmd(LINEAR_CMD, FLLcmd);
		saber2.saberCmd(ANGLE_CMD, BLAcmd);
		saber2.saberCmd(LINEAR_CMD, BLLcmd);
		saber3.saberCmd(ANGLE_CMD, FRAcmd);
		saber3.saberCmd(LINEAR_CMD, FRLcmd);
		saber4.saberCmd(ANGLE_CMD, BRAcmd);
		saber4.saberCmd(LINEAR_CMD, BRLcmd);
		
		/* Serial.print(incrNum, 4);
		Serial.print("\t");
		Serial.print(refFRangle, 4);//(refFLlinear, 4);
		Serial.print("\t");
		Serial.print(FRangle, 4);//(FLlinear, 4);
		Serial.print("\t");
		Serial.print(FRLcmd);
		Serial.print("\t");
		Serial.print(refFRAsokudo, 4);//(refFLLsokudo, 4);//
		Serial.print("\t");
		Serial.println(FRAsokudo, 4);//(FLLsokudo, 4);// */

		Serial.print(height[ FL ], 4);
		Serial.print("\t");
		Serial.print(height[ BL ], 4);
		Serial.print("\t");
		Serial.print(height[ FR ], 4);
		Serial.print("\t");
		Serial.println(height[ BR ], 4);

		

		/* Serial.print(step_count);
		Serial.print("\t");
		Serial.print(frontLeft.calcSayu( incrNum ), 4);
		Serial.print("\t");
		Serial.println(frontLeft.calcJoge( incrNum ), 4); */
		/* Serial.print("\t");
		Serial.print(BackLeft.calcSayu( incrNum ), 4);
		Serial.print("\t");
		Serial.print(BackLeft.calcJoge( incrNum ), 4);
		Serial.print("\t");
		Serial.print(frontRight.calcSayu( incrNum ), 4);
		Serial.print("\t");
		Serial.print(frontRight.calcJoge( incrNum ), 4);
		Serial.print("\t");
		Serial.print(BackRight.calcSayu( incrNum ), 4);
		Serial.print("\t");
		Serial.println(BackRight.calcJoge( incrNum ), 4); */

		/* if( incrNum > ( 1.0 - ( constNum * 4.0 ) ) ){
			if( sensVal_count < 4 ){
				sensVal_left[ sensVal_count ] = gSensor[0].readRangeSingleMillimeters() * 0.001;
				sensVal_right[ sensVal_count ] = gSensor[1].readRangeSingleMillimeters() * 0.001;
			} else {
				sensVal_left[ 0 ] = gSensor[0].readRangeSingleMillimeters() * 0.001;
				sensVal_right[ 0 ] = gSensor[1].readRangeSingleMillimeters() * 0.001;
			}
			sensVal_count++;
		}else{
			sensVal_count = 0;
		} */
		/* if( incrNum > ( 1.0 - constNum ) ){
			Serial.print(step_count);
			Serial.print("\t");
			sensVal_left[0] = gSensor[0].readRangeSingleMillimeters() * 0.001;
			Serial.print(sensVal_left[0], 4);
			Serial.print("\t");
			sensVal_right[0] = gSensor[1].readRangeSingleMillimeters() * 0.001;
			Serial.println(sensVal_right[0], 4);
		} */

		/* kari++;

		if( kari == 6 ){
			Serial.print(step_count);
			Serial.print("\t");
			sensVal_left[0] = (double)gSensor[0].readRangeSingleMillimeters() * 0.001;
			if (gSensor[0].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
			Serial.print(sensVal_left[0], 4);
			Serial.print("\t");
			sensVal_right[0] = (double)gSensor[1].readRangeSingleMillimeters() * 0.001;
			if (gSensor[1].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
			Serial.println(sensVal_right[0], 4);
			kari = 0;
		} */
	}
}

void reboot_function(){
	system_reboot(REBOOT_USERAPP);
}

void setup() {
	// put your setup code here, to run once:
	pinMode(PIN_SW, INPUT);
	pinMode(PIN_LED0, OUTPUT);
    pinMode(PIN_LED1, OUTPUT);
	pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
	pinMode(PIN_FLSW, INPUT);
	pinMode(PIN_BLSW, INPUT);
	pinMode(PIN_FRSW, INPUT);
	pinMode(PIN_BRSW, INPUT);
	pinMode(PIN_START, INPUT);
	pinMode(PIN_ASHISAKI_FL, INPUT);
	pinMode(PIN_ASHISAKI_FR, INPUT);
	pinMode(PIN_GEREGE, INPUT);

	pinMode(PIN_15, OUTPUT);
	digitalWrite(PIN_15, HIGH);
	pinMode(PIN_51, OUTPUT);
	digitalWrite(PIN_51, HIGH);
	pinMode(PIN_13, OUTPUT);
	digitalWrite(PIN_13, HIGH);
	pinMode(PIN_12, OUTPUT);
	digitalWrite(PIN_12, HIGH);
	
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
	//SPI.setClockDivider(SPI_CLOCK_DIV32);//1.5MHz
	SPI.begin();
	
	FLlinearEnc.init();
	BLlinearEnc.init();
	FRlinearEnc.init();
	BRlinearEnc.init();
	
	Serial.begin(115200);
	
	SWSerial1.begin(115200);
	
	/* Wire.begin();
	for (int i = 0; i < SENSOR_NUM; i++){
		pinMode(VL53L0X_GPIO[i], OUTPUT);
		digitalWrite(VL53L0X_GPIO[i], LOW);
	}

	for (int i = 0; i < SENSOR_NUM; i++) {
		// センサを初期化
		pinMode(VL53L0X_GPIO[i], INPUT);
		if (gSensor[i].init() == true){
			gSensor[i].setTimeout(50);//(100);
			gSensor[i].startContinuous();
			int address = ADDRESS_00 + (i * 2);
			gSensor[i].setAddress(address);
			//gSensor[i].setMeasurementTimingBudget(20000);
		}else{
			Serial.print("Sensor ");
			Serial.print(i);
			Serial.println(" error");
		}
	} */
	
	FLLichiPID.PIDinit(0.0, 0.0);
	FLAichiPID.PIDinit(0.0, 0.0);
	FLLsokudoPID.PIDinit(0.0, 0.0);
	FLAsokudoPID.PIDinit(0.0, 0.0);
	BLLichiPID.PIDinit(0.0, 0.0);
	BLAichiPID.PIDinit(0.0, 0.0);
	BLLsokudoPID.PIDinit(0.0, 0.0);
	BLAsokudoPID.PIDinit(0.0, 0.0);
	FRLichiPID.PIDinit(0.0, 0.0);
	FRAichiPID.PIDinit(0.0, 0.0);
	FRLsokudoPID.PIDinit(0.0, 0.0);
	FRAsokudoPID.PIDinit(0.0, 0.0);
	BRLichiPID.PIDinit(0.0, 0.0);
	BRAichiPID.PIDinit(0.0, 0.0);
	BRLsokudoPID.PIDinit(0.0, 0.0);
	BRAsokudoPID.PIDinit(0.0, 0.0);
	frontLeft.setSayu(INT_TIME, PAIR1, constNum, stroke[0][LEFT], kataSokudo_FL, shift[0][FL]);
	frontLeft.setJoge(INT_TIME, PAIR1, constNum, x_height[FL], stroke[0][LEFT], kataSokudo_FL, height[FL]);
	BackLeft.setSayu(INT_TIME, PAIR2, constNum, stroke[0][LEFT], kataSokudo_BL, shift[0][BL]);
	BackLeft.setJoge(INT_TIME, PAIR2, constNum, x_height[BL], stroke[0][LEFT], kataSokudo_BL, height[BL]);
	frontRight.setSayu(INT_TIME, PAIR2, constNum, stroke[0][RIGHT], kataSokudo_FR, shift[0][FR]);
	frontRight.setJoge(INT_TIME, PAIR2, constNum, x_height[FR], stroke[0][RIGHT], kataSokudo_FR, height[FR]);
	BackRight.setSayu(INT_TIME, PAIR1, constNum, stroke[0][RIGHT], kataSokudo_BR, shift[0][BR]);
	BackRight.setJoge(INT_TIME, PAIR1, constNum, x_height[BR], stroke[0][RIGHT], kataSokudo_BR, height[BR]);
	
	MsTimerTPU3::set((int)(INT_TIME * 1000), timerWarikomi_10ms); // 10ms period
	MsTimerTPU3::start();
}

void loop() {
	// put your main code here, to run repeatedly:
	if(!digitalRead(PIN_SW)){
		reboot_function();
	}
	
	// Serial3.flush();
	
	//delayMicroseconds(200);
}