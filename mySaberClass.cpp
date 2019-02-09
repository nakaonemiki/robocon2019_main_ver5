#include "Arduino.h"
#include "SoftwareSerial.h"
#include "mySaberClass.h"

mySaberClass::mySaberClass(Sabertooth* xSaber){
  pSaber = xSaber;
}

void mySaberClass::saberCmd(byte xSaber_cmd, float xVal){ // xSaber_cmd : 1 or 2
	byte saber_cmd;
	
	xVal *= 127.0;
	
	if( xVal >= 0.0 ){
		if( xSaber_cmd == 1 ){
			saber_cmd = 0;
		} else { // 2
			saber_cmd = 4;
		}
		
		
		if( xVal > 127.0 ){
			val = 127.0;
		}else{
			val = xVal;
		}
	}else if( xVal < 0.0 ){
		if( xSaber_cmd == 1 ){
			saber_cmd = 1;
		} else { // 2
			saber_cmd = 5;
		}
		
		if( xVal < -127.0 ){
			val = 127.0;
		}else{
			val = (-1.0) * xVal;
		}
	}
	
	/* Serial3.write(128);
	Serial3.write(saber_cmd);
	Serial3.write(val);
	Serial3.write((128 + saber_cmd + val) & B01111111); */
	
	pSaber->command(saber_cmd, (byte)val);
}

// 元のプログラム
/* void mySaberClass::saberCmd(byte xCmd, float xVal){
	xVal *= 127.0;
	if( xVal >= 0.0 ){
		if( xVal > 127.0 ){
			val = 127.0;
		}else{
			val = xVal;
		}
	}else if( xVal < 0.0 ){
		if( xVal < -127.0 ){
			val = 127.0;
		}else{
			val = (-1.0) * xVal;
		}
	}
	
	pSaber->command(xCmd, (byte)val);
} */







/* #include "Arduino.h"
#include "SoftwareSerial.h"
#include "Sabertooth.h"

#include "mySabertoothClass.h"

// コンストラクタ
mySabertoothClass::mySabertoothClass(byte RxPin, byte TxPin, byte saberAdress, long saberbps){ // RxPinはなければNC(NOT_A_PINは0っぽいので0のピン使うことになっちゃう？)
	gSaberAdress = saberAdress;
	
	SoftwareSerial SWSerial(RxPin, TxPin); // RX on no pin (unused), TX on pin 11 (to S1).
	Sabertooth ST(saberAdress, SWSerial); // Address 128, and use SWSerial as the serial port.
	
	SWSerial.begin(saberbps);
	ST.autobaud();
}

void mySabertoothClass::write(byte saberCmd, float fSaberVal){ // saberCmdは必ず-1.0~1.0の値を受け取る
	fSaberVal *= 127.0;
	byte bSaberVal = (byte)fSaberVal;
	
	ST.command((byte)saberCmd, (byte)bSaberVal);
} */