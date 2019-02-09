#include "Sabertooth.h"

class mySaberClass{
  public:
  mySaberClass(Sabertooth*);

  void saberCmd(byte xSaber_cmd, float val);

  private:
  Sabertooth *pSaber;
  byte val;
};





/* #ifndef mySabertoothClass_h
#define mySabertoothClass_h

#include "SoftwareSerial.h"
#include "Sabertooth.h"

class mySabertoothClass{

  public:
	// コンストラクタ用
    mySabertoothClass(byte RxPin, byte TxPin, byte saberAdress, long saberbps);
	
	//void write(byte saberCmd, float fSaberVal);
	
  private:
	SoftwareSerial SWSerial; // RX on no pin (unused), TX on pin 11 (to S1).
	Sabertooth ST;
	//byte gSaberAdress;
};

#endif
 */