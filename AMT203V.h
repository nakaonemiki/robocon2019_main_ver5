#include "mySPI.h"

class AMT203V{
  public:
  AMT203V(SPIClass*, byte xCSBpin);

  int spi_write(int msg);
  int getEncount();
  int setZeroPos();

  private:
  byte CSBpin;
  int ABSposition;
  int temp[2];
  SPIClass *pSPI;
  
};
