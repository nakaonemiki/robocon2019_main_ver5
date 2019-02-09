#include "Arduino.h"

class legClass
{
public:
    legClass();
	
	double setSayu(double xint_time, byte xpairNum, double xconstNum, double xY, double xkataSokudo, double xLegShift);
	double setJoge(double xint_time, byte xpairNum, double xconstNum, double xX, double xY, double xkataSokudo, double xLegHeight);
	double calcSayu(double incrNum);
	double calcJoge(double incrNum);
	double changeJoge(double xX);
	double changeSayu(double xY);
	double changeLegShift(double xLegShift);
	double changeLegHeight(double xLegHeight);
	double changeKataSokudo(double xkataSokudo);
	double changeConstNum(double xconstNum);
	double calcAngle(double Y, double X);
	double calcLinear(double Y, double X);

private:
	double int_time;
	double Xcoord;
	double Ycoord;
	byte pairNum;
	double constNum;
	double Y;
	double X;
	double kataSokudo; 
	double LegShift;
	double LegHeight;
};