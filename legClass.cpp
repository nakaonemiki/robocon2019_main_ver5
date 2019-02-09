#include "Arduino.h"
#include "legClass.h"

// コンストラクタ
legClass::legClass()
{
}

// 左右のセッティング
double legClass::setSayu(double xint_time, byte xpairNum, double xconstNum, double xY, double xkataSokudo, double xLegShift)
{
	int_time = xint_time;
	pairNum = xpairNum;
	constNum = xconstNum;
	Y = xY;
	kataSokudo = xkataSokudo;
	LegShift = xLegShift;
}

// 上下のセッティング
double legClass::setJoge(double xint_time, byte xpairNum, double xconstNum, double xX, double xY, double xkataSokudo, double xLegHeight)
{
	int_time = xint_time;
	pairNum = xpairNum;
	constNum = xconstNum;
	X = xX;
	Y = xY;
	kataSokudo = xkataSokudo;
	LegHeight = xLegHeight;
}

// 左右の計算
double legClass::calcSayu(double incrNum)
{	
	// pair 1
	if( pairNum == 1 ){
		if( ( 0.0 <= incrNum ) && ( incrNum <= 0.5 ) ){
			Ycoord = 2.0 * ( Y * ( incrNum / 0.5 - sin( 2.0 * PI * incrNum / 0.5 ) / ( 2.0 * PI ) ) - incrNum * Y - Y / 4.0 ) - LegShift;
		}else if( ( 0.5 < incrNum ) && ( incrNum <= 1.0 ) ){
			Ycoord = ( Y / 2.0 - kataSokudo * ( incrNum - 0.5 ) / constNum * ( int_time * 2.0 ) ) - LegShift;
		}
	// pair 2
	}else if( pairNum == 2 ){
		double pair2_Num = incrNum;
		/* if( ( 0.0 <= incrNum ) && ( incrNum < 0.5 ) ){
			pair2_Num = incrNum + 0.5;
		}else if( 0.5 <= incrNum ){
			pair2_Num = incrNum - 0.5;
		} */
		/* if( incrNum <= 0.5 ){
			pair2_Num = incrNum + 0.5;
		}else{
			pair2_Num = incrNum - 0.5;
		} */
		
		if( ( 0.0 <= pair2_Num ) && ( pair2_Num < 0.5 ) ){
			Ycoord = ( Y / 2.0 - kataSokudo * ( pair2_Num ) / constNum * ( int_time * 2.0 ) ) - LegShift;//kataSokudo * ( int_time / constNum * incrNum ) - Y / 4.0 ) - LegShift;
		}else if( ( 0.5 <= pair2_Num ) && ( pair2_Num <= 1.0 ) ){
			Ycoord = 2.0 * ( Y * ( ( pair2_Num - 0.5 ) / 0.5 - sin( 2.0 * PI * ( pair2_Num - 0.5 ) / 0.5 ) / ( 2.0 * PI ) ) - ( incrNum - 0.5 ) * Y - Y / 4.0 ) - LegShift;//2.0 * ( Y / 4.0 - kataSokudo * ( pair2_Num - 0.5 ) ) - LegShift;
		}/* else if( 0.0 == pair2_Num ){
			Ycoord = Y/2 - LegShift;
		} */
	}
	
	return Ycoord;
}

// 上下の計算
double legClass::calcJoge(double incrNum)
{	
	if( pairNum == 1 ){
		if( ( 0.0 <= incrNum ) && ( incrNum <= 0.25 ) ){
			Xcoord = X - ( 2.0 * LegHeight * ( incrNum / 0.5 - sin( 4.0 * PI * incrNum / 0.5 ) / ( 4.0 * PI ) ) );
		}else if( ( 0.25 < incrNum ) && ( incrNum <= 0.5 ) ){
			Xcoord = X - ( 2.0 * LegHeight * ( 1.0 - incrNum / 0.5 - sin( 4.0 * PI * ( 1.0 -  incrNum / 0.5 ) ) / ( 4.0 * PI ) ) );
		}else if( ( 0.5 < incrNum ) && ( incrNum <= 1.0 ) ){
			Xcoord = X;
		}
	// pair 2
	}else if( pairNum == 2 ){
		double pair2_Num = incrNum;//0.0;
		/* if( incrNum <= 0.5 ){
			pair2_Num = incrNum + 0.5;
		}else{
			pair2_Num = incrNum - 0.5;
		} */
		
		if( ( 0.0 <= pair2_Num ) && ( pair2_Num < 0.5 ) ){
			Xcoord = X;
		}else if( ( 0.5 <= pair2_Num ) && ( pair2_Num <= 0.75 ) ){
			Xcoord = X - ( 2.0 * LegHeight * ( ( pair2_Num - 0.5 ) / 0.5 - sin( 4.0 * PI * ( pair2_Num - 0.5 ) / 0.5 ) / ( 4.0 * PI ) ) );
		}else if( ( 0.75 < pair2_Num ) && ( pair2_Num <= 1.0 ) ){
			Xcoord = X - ( 2.0 * LegHeight * ( 1.0 - ( pair2_Num - 0.5 ) / 0.5 - sin( 4.0 * PI * ( 1.0 - ( pair2_Num - 0.5 ) / 0.5 ) ) / ( 4.0 * PI ) ) );
		}
	}
	
	return Xcoord;
}

// 値の変更
double legClass::changeJoge(double xX)
{
	X = xX;
}

double legClass::changeSayu(double xY){
	Y = xY;
}

double legClass::changeLegShift(double xLegShift)
{
	LegShift = xLegShift;
}

double legClass::changeLegHeight(double xLegHeight)
{
	LegHeight = xLegHeight;
}

double legClass::changeKataSokudo(double xkataSokudo)
{
	kataSokudo = xkataSokudo;
}

double legClass::changeConstNum(double xconstNum){
	constNum = xconstNum;
}

// 角度と直動用の計算
double legClass::calcAngle(double Y, double X)
{
	return atan( Y / X );
}

double legClass::calcLinear(double Y, double X)
{
	return sqrt( pow( Y, 2 ) + pow( X, 2 ));
}