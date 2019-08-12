#include "leafDetection.h"

int main(int argc,char* argv[])
{
	cout<<"hello"<<endl;
	
	cout<<"predict leaf position with LLA"<<endl;
	
	cout<<"demo"<<endl;
	leaf_detection l1(42.622128,117.7054,1793.1,197.73,10);
	l1.print();
	leaf_detection l2(42.622141,117.70534,1793.32,196.7,5);
	l2.print();
	
	cout<<"test"<<endl;
	leaf_detection a1(42.6196555,117.712007,1792.52,258.0,10.3);
	a1.print();
	leaf_detection a2(42.6201965,117.711978,1791.545,100.36,7.1);
	a2.print();
	
	cout<<"predict the third point position with LLA"<<endl;
	leaf_detection a11(42.6197461,117.711981,1792.52,258.0,10.3);
	leaf_detection a22(42.6201337,117.711962,1791.545,100.36,7.1);
	leaf_detection::getVertex(a11,a22);

	cout<<"predict flight angle"<<endl;
	/*
	cout<<"demo"<<endl;
	leaf_detection p1(42.639746,117.677542,1790.89,141.69,10);
	leaf_detection p2(42.63963,117.678424,1777.258,141.69,10);
	double angle1=leaf_detection::getFlightAngle(p1,p2);
	double angle2=leaf_detection::getFlightAngle(42.639746,117.677542,42.639630,117.678424);//la,lo,la,lo
	cout<<angle1<<endl;
	cout<<angle2*180.0/3.1415926<<endl;
	*/
	cout<<"test"<<endl;
	leaf_detection an1(42.639746,117.677542,1790.89,197.73,10);
	leaf_detection an2(42.63963,117.678424,1777.258,196.7,5);
	double angle11=leaf_detection::getFlightAngle(an1,an2);
	double angle22=leaf_detection::getFlightAngle(42.6196555,117.712007,42.6201965,117.711978);//la,lo,la,lo
	cout<<angle11*180.0/3.1415926<<endl;
	cout<<angle22*180.0/3.1415926<<endl;

	cout<<"ALL"<<endl;
	leaf_detection::getVertexAll(a1,a2);

	return 0;
}