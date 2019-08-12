#ifndef __LEAFDETECTION_H__
#define __LEAFDETECTION_H__

#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>

using namespace std;
using namespace cv;

const double eR=6371000;
const double pi=3.1415926;

#ifndef PCL
namespace pcl{
	class PointXYZ
	{
	public:
		double x;
		double y;
		double z;
	};
}
#endif

class leaf_detection
{
	public:
		leaf_detection(double la=42.622128,double lo=117.705400,double height=100.0,double y=0.0,double length=10.0):
			_la(la),_lo(lo),_yaw(y),_l(length),_h(height){}
		
		Point2d getLeaf() const
		{
			Point2d leaf;
			//local
			double local_dN=_l*sin((360.0-_yaw)*pi/180.0);
			double local_dE=_l*cos((360.0-_yaw)*pi/180.0);
			//global
			double dN=local_dN*getLatiPerMeter();
			double dE=local_dE*getLongiPerMeter();

			leaf.x=_la+dN;
			leaf.y=_lo+dE;
			return leaf;
		}
		
		//l0 is reference 
		Point2d getCoor(const leaf_detection& l0) const
		{
			Point2d Coor;
			double edge_length=getDistanceOfTwoPoints(*this,l0);
			cout<<"edge_length: "<<edge_length<<endl;
			double d_height=l0._h-_h;
			Coor.x=sqrt(edge_length*edge_length-d_height*d_height);
			Coor.y=d_height;
			return Coor;
		}

		void print() const
		{
			cout.precision(9);
			cout<<"leaf latitude is: "<<getLeaf().x<<endl;
			cout<<"leaf longitude is: "<<getLeaf().y<<endl;
			cout<<"leaf longitude is: "<<_h<<endl;

		}

		static double getFlightAngle(const leaf_detection& l1,const leaf_detection& l2)
		{
			double dlo=l2._lo-l1._lo;
			double dla=l2._la-l1._la;
			return atan(dla/(dlo*cos(pi*l1._la/180.0)));
			//return atan(dla*cos(pi*l1._la/180.0)/dlo);
		}

		static double getFlightAngle(const double& la1,const double& lo1,const double& la2,const double& lo2)
		{
			double dlo=lo2-lo1;
			double dla=la2-la1;
			return atan(dla/(dlo*cos(pi*la1/180.0)));
			//return atan(dla*cos(pi*la1/180.0)/dlo);
		}

		void getVertex(const leaf_detection &leaf1);
		
        static void getVertex(const leaf_detection &leaf1,const leaf_detection &leaf2);//精确leaf坐标

		static void getVertexAll(const leaf_detection &leaf1,const leaf_detection &leaf2);//粗略leaf坐标

	private:
		double _la;
		double _lo;
		double _yaw;
		double _l;
		double _h;

		double getLatiPerMeter() const
		{
			return 180.0/(pi*eR);
		}

		double getLongiPerMeter() const
		{
			return 180.0/(pi*eR*cos(_la*pi/180.0));
		}

		double getMeterPerLati() const
		{
			return 1.0/getLatiPerMeter();
		}

		double getMeterPerLongi() const
		{
			return 1.0/getLongiPerMeter();
		}

		
		static void jingweidu2XYZ(double longitude ,double latitude ,double height ,pcl::PointXYZ &p)
		{
			double iPI = 0.0174532925199433;    // 3.1415296535898/180 弧度
			double a = 6378137.0;  // 长半轴
			double b = 6356752.3142;  //短半轴
		// double e = 0.00669437999013;  # 第二偏心率
			double f = 1.0 / 298.257223565;  // 第一扁率
			longitude = longitude * iPI;
			latitude = latitude * iPI;   // 纬度转换为弧度
			double e = sqrt( pow(a,2) - pow(b,2) ) / a;
			double W = sqrt( 1.0 - e * e * pow( sin(latitude),2) ) ; 
			double N = a / W;
			
			p.x = ( N + height ) * cos( latitude ) * cos( longitude );
			p.y = ( N + height ) * cos( latitude ) * sin( longitude ) ;
			p.z = ( N * (1 - pow(e,2)) + height ) * sin(latitude);
		}

		static double getDistanceOfTwoPoints(const Point2d &p1,const Point2d &p2)
		{
			return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
		}
		static double getDistanceOfTwoPoints(const Point2d &p2)
		{
			return sqrt(p2.x*p2.x+p2.y*p2.y);
		}

		static double getDistanceOfTwoPoints(const leaf_detection& l1,const leaf_detection& l2)
		{
			pcl::PointXYZ p1;
			jingweidu2XYZ(l1._lo,l1._la,l1._h,p1);
			pcl::PointXYZ p2;
			jingweidu2XYZ(l2._lo,l2._la,l2._h,p2);
			return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
		}

		static Point2d getCoor(const leaf_detection& l1,const leaf_detection& l2)
		{
			Point2d Coor;
			double edge_length=getDistanceOfTwoPoints(l1,l2);
			double d_height=l1._h-l2._h;
			Coor.x=sqrt(edge_length*edge_length-d_height*d_height);
			Coor.y=d_height;
			return Coor;
		}

};

void leaf_detection::getVertex(const leaf_detection &leaf1)
{
    Point3d Coor;
    Point2d Coor1(0,0);//set original point
    Point2d Coor2=getCoor(leaf1);

    double edge_length=getDistanceOfTwoPoints(Coor1,Coor2);
    double d_height=leaf1._h-_h;
    double alpha=asin(d_height/edge_length);

    //local
    double x=edge_length*cos(pi/3-alpha);
    double y=edge_length*sin(pi/3-alpha);

    //global
    Coor.z=leaf1._h+y;
    Coor.y=leaf1._lo+(_lo-leaf1._lo)*x/Coor2.x;
    Coor.x=leaf1._la+(_la-leaf1._la)*x/Coor2.x;

    cout<<"leaf latitude is: "<<Coor.x<<endl;
    cout<<"leaf longitude is: "<<Coor.y<<endl;
    cout<<"leaf height is: "<<Coor.z<<endl;
}

//global funciton
void leaf_detection::getVertex(const leaf_detection &leaf1,const leaf_detection &leaf2)
{
    Point3d Coor;
    Point2d Coor1(0,0);//set original point
    //Point2d Coor2=leaf2.getCoor(leaf1);
    Point2d Coor2=getCoor(leaf1,leaf2);

    //double edge_length=getDistanceOfTwoPoints(Coor1,Coor2);
	//double edge_length=getDistanceOfTwoPoints(Coor2);
	double edge_length=getDistanceOfTwoPoints(leaf1,leaf2);

    //double d_height=abs(leaf1._h-leaf2._h);
	double d_height=leaf1._h-leaf2._h;
    double alpha=asin(d_height/edge_length);

/* 
	Point2d Coor2;
	Coor2.y=leaf1._h-leaf2._h;
	Coor2.x=sqrt(edge_length*edge_length-d_height*d_height);
*/
    //local
    double x=edge_length*cos(pi/3-alpha);
    double y=edge_length*sin(pi/3-alpha);

    //global
    Coor.z=leaf1._h+y;
    Coor.y=leaf1._lo+(leaf2._lo-leaf1._lo)*x/Coor2.x;
    Coor.x=leaf1._la+(leaf2._la-leaf1._la)*x/Coor2.x;

	cout<<edge_length<<endl;

    cout<<"leaf latitude is: "<<Coor.x<<endl;
    cout<<"leaf longitude is: "<<Coor.y<<endl;
    cout<<"leaf height is: "<<Coor.z<<endl;
}

//global funciton
void leaf_detection::getVertexAll(const leaf_detection &leaf1,const leaf_detection &leaf2)
{
	//1.get leaf
	leaf1.print();
	leaf_detection l1;
	l1._la=leaf1.getLeaf().x;
	l1._lo=leaf1.getLeaf().y;
	l1._h=leaf1._h;
	leaf2.print();
	leaf_detection l2;
	l2._la=leaf2.getLeaf().x;
	l2._lo=leaf2.getLeaf().y;
	l2._h=leaf2._h;
	getVertex(l1,l2);
}
#endif