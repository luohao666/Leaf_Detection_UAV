 #include <iostream>
 #include <math.h>
 #include <stdio.h>

#define PCL

using namespace std;
#ifdef PCL

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#endif



#define DISTANCE 0.1


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

#ifdef PCL
//坐标系箭头 
void show_axis(pcl::visualization::PCLVisualizer  &viewer)
{
	float len = 20;
	pcl::PointXYZ O,X,Y,Z;
	O.x = 0; O.y = 0; O.z = 0;
	X.x = len; X.y = 0; X.z = 0;
	Y.x = 0; Y.y = len; Y.z = 0;
	Z.x = 0; Z.y = 0; Z.z = len;
 
	//添加箭头
	viewer.addArrow<pcl::PointXYZ>( X,O, 50, 50, 50, false, "X", 0);
	viewer.addArrow<pcl::PointXYZ>( Y,O, 50, 50, 50, false, "Y", 0);
	viewer.addArrow<pcl::PointXYZ>( Z,O, 50, 50, 50, false, "Z", 0);
}
#endif


void jingweidu2XYZ(double longitude ,double latitude ,double height ,pcl::PointXYZ &p)
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

void XYZ2jingweidu(pcl::PointXYZ p ,double &longitude ,double &latitude ,double &height  )
{
	double a = 6378137.0;  // 长半轴
    double b = 6356752.3142;  //短半轴
    double iPI = 0.0174532925199433;    // 3.1415296535898/180 弧度
	double e = ( pow(a,2) - pow(b,2) ) / pow(a,2);   // 第一偏心率
    double e2 = ( pow(a,2) - pow(b,2) ) / pow(b,2);  // 第二偏心率
    double q = atan( p.z * a / ( sqrt(p.x * p.x + p.y * p.y) * b ) );  
    //W1 = sqrt( 1.0 - e  * pow(sin(Lat),2 ) )  
   // N = a / W1
    double Lon = atan(p.y / p.x);
    double Lat = atan(p.z / sqrt(p.x * p.x + p.y * p.y));
    double A0 = 0;
    double N = 0;
	double B = 0;
    while (fabs(A0 - Lat ) > 0.000000008)
	{
        A0 = Lat ;
        N = a / sqrt( 1.0 - e  * pow(sin(Lat),2 ) ); 
        B = atan( (p.z + N * e * sin(Lat)) / sqrt(p.x * p.x + p.y * p.y) ) ;
        Lat = B ;
	} 

    //Hei = sqrt(p.x * p.x + p.y * p.y) / cos(Lat) - N 
    double Hei = p.y / fabs(( cos(Lat) * sin(Lon) )) - N ;
    //print( sqrt(p.x * p.x + Y * Y) / cos(Lat))
    Lon = Lon / iPI;
    Lat = Lat / iPI;
    if (Lon < 0)
        Lon = Lon + 180;
	longitude = Lon;
	latitude = Lat;
	height = Hei;
}


int main(void)
{
	pcl::PointXYZ p;
	double lat = 111.071574508;
	double log = 31.716409620;
	double hei = 1667.196;
	for(int i = 0; i < 10; i++)
	{
		jingweidu2XYZ(lat,log,hei,p);
		//std::cout.precision(7);
		//std::cout << p.x << p.y << p.z << std::endl;
		printf("i=%d \r\n",i);
		printf("p.x = %.7f , p.y = %.7f , p.z = %.7f \n",p.x,p.y,p.z );
		XYZ2jingweidu(p,lat,log,hei);
		printf("lat = %.7f , log = %.7f , hei = %.7f\n",lat,log,hei );
	}
	

	//std::cout.precision(7);
	//std::cout << lat << log << hei <<std::endl;
	return 0;
}
/*

int main()
{
#ifdef PCL
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//指针
	pcl::visualization::PCLVisualizer viewer("Test");
	viewer.setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
 
	show_axis(viewer);
#endif
	pcl::PointXYZ p[3] ,o; //三个顶点坐标p[0],p[1],p[2]

	p[1].x = 4; p[1].y = 0; p[1].z = 0;
	p[2].x = 0; p[2].y = 10; p[2].z = 20;
	p[0].x = 20; p[0].y = 20; p[0].z = 0;

	//计算重心o
	o.x = o.y = o.z = 0;
	for(int i = 0; i < 3; i++)
	{
		o.x += p[i].x;
		o.y += p[i].y;
		o.z += p[i].z;
	}
	o.x /= 3;o.y /= 3;o.z /= 3;
	std::cout << "Center : o.x = " << o.x << ",  o.y = " << o.y << ",  o.z = " << o.z << endl; 

	//计算重心到三个顶点的距离op[0],op[1],op[2]
	double op[3];
	for(int i = 0; i < 3; i++)
	{
		op[i] = sqrt((o.x-p[i].x)*(o.x-p[i].x) + (o.y-p[i].y)*(o.y-p[i].y) + (o.y-p[i].y)*(o.y-p[i].y));
		std::cout << "op" << i << ".len = " << op[i] << endl; 
	}

	//顶点用球标识，用于pcl库 球半径跟随三角形大小自适应
	double radius = (op[0] + op[1] + op[2])/100;

	//计算三个顶点到达重心连线的延长线上，p'处的坐标。延长的距离是op[i]的DISTANCE倍
	pcl::PointXYZ p_[3];
	for(int i = 0; i < 3; i++)
	{
		p_[i].x = o.x - (p[i].x - o.x) * DISTANCE;
		p_[i].y = o.y - (p[i].y - o.y) * DISTANCE;
		p_[i].z = o.z - (p[i].z - o.z) * DISTANCE;

#ifdef PCL
		std::string line = "line" + std::to_string(i);
		viewer.addLine(o, p[i], 0, 200, 0, line, 0);
		std::string sphere_p = "sphere_p" + std::to_string(i);
		viewer.addSphere(p[i],radius,0, 100, 0, sphere_p, 0);
		std::string sphere_p_ = "sphere_p_" + std::to_string(i);
		viewer.addSphere(p_[i],radius/2,0, 0, 100, sphere_p_, 0);
#endif
	}

	//沿着op[i]平行的方向，左右两边各增加一条以p'为起点的线段，线段长度等于op[i]
	pcl::PointXYZ pa[3],pb[3];
	for(int i = 0; i < 3; i++)
	{

		pa[i].x = p_[(i+2)%3].x + (p[i].x - o.x) ;
		pa[i].y = p_[(i+2)%3].y + (p[i].y - o.y) ;
		pa[i].z = p_[(i+2)%3].z + (p[i].z - o.z) ;

		pb[i].x = p_[(i+1)%3].x + (p[i].x - o.x) ;
		pb[i].y = p_[(i+1)%3].y + (p[i].y - o.y) ;
		pb[i].z = p_[(i+1)%3].z + (p[i].z - o.z) ;
#ifdef PCL
		// std::string sphere_pa = "sphere_pa" + std::to_string(i);
		// viewer.addSphere(pa[i],radius/2,0, 0, 100, sphere_pa, 0);

		// std::string sphere_pb = "sphere_pb" + std::to_string(i);
		// viewer.addSphere(pb[i],radius/2,0, 0, 100, sphere_pb, 0);

		// std::string lineab = "lineab" + std::to_string(i);
		// viewer.addLine(pa[i], pb[i], 0, 200, 200, lineab, 0);

		// std::string linea_ = "linea_" + std::to_string(i);
		// viewer.addLine(pa[i], p_[(i+2)%3], 0, 200, 200, linea_, 0);

		// std::string lineb_ = "lineb_" + std::to_string(i);
		// viewer.addLine(pb[i], p_[(i+1)%3], 0, 200, 200, lineb_, 0);
#endif
	}

#ifdef PCL
  	viewer.addCoordinateSystem();
	// // viewer.removeAllPointClouds();//删除所有点
	// //viewer.removeAllShapes();移除形状
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(1);
	}
#endif
}


*/
