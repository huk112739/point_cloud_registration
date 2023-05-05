#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace Eigen;
using namespace std;

int main(int argc, char** argv )
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

	cout << "Cloud_in" << Cloud_in->width << Cloud_in->height << endl;

	Cloud_in->width = 3;
	Cloud_in->height = 1;
	Cloud_in->is_dense = false;
	Cloud_in->resize(Cloud_in->width * Cloud_in->height);

	Cloud_out->width = 3;
	Cloud_out->height = 1;
	Cloud_out->is_dense = false;
	Cloud_out->resize(Cloud_out->width * Cloud_out->height);

	MatrixXd m = MatrixXd::Random(6, 3);
	fstream f;
	f.open("zd.txt",ios::out);
	f << m << endl;
	FILE* fRead;
	fRead = fopen("zd.txt", "r");

	cout << "cloud_in : " << endl;

	for (int i = 0; i < Cloud_in->points.size(); i++)
	{
		double pt[3];
		fscanf(fRead, "%lf %lf %lf", pt, pt + 1, pt + 2);
		Cloud_in->points[i].x = pt[0];
		Cloud_in->points[i].y = pt[1];
		Cloud_in->points[i].z = pt[2];

		cout << Cloud_in->points[i].x << "\t" << Cloud_in->points[i].y << "\t" << Cloud_in->points[i].z << endl;
	}

	cout << "cloud_out : " << endl;
	for (int i = 0; i < Cloud_out->points.size(); i++)
	{
		double pt[3];
		fscanf(fRead, "%lf %lf %lf", pt, pt + 1, pt + 2);
		Cloud_out->points[i].x = pt[0];
		Cloud_out->points[i].y = pt[1];
		Cloud_out->points[i].z = pt[2];

		cout << Cloud_out->points[i].x << "  \t" << Cloud_out->points[i].y << "  \t" << Cloud_out->points[i].z << endl;
	}

	//---------------------------------利用SVD方法求解变换矩阵--------------------------
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
	TESVD.estimateRigidTransformation(*Cloud_in, *Cloud_out, transformation2);

	// -----------------------------------输出变换矩阵信息------------------------------
	cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << endl;
	printf("\n");
	printf("    | %6.3f %6.3f %6.3f | \n", transformation2(0, 0), transformation2(0, 1), transformation2(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", transformation2(1, 0), transformation2(1, 1), transformation2(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", transformation2(2, 0), transformation2(2, 1), transformation2(2, 2));
	printf("\n");
	printf("t = < %0.3f, %0.3f, %0.3f >\n", transformation2(0, 3), transformation2(1, 3), transformation2(2, 3));

	return 0;


}