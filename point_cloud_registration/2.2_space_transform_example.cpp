#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#define _USE_MATH_DEFINES

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) 
{
	//#####################################################
	// 旋转矩阵转轴角
	//#####################################################
	Eigen::AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1)); //轴角
	Eigen::Matrix3d rotation_matrix; //旋转矩阵
	cout << "轴角对应的旋转矩阵1为:" << endl << rotation_vector.matrix() << endl;
	cout << "轴角对应的旋转矩阵2为:" << endl << rotation_vector.toRotationMatrix() << endl;

	//旋转向量的fromRotationMatrix()函数来对旋转向量赋值
	AngleAxisd v2;
	v2.fromRotationMatrix(rotation_matrix);
	cout << "旋转矩阵对应的轴角1为：" << endl << v2.axis().transpose() << " " << v2.angle() << endl;

	//使用旋转矩阵来对旋转向量赋值
	AngleAxisd v3;
	v3 = rotation_matrix;
	cout << "旋转矩阵对应的轴角2为：" << endl << v3.axis().transpose() << " " << v3.angle() << endl;

	//使用旋转矩阵来对旋转向量进行初始化
	AngleAxisd v4(rotation_matrix);
	cout << "旋转矩阵对应的轴角3为：" << endl << v4.axis().transpose() << " " << v4.angle() << endl;

	//#####################################################
	// 旋转矩阵转欧拉角
	//#####################################################
	Eigen::Vector3d eulerAngle;//欧拉角
	//旋转矩阵转欧拉角
	eulerAngle = rotation_matrix.eulerAngles(2, 1, 0); //顺序ZYX内旋
	cout << "yaw(z) pitch(y) roll(x)为  " << eulerAngle.transpose() << endl;

	//欧拉角转旋转矩阵
	Eigen::Matrix3d rotation_matrix1;
	rotation_matrix1 = Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitZ()) * 
		Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()) * 
		Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitX());
	cout << "欧拉角转旋转矩阵为：" << endl << rotation_matrix1 << endl;

	//#####################################################
	// 旋转矩阵转四元数
	//#####################################################
	Eigen::Quaterniond quaternion1(0.92388, 0, 0, 0.382683);
	cout << "四元数1" << endl << quaternion1.coeffs() << endl;

	//四元数转旋转矩阵
	Eigen::Matrix3d rotation_matrix2;
	rotation_matrix2 = quaternion1.matrix();
	Eigen::Matrix3d rotation_matrix3;
	rotation_matrix3 = quaternion1.toRotationMatrix();\
	cout << "四元数转旋转矩阵1 ：\n" << rotation_matrix2 << endl;
	cout << "四元数转旋转矩阵2 ：\n" << rotation_matrix3 << endl;

	//#####################################################
	// 轴角与四元数
	//#####################################################
	//轴角转四元素
	Eigen::Quaterniond quaternion4(rotation_vector);
	cout << "轴角转四元数1：" << endl << quaternion4.coeffs() << endl;
	Eigen::Quaterniond quaternion5;
	quaternion5 = rotation_vector;
	cout << "轴角转四元数2：" << endl << quaternion5.coeffs() << endl;

	//四元数转轴角
	Eigen::AngleAxisd V5(quaternion4);
	cout << "四元数转轴角1：" << endl << V5.axis().transpose() << " " << V5.angle() << endl;
	Eigen::AngleAxisd V6;
	V6 = quaternion4;
	cout << "四元数转轴角2：" << endl << V6.axis().transpose() << " " << V6.angle() << endl;

	//#####################################################
	// 轴角与四元数
	//#####################################################
	//轴角转欧拉角
	Eigen::Vector3d eulerAngle1 = rotation_vector.matrix().eulerAngles(2, 1, 0);
	cout << "轴角转欧拉角：" << endl << eulerAngle.transpose() << endl;

	Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle1(2), Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle1(1), Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle1(0), Vector3d::UnitZ()));
	Eigen::AngleAxisd rotation_vector1;
	rotation_vector1 = yawAngle * pitchAngle * rollAngle;
	cout << "欧拉角转轴角：" << endl << rotation_vector1.axis().transpose() << " " << rotation_vector1.angle() << endl;

	//#####################################################
	// 欧拉角与四元数
	//#####################################################
	//欧拉角转四元数
	Eigen::Quaterniond quaternion6;
	quaternion6 = Eigen::AngleAxisd(eulerAngle1[0], Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(eulerAngle1[1], Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(eulerAngle1[2], Eigen::Vector3d::UnitX());
	cout << "欧拉角转四元数：" << endl << quaternion6.coeffs() << endl;

	//四元数转欧拉角
	Eigen::Vector3d eulerAngle2 = quaternion6.matrix().eulerAngles(2, 1, 0);
	cout << " 四元数转欧拉角 ：" << endl << eulerAngle2.transpose() << endl;

	//#####################################################
	// 空间转换
	//#####################################################
	//欧式变换
	Eigen::Isometry3d T = Isometry3d::Identity();
	Eigen::Vector3d translation(1, 3, 4);//设置平移向量
	Eigen::AngleAxisd V(3.1415926 / 4, Eigen::Vector3d(1, 0, 1).normalized());//设置旋转的轴角表示
	T.rotate(V);//设置旋转部分
	T.pretranslate(translation);//相当于旋转矩阵左乘平移,相当于先平移后旋转
	cout << " 旋转矩阵为 ：" << endl << T.matrix() << std::endl;

	T.rotate(V);//设置旋转部分
	T.translate(translation);//相当于旋转矩阵右乘平移，相当于先旋转后平移
	cout << " 旋转矩阵为 ：" << endl << T.matrix() << std::endl;

	Eigen::Vector3d p1(2, 3, 4);
	auto p2 = T * p1;
	std::cout << "p2坐标为：" << p2.transpose() << std::endl;

	//仿射变换
	Eigen::Affine3d A = Affine3d::Identity();
	cout << " 仿射变换矩阵为 ：" << endl << A.matrix() << std::endl;
	//射影变换
	Eigen::Projective3d P = Projective3d::Identity();
	cout << " 射影变换矩阵为 ：" << endl << A.matrix() << std::endl;


}