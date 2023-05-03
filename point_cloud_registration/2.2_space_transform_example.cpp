#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#define _USE_MATH_DEFINES

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) 
{
	//#####################################################
	// ��ת����ת���
	//#####################################################
	Eigen::AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1)); //���
	Eigen::Matrix3d rotation_matrix; //��ת����
	cout << "��Ƕ�Ӧ����ת����1Ϊ:" << endl << rotation_vector.matrix() << endl;
	cout << "��Ƕ�Ӧ����ת����2Ϊ:" << endl << rotation_vector.toRotationMatrix() << endl;

	//��ת������fromRotationMatrix()����������ת������ֵ
	AngleAxisd v2;
	v2.fromRotationMatrix(rotation_matrix);
	cout << "��ת�����Ӧ�����1Ϊ��" << endl << v2.axis().transpose() << " " << v2.angle() << endl;

	//ʹ����ת����������ת������ֵ
	AngleAxisd v3;
	v3 = rotation_matrix;
	cout << "��ת�����Ӧ�����2Ϊ��" << endl << v3.axis().transpose() << " " << v3.angle() << endl;

	//ʹ����ת����������ת�������г�ʼ��
	AngleAxisd v4(rotation_matrix);
	cout << "��ת�����Ӧ�����3Ϊ��" << endl << v4.axis().transpose() << " " << v4.angle() << endl;

	//#####################################################
	// ��ת����תŷ����
	//#####################################################
	Eigen::Vector3d eulerAngle;//ŷ����
	//��ת����תŷ����
	eulerAngle = rotation_matrix.eulerAngles(2, 1, 0); //˳��ZYX����
	cout << "yaw(z) pitch(y) roll(x)Ϊ  " << eulerAngle.transpose() << endl;

	//ŷ����ת��ת����
	Eigen::Matrix3d rotation_matrix1;
	rotation_matrix1 = Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitZ()) * 
		Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()) * 
		Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitX());
	cout << "ŷ����ת��ת����Ϊ��" << endl << rotation_matrix1 << endl;

	//#####################################################
	// ��ת����ת��Ԫ��
	//#####################################################
	Eigen::Quaterniond quaternion1(0.92388, 0, 0, 0.382683);
	cout << "��Ԫ��1" << endl << quaternion1.coeffs() << endl;

	//��Ԫ��ת��ת����
	Eigen::Matrix3d rotation_matrix2;
	rotation_matrix2 = quaternion1.matrix();
	Eigen::Matrix3d rotation_matrix3;
	rotation_matrix3 = quaternion1.toRotationMatrix();\
	cout << "��Ԫ��ת��ת����1 ��\n" << rotation_matrix2 << endl;
	cout << "��Ԫ��ת��ת����2 ��\n" << rotation_matrix3 << endl;

	//#####################################################
	// �������Ԫ��
	//#####################################################
	//���ת��Ԫ��
	Eigen::Quaterniond quaternion4(rotation_vector);
	cout << "���ת��Ԫ��1��" << endl << quaternion4.coeffs() << endl;
	Eigen::Quaterniond quaternion5;
	quaternion5 = rotation_vector;
	cout << "���ת��Ԫ��2��" << endl << quaternion5.coeffs() << endl;

	//��Ԫ��ת���
	Eigen::AngleAxisd V5(quaternion4);
	cout << "��Ԫ��ת���1��" << endl << V5.axis().transpose() << " " << V5.angle() << endl;
	Eigen::AngleAxisd V6;
	V6 = quaternion4;
	cout << "��Ԫ��ת���2��" << endl << V6.axis().transpose() << " " << V6.angle() << endl;

	//#####################################################
	// �������Ԫ��
	//#####################################################
	//���תŷ����
	Eigen::Vector3d eulerAngle1 = rotation_vector.matrix().eulerAngles(2, 1, 0);
	cout << "���תŷ���ǣ�" << endl << eulerAngle.transpose() << endl;

	Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle1(2), Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle1(1), Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle1(0), Vector3d::UnitZ()));
	Eigen::AngleAxisd rotation_vector1;
	rotation_vector1 = yawAngle * pitchAngle * rollAngle;
	cout << "ŷ����ת��ǣ�" << endl << rotation_vector1.axis().transpose() << " " << rotation_vector1.angle() << endl;

	//#####################################################
	// ŷ��������Ԫ��
	//#####################################################
	//ŷ����ת��Ԫ��
	Eigen::Quaterniond quaternion6;
	quaternion6 = Eigen::AngleAxisd(eulerAngle1[0], Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(eulerAngle1[1], Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(eulerAngle1[2], Eigen::Vector3d::UnitX());
	cout << "ŷ����ת��Ԫ����" << endl << quaternion6.coeffs() << endl;

	//��Ԫ��תŷ����
	Eigen::Vector3d eulerAngle2 = quaternion6.matrix().eulerAngles(2, 1, 0);
	cout << " ��Ԫ��תŷ���� ��" << endl << eulerAngle2.transpose() << endl;

	//#####################################################
	// �ռ�ת��
	//#####################################################
	//ŷʽ�任
	Eigen::Isometry3d T = Isometry3d::Identity();
	Eigen::Vector3d translation(1, 3, 4);//����ƽ������
	Eigen::AngleAxisd V(3.1415926 / 4, Eigen::Vector3d(1, 0, 1).normalized());//������ת����Ǳ�ʾ
	T.rotate(V);//������ת����
	T.pretranslate(translation);//�൱����ת�������ƽ��,�൱����ƽ�ƺ���ת
	cout << " ��ת����Ϊ ��" << endl << T.matrix() << std::endl;

	T.rotate(V);//������ת����
	T.translate(translation);//�൱����ת�����ҳ�ƽ�ƣ��൱������ת��ƽ��
	cout << " ��ת����Ϊ ��" << endl << T.matrix() << std::endl;

	Eigen::Vector3d p1(2, 3, 4);
	auto p2 = T * p1;
	std::cout << "p2����Ϊ��" << p2.transpose() << std::endl;

	//����任
	Eigen::Affine3d A = Affine3d::Identity();
	cout << " ����任����Ϊ ��" << endl << A.matrix() << std::endl;
	//��Ӱ�任
	Eigen::Projective3d P = Projective3d::Identity();
	cout << " ��Ӱ�任����Ϊ ��" << endl << A.matrix() << std::endl;


}