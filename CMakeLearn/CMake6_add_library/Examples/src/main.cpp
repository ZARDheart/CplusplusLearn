#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>//OpenCV也提供了YAML文件扩展配置的功能，也就是FileStorage类
#include "testFunc.h"
#include "testFunc1.h"
#include "hello.h"
using namespace Eigen;
using namespace cv;
using namespace std;

int main()
{
    HelloFunc();
    HelloFunc2(23);
	cout<<"First CMake"<<endl;
	func(100);
    func1(100);
    
    Matrix<float, 2, 3> matrix_23;
    MatrixXd m = MatrixXd::Random(3, 3);
    m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;
    cout << "m =" << endl << m << endl;
    
    FileStorage fSettings("../config/est.yaml", FileStorage::READ);
    string name= fSettings["name"], sex=fSettings["sex"];
    int age=fSettings["age"];
    cout << "name:" << name<< endl<< "sex:" << sex<< endl<< "age:" <<age<< endl;
    fSettings.release();
    
    Mat I= imread("/home/zrad/Pictures/back.jpeg"); 
    cout << "图像宽为" << I.cols << ",高为" << I.rows<<",通道数为" << I.channels() << endl;
    imshow("源图像",I);
    waitKey(0); 
    
	return 0;
}
