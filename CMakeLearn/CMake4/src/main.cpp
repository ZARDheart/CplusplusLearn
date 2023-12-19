#include <iostream>
#include <Eigen/Core> 
#include <opencv4/opencv2/opencv.hpp>
#include "testFunc.h"
#include "testFunc1.h"
using namespace Eigen;
using namespace cv;
using namespace std;

int main()
{
	cout<<"First CMake"<<endl;
	func(100);
    func1(100);
    
    Matrix<float, 2, 3> matrix_23;
    MatrixXd m = MatrixXd::Random(3, 3);
    m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;
    cout << "m =" << endl << m << endl;
    
    Mat I= imread("/home/zard/Pictures/back.jpeg"); 
    cout << "图像宽为" << I.cols << ",高为" << I.rows<<",通道数为" << I.channels() << endl;
    imshow("源图像",I);
    waitKey(0); 
    
	return 0;
}
