#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>//OpenCV也提供了YAML文件扩展配置的功能，也就是FileStorage类
using namespace std;

/*
一般常用yaml-cpp和OpenCV进行解析。
相比yaml-cpp，OpenCV的优点是可以在YAML文件中存储矩阵，读出来就是cv::Mat格式；
缺点是OpenCV要求YAML文件有一个特殊的头，与标准的YAML文件并不兼容。
或者也可以理解为OpenCV定义了一种新的YAML格式。
*/

int main()
{
    cv::FileStorage fSettings("../config/demo14_test.yaml", cv::FileStorage::READ);
    string name= fSettings["name"], sex=fSettings["sex"];
    int age=fSettings["age"];
    cout << "name:" << name<< endl<< "sex:" << sex<< endl<< "age:" <<age<< endl;
    
    fSettings.release();
    
    return 0;
}
