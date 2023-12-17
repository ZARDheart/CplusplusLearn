#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>//OpenCV也提供了YAML文件扩展配置的功能，也就是FileStorage类
using namespace std;

int main()
{
    cv::FileStorage fSettings("../config/demo14_test.yaml", cv::FileStorage::READ);
    string name= fSettings["name"], sex=fSettings["sex"];
    int age=fSettings["age"];
    cout << "name:" << name<< endl<< "sex:" << sex<< endl<< "age:" <<age<< endl;
    
    fSettings.release();
    
    return 0;
}
