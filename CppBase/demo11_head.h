//避免多次引用此头文件
#ifndef HEAD_H_
#define HEAD_H_
#include <iostream>
#include <cstring>
using namespace std;
/*
函数的分文件编写：让代码结构更加清晰，一般有4个步骤
    创建后缀名为.h的头文件
    创建后缀名为.cpp的源文件
    在头文件中写函数的声明
    在源文件中写函数的定义
在其他源文件中调用头文件
*/

struct point
{
    double x;
    double y;
    double z;
};
//函数声明
double avange(double arr[],int n);         //arr[],指出arr是指针，等价于 *arr：
//double avange(double *arr,int n);
//double avange(const double arr[],int n); //定义为常量数组函数将不能修改数组
double sum(double arr[][3],int n);         //二维数组参数
//double sum(double (*arr)[3],int n);      //与上句等价
// 将一个点平移trans[3]
point translation(point p,double trans[3]);
#endif
