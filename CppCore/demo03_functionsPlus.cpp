#include <iostream>
#include <cstring>
#include <vector>
#include <cstdlib> //控制程序
using namespace std;
/*
1 函数默认参数:在C++中，函数的形参列表中的形参是可以有默认值的。
    语法： 返回值类型 函数名 （参数= 默认值）{}
2 函数占位参数:C++中函数的形参列表里可以有占位参数，用来做占位，调用函数时必须填补该位置
    语法： 返回值类型 函数名 (数据类型){}

3 函数重载：函数名可以相同，提高复用性
函数重载满足条件：
    同一个作用域下
    函数名称相同
    函数参数类型不同 或者 个数不同 或者 顺序不同
注意: 函数的返回值不可以作为函数重载的条件
*/

// 1 函数默认参数
//  如果某个位置参数有默认值，那么从这个位置往后，从左向右，必须都要有默认值
int func2(int a, int b = 10, int c = 10);
//  如果函数声明有默认值，函数实现的时候就不能有默认参数
int func2(int a, int b, int c)
{
    return a + b + c;
}
int fact(int n, int a = 1) // 定义递归函数，设置默认参数
{
    int fa = n;
    n--;
    if (n > 0)
        // 后面递归的时候，默认a=1
        fa = fa * fact(n);
    return a * fa;
}

// 2 函数占位参数
void func3(int a, int)
{
    cout << "this is func3" << endl;
}
// 占位参数也可以有默认参数
void func4(int a, int = 10)
{
    cout << "this is func4" << endl;
}

// 3 函数重载：函数名可以相同，提高复用性
// 函数重载需要函数都在同一个作用域下
void func()
{
    cout << "func 的调用！" << endl;
}
void func(int a)
{
    cout << "func (int a) 的调用！" << endl;
}
void func(double a)
{
    cout << "func (double a)的调用！" << endl;
}
void func(int a, double b)
{
    cout << "func (int a ,double b) 的调用！" << endl;
}
void func(double a, int b)
{
    cout << "func (double a ,int b)的调用！" << endl;
}
// 函数返回值不可以作为函数重载条件
// int func(double a, int b)
//{
//	cout << "func (double a ,int b)的调用！" << endl;
// }

// 函数重载注意事项
// 1、引用作为重载条件
void func0(int &a) // 输入变量
{
    cout << "func (int &a) 调用 " << endl;
}
void func0(const int &a) // 输入常量
{
    cout << "func (const int &a) 调用 " << endl;
}

// 2、函数重载碰到函数默认参数
void func1(int a, int b = 10)
{
    cout << "func2(int a, int b = 10) 调用" << endl;
}
void func1(int a)
{
    cout << "func2(int a) 调用" << endl;
}

int main()
{
    // 1 函数默认参数
    cout << "ret = " << func2(20, 20, 20) << endl;
    cout << "ret = " << func2(100) << endl;
    cout << "5!*2=" << fact(5, 2) << endl; // 函数递归
    cout << "3!=" << fact(3) << endl;      // 使用默认参数，传入参数时覆盖

    // 2 函数占位参数
    func3(10, 10); // 占位参数必须填补

    // 3 函数重载
    func();
    func(10);
    func(3.14);
    func(10, 3.14);
    func(3.14, 10);
    // 函数重载注意事项
    int a = 10;
    // 变量
    func0(a);
    // 常量
    func0(10);
    // func1(10); //碰到默认参数产生歧义，需要避免
    func1(10, 20);

    // 宏定义函数
    int b = 2;
#define sumab(i) i *(a + b)
    cout << sumab(3) << endl;
#undef sumab // 取消宏定义

    return 0;
}