#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
using namespace std;
/*
C++程序在执行时，将内存大方向划分为4个区域
代码区：存放函数体的二进制代码，由操作系统进行管理的
全局区：存放全局变量和静态变量以及常量
栈区：由编译器自动分配释放, 存放函数的参数值,局部变量等
堆区：由程序员分配和释放,若程序员不释放,程序结束时由操作系统回收
内存四区意义：不同区域存放的数据，赋予不同的生命周期, 给我们更大的灵活编程
*/

// 全局变量
/// 外部链接静态变量（其他文件可访问，例如本文件是头文件）
int g_a = 10;
int g_b = 10;
// 全局常量
const int c_g_a = 10;
const int c_g_b = 10;

static double pi = 3.1415926; ///< 内部链接静态变量（只在当前文件访问）,定义并初始化

extern double e1 = 1.23456; // 外部链接静态变量（其他文件可访问,使用他的文件要声明：extern double e1;）

// 函数局部变量，返回的是野指针
int *func()
{
	int a = 10 + pi;
	return &a;
}

// 堆区是可以返回的
int *func2()
{
	int *a = new int(10);
	return a;
}

int main()
{
	// 1 局部区
	// 局部变量
	int a = 10;
	int b = 10;
	// 打印地址
	cout << "局部变量a地址为： " << (long long)&a << endl;
	cout << "局部变量b地址为： " << (long long)&b << endl;
	// 局部常量
	const int c_l_a = 10;
	const int c_l_b = 10;
	cout << "局部常量c_l_a地址为： " << (long long)&c_l_a << endl;
	cout << "局部常量c_l_b地址为： " << (long long)&c_l_b << endl;

	// 2 全局区：该区域的数据在程序结束后由操作系统释放
	// 全局变量
	cout << "全局变量g_a地址为： " << (long long)&g_a << endl;
	cout << "全局变量g_b地址为： " << (long long)&g_b << endl;
	// 静态变量也在全局区中
	static int s_a = 10;
	static int s_b = 10;
	cout << "静态变量s_a地址为： " << (long long)&s_a << endl;
	cout << "静态变量s_b地址为： " << (long long)&s_b << endl;
	// 全局区还包含了常量区, 字符串常量和其他常量也存放在此
	cout << "字符串常量地址为： " << (long long)&"hello world" << endl;
	cout << "字符串常量地址为： " << (long long)&"hello world1" << endl;
	cout << "全局常量c_g_a地址为： " << (long long)&c_g_a << endl;
	cout << "全局常量c_g_b地址为： " << (long long)&c_g_b << endl;

	// 3 栈区：​ 由编译器自动分配释放, 存放函数的参数值,局部变量等
	// 注意事项：不要返回局部变量的地址，栈区开辟的数据由编译器自动释放
	int *p = func();
	// 返回的是个已经被释放的野指针，会报段错误Segmentation fault (core dumped)
	// cout << *p << endl;

	// 4 堆区：由程序员分配释放,若程序员不释放,程序结束时由操作系统回收，在C++中主要利用new在堆区开辟内存
	int *p2 = func2();
	cout << "堆区数据不会被自动释放：" << *p2 << endl;
	// 利用delete释放堆区数据
	delete p2;

	// 5 new操作符：C++中利用new操作符在堆区开辟数据
	// 	堆区开辟的数据，由程序员手动开辟，手动释放，释放利用操作符 delete
	// 	语法： new 数据类型
	// 	利用new创建的数据，会返回该数据对应的类型的指针
	int *pn = new int; // 使用New找到规定内存大小的地址并返回
	*pn = 10001;
	cout << *pn << " " << pn << endl;
	delete pn; // 释放内存
	// cout << *p << endl; //报错，释放的空间不可访问
	// 创建动态数组
	int *ps = new int[5];
	ps[0] = 100;
	ps[1] = 101;
	cout << &ps[0] << ":" << ps[0] << endl;
	cout << &ps[1] << ":" << ps[1] << endl;
	cout << ps + 1 << ":" << *(ps + 1) << endl; // 数组名代表的是第一个元素的地址，指针变量加一后，指针增加的量是他指向数据类型的字节数
	// 数组和指针基本等价
	int pd[5] = {1, 2, 0, 0, 0};
	cout << &pd[0] << ":" << pd[0] << endl;
	cout << &pd[1] << ":" << pd[1] << endl;
	cout << pd + 1 << ":" << *(pd + 1) << endl; // 再次说明：pd=&pd[0],*(pd+i)=pd[i],pd与ps的区别是pd为常量，不可修改
	// 释放数组 delete 后加 []
	delete[] ps;

	// 智能指针
	// #include <boost/shared_ptr.hpp>
	// #include <boost/make_shared.hpp>
	boost::shared_ptr<string> ptr1 = boost::make_shared<string>("hello");
	cout << *ptr1 << endl;

	return 0;
}