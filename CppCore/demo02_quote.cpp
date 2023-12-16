#include <iostream>
using namespace std;
/*
引用的基本使用,给变量起别名,修改别名就修改原来的变量
语法： 数据类型 &别名 = 原名
引用必须初始化
引用在初始化后，不可以改变
*/

// 1. 值传递
void mySwap01(int a, int b)
{
	int temp = a;
	a = b;
	b = temp;
}

// 2. 地址传递
void mySwap02(int *a, int *b)
{
	int temp = *a;
	*a = *b;
	*b = temp;
}

// 3. 引用传递 通过引用参数产生的效果同按地址传递是一样的，但是引用的语法更清楚简单
void mySwap03(int &a, int &b)
{
	int temp = a;
	a = b;
	b = temp;
}

// 返回局部变量引用
int &test01()
{
	int a = 10; // 局部变量
	return a;
}

// 返回静态变量引用
int &test02()
{
	static int a = 20;
	return a;
}

// 发现是引用，转换为 int* const ref = &a;
void func(int &ref)
{
	ref = 100; // ref是引用，转换为*ref = 100
}

// 引用使用的场景，通常用来修饰形参
void showValue(const int &v)
{
	// v += 10;
	cout << v << endl;
}

int main()
{
	// 1 引用的使用
	int a = 10, c = 30;
	int &b = a;
	// int &c; //错误，引用必须初始化
	// int &b = c; //错误一旦初始化后，就不可以更改
	cout << "a = " << a << endl;
	cout << "b = " << b << endl;
	b = 100; // 这是赋值操作，不是更改引用
	// 引用改了，原变量也会改
	cout << "a = " << a << endl;
	cout << "b = " << b << endl;

	// 2 引用做函数参数：函数传参时，可以利用引用的技术让形参修饰实参，可以简化指针修改实参
	mySwap01(a, c);
	// 无法改变
	cout << "a:" << a << " c:" << c << endl;
	// 地址改变
	mySwap02(&a, &c);
	cout << "a:" << a << " c:" << c << endl;
	// 通过别名改变（换回来）
	mySwap03(a, c);
	cout << "a:" << a << " c:" << c << endl;

	// 3 引用做函数返回值，但是不要返回局部变量引用
	// 不能返回局部变量的引用
	int &ref1 = test01();
	// Segmentation fault (core dumped)
	// cout << "ref1 = " << ref1 << endl;
	int &ref2 = test02();
	cout << "ref2 = " << ref2 << endl;
	// 如果函数做左值，那么必须返回引用
	// 原因是返回的是全局变量的引用，相当于给全局变量赋值
	test02() = 1000;
	cout << "ref2 = " << ref2 << endl;

	// 4 引用的本质：引用的本质在c++内部实现是一个指针常量，但是所有的指针操作编译器都帮我们做了
	// 定义的时候自动转换为 int* const ref = &a; 指针常量是指针指向不可改，也说明为什么引用不可更改
	int &ref = a;
	ref = 20; // 内部发现ref是引用，自动帮我们转换为: *ref = 20;
	cout << "ref:" << ref << endl;
	// 发现是引用，转换为 int* const ref = &a;
	func(a);

	// 5 常量引用：主要用来修饰形参，防止误操作，在函数形参列表中，可以加const修饰形参，防止形参改变实参
	// int& ref3 = 10;  引用本身需要一个合法的内存空间，因此这行错误
	// 加入const就可以了，编译器优化代码，int temp = 10; const int& ref3 = temp;
	const int &ref3 = 10;
	// ref3 = 100;  //加入const后不可以修改变量
	cout << ref3 << endl;
	// 函数中利用常量引用防止误操作修改实参
	showValue(a);

	return 0;
}