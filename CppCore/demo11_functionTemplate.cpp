#include <iostream>
using namespace std;
/*
1.1 模板就是建立通用的模具，目的是为了提高复用性，将类型参数化
模板的特点：
	模板不可以直接使用，它只是一个框架
	模板的通用并不是万能的
1.2 函数模板
	C++另一种编程思想称为 ==泛型编程== ，主要利用的技术就是模板
	C++提供两种模板机制:函数模板和类模板
普通函数与函数模板区别：
	普通函数调用时可以发生自动类型转换（隐式类型转换）
	函数模板调用时，如果利用自动类型推导，不会发生隐式类型转换
	如果利用显示指定类型的方式，可以发生隐式类型转换
*/

// 函数模板作用：建立一个通用函数，其函数返回值类型和形参类型可以不具体制定，用一个虚拟的类型来代表。
// 语法：
// 	template<typename T>
// 	+ 函数声明或定义
// 		template — 声明创建模板
// 		typename — 表面其后面的符号是一种数据类型，可以用class代替
// 		T — 通用的数据类型，名称可以替换，通常为大写字母
template <typename T>
void mySwap(T &a, T &b)
{
	T temp = a;
	a = b;
	b = temp;
}

// 2、模板必须要确定出T的数据类型，才可以使用
template <class T>
void func()
{
	cout << "func 调用" << endl;
}

// 4 使用函数模板时，如果用自动类型推导，不会发生自动类型转换,即隐式类型转换
int myAdd(int a, int b)
{
	return a + b;
}
template <class T>
int myAddTemplate(T a, T b)
{
	cout << "调用的模板函数" << endl;
	return a + b;
}
// 5 普通函数与函数模板调用规则
int myAddTemplate(int a, int b)
{
	cout << "调用的普通函数" << endl;
	return a + b;
}
// 6 函数模板也可以发生重载
template <class T>
int myAddTemplate(T a, T b, T c)
{
	cout << "调用重载的模板" << endl;
	return a + b + c;
}

// 7 为特定的类型提供具体化的模板,具体化优先于常规模板
// 还有一种解决办法就是在类中重载运算符
class Person
{
public:
	Person(string name, int age)
	{
		this->m_Name = name;
		this->m_Age = age;
	}
	string m_Name;
	int m_Age;
};
// 显示具体化的原型和定意思以template<>开头，并通过名称来指出类型
template <>
int myAddTemplate(Person p1, Person p2)
{
	cout << "调用具体化的模板" << endl;
	return p1.m_Age + p2.m_Age;
}

int main()
{
	int a = 10;
	int b = 20;

	// 利用模板实现交换
	// 1 自动类型推导 自动类型推导，必须推导出一致的数据类型T,才可以使用
	mySwap(a, b);
	// 2 显示指定类型
	mySwap<int>(a, b);

	// 3、模板必须要确定出T的数据类型，才可以使用
	// func(); //错误，模板不能独立使用，必须确定出T的类型
	func<int>(); // 利用显示指定类型的方式，给T一个类型，才可以使用该模板

	// 4 使用函数模板时，如果用自动类型推导，不会发生自动类型转换,即隐式类型转换
	char c = 'c';
	// myAddTemplate(a, c); // 错误，推导不出一致的T类型,因为不会发生隐式类型转换
	// 使用普通函数，将char类型的'c'隐式转换为int类型  'c' 对应 ASCII码 99
	cout << myAdd(a, c) << endl;
	// 如果用显示指定类型，可以发生隐式类型转换
	cout << myAddTemplate<int>(a, c) << endl;
	// 引用时也无法发生隐式类型转换
	// mySwap<int>(a, c);

	cout << endl;
	// 5 如果函数模板和普通函数都可以实现，优先调用普通函数
	cout << myAddTemplate(a, b) << endl; // 调用普通函数
	// 但是可以通过空模板参数列表来强制调用函数模板
	cout << myAddTemplate<>(a, b) << endl; // 调用函数模板
	// 另外如果函数模板可以产生更好的匹配,优先调用函数模板
	char c1 = 'a', c2 = 'b';
	cout << myAddTemplate<>(c1, c2) << endl; // 调用函数模板

	// 6 函数模板也可以发生重载
	int t = 30;
	cout << myAddTemplate(a, b, t) << endl; // 调用重载的函数模板

	// 7 自定义数据类型，不会调用普通的函数模板
	Person p1("Tom", 10);
	Person p2("Bob", 20);
	// 可以创建具体化的Person数据类型的模板，用于特殊处理这个类型
	cout << myAddTemplate(p1, p2) << endl; // 调用重载的函数模板

	return 0;
}