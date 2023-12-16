#include <iostream>
#include <string>
using namespace std;
/*
类模板：建立一个通用类，类中的成员 数据类型可以不具体制定，用一个虚拟的类型来代表。
语法：
	template<typename T>
	+ 类

类模板与函数模板区别主要有两点：
	类模板没有自动类型推导的使用方式
	类模板在模板参数列表中可以有默认参数
	类模板中成员函数类外实现时，需要加上模板参数列表
*/

// 类模板中成员函数创建时机是在调用阶段，导致分文件编写时链接不到
// #include "demo12_head.hpp"
// #include "demo12_head.cpp" //解决方式1，包含cpp源文件
// 解决方式2，将声明和实现写到一起，文件后缀名改为.hpp
#include "demo12_head.hpp"

// 1 类模板 模板参数列表可以指定默认参数
template <class NameType, class AgeType = int>
class Person
{
public:
	Person()
	{
		this->mName = "null";
		this->mAge = 0;
	}
	Person(NameType name, AgeType age)
	{
		this->mName = name;
		this->mAge = age;
	}
	// 成员函数类内声明
	void showPerson();

public:
	NameType mName;
	AgeType mAge;
};
// 类模板中成员函数类外实现时，需要加上模板参数列表
template <class T1, class T2>
void Person<T1, T2>::showPerson()
{
	cout << "姓名: " << this->mName << " 年龄:" << this->mAge << endl;
}

// 2 类模板中成员函数和普通类中成员函数创建时机是有区别的：
// 	普通类中的成员函数一开始就可以创建
// 	类模板中的成员函数在调用时才创建
class Person1
{
public:
	void showPerson1()
	{
		cout << "Person1 show" << endl;
	}
};

class Person2
{
public:
	void showPerson2()
	{
		cout << "Person2 show" << endl;
	}
};

template <class T>
class MyClass
{
public:
	T obj;

	// 类模板中的成员函数，并不是一开始就创建的，而是在模板调用时再生成
	void fun1() { obj.showPerson1(); }
	void fun2() { obj.showPerson2(); }
};

// 3 类模板对象做函数参数向函数传参的方式
// 3.1、指定传入的类型 — 直接显示对象的数据类型
void printPerson1(Person<string, int> &p)
{
	p.showPerson();
}
// 3.2、参数模板化 — 将对象中的参数变为模板进行传递
template <class T1, class T2>
void printPerson2(Person<T1, T2> &p)
{
	p.showPerson();
	// 查看模板推导出来的数据类型
	cout << "T1的类型为： " << typeid(T1).name() << endl;
	cout << "T2的类型为： " << typeid(T2).name() << endl;
}
// 3.3、整个类模板化 — 将这个对象类型 模板化进行传递
template <class T>
void printPerson3(T &p)
{
	cout << "T的类型为： " << typeid(T).name() << endl;
	p.showPerson();
}

// 4 当类模板碰到继承时，需要注意一下几点：
// 	当子类继承的父类是一个类模板时，子类在声明的时候，要指定出父类中T的类型,如果不指定，编译器无法给子类分配内存
// 	如果想灵活指定出父类中T的类型，子类也需变为类模板
// class Son1:public Person  //错误，c++编译需要给子类分配内存，必须知道父类中T的类型才可以向下继承
class Son1 : public Person<string, int> // 必须指定一个类型
{
};
// 类模板继承类模板 ,可以用T1,T2指定父类中的T1,T2类型
template <class T, class T1, class T2>
class Son2 : public Person<T1, T2>
{
public:
	Son2()
	{
		cout << typeid(T).name() << endl;
		cout << typeid(T1).name() << endl;
		cout << typeid(T2).name() << endl;
	}
};

int main()
{
	// 1 类模板没有自动类型推导的使用方式,指定NameType 为string类型，AgeType 为 int类型
	Person<string, int> P1("孙悟空", 999);
	P1.showPerson();
	Person<string, double> P2("ZARD", 55.9);
	P2.showPerson();

	// 2 类模板中的模板参数列表 可以指定默认参数
	Person<string> p3("猪八戒", 999);
	p3.showPerson();

	// 3 类模板中的成员函数是在模板调用时再生成
	MyClass<Person1> m;
	// 在调用fun1()时已经确认obj为showPerson1
	m.fun1();
	// m.fun2();//编译会出错，showPerson1没有此成员函数

	// 4 类模板对象做函数参数向函数传参的方式
	printPerson1(P1);
	printPerson2(P2);
	printPerson3(P1);

	// 5 类模板碰到继承时
	cout << endl;
	Son1 s1;
	s1.showPerson();
	Son2<char, string, int> child1;

	return 0;
}