#ifndef HEAD_H_
#define HEAD_H_
#include <iostream>
using namespace std;
#include <string>

template <class T1, class T2>
class People
{
public:
	People(T1 name, T2 age);
	void showPeople();

public:
	T1 m_Name;
	T2 m_Age;
};

// 构造函数 类外实现
template <class T1, class T2>
People<T1, T2>::People(T1 name, T2 age)
{
	this->m_Name = name;
	this->m_Age = age;
}

// 成员函数 类外实现
template <class T1, class T2>
void People<T1, T2>::showPeople()
{
	cout << "姓名: " << this->m_Name << " 年龄:" << this->m_Age << endl;
}

#endif