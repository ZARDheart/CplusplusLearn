#include "demo12_head.hpp"
/*
类模板中成员函数创建时机是在调用阶段，导致分文件编写时链接不到
解决：
	直接包含.cpp源文件
	将声明和实现写到同一个文件中，并更改后缀名为.hpp，hpp是约定的名称，并不是强制
*/

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
