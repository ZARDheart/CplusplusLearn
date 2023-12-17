#ifndef HEAD_H_
#define HEAD_H_
#include <iostream>
#include <string>
using namespace std;

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

// 类模板实现一个通用的数组类，要求如下：
// 	可以对内置数据类型以及自定义数据类型的数据进行存储
// 	将数组中的数据存储到堆区
// 	构造函数中可以传入数组的容量
// 	提供对应的拷贝构造函数以及operator=防止浅拷贝问题
// 	提供尾插法和尾删法对数组中的数据进行增加和删除
// 	可以通过下标的方式访问数组中的元素
// 	可以获取数组中当前元素个数和数组的容量
template <class T>
class MyArray
{
public:
	// 构造函数 容量
	MyArray(int capacity)
	{
		this->m_Capacity = capacity;
		this->m_Size = 0;
		// 按照容量开辟空间
		pAddress = new T[this->m_Capacity];
	}

	// 拷贝构造
	MyArray(const MyArray &arr)
	{
		this->m_Capacity = arr.m_Capacity;
		this->m_Size = arr.m_Size;
		this->pAddress = new T[this->m_Capacity];
		for (int i = 0; i < this->m_Size; i++)
		{
			// 如果T为对象，而且还包含指针，必须需要重载 = 操作符，因为这个等号不是 构造 而是赋值，
			//  普通类型可以直接= 但是指针类型需要深拷贝
			this->pAddress[i] = arr.pAddress[i];
		}
	}

	// 重载= 操作符  防止浅拷贝问题
	MyArray &operator=(const MyArray &myarray)
	{

		if (this->pAddress != NULL)
		{
			delete[] this->pAddress;
			this->m_Capacity = 0;
			this->m_Size = 0;
		}

		this->m_Capacity = myarray.m_Capacity;
		this->m_Size = myarray.m_Size;
		this->pAddress = new T[this->m_Capacity];
		for (int i = 0; i < this->m_Size; i++)
		{
			this->pAddress[i] = myarray[i];
		}
		return *this;
	}

	// 重载[] 操作符  arr[0]
	T &operator[](int index)
	{
		return this->pAddress[index]; // 不考虑越界，用户自己去处理
	}

	// 尾插法
	void Push_back(const T &val)
	{
		if (this->m_Capacity == this->m_Size)
		{
			return;
		}
		this->pAddress[this->m_Size] = val;
		this->m_Size++;
	}

	// 尾删法
	void Pop_back()
	{
		if (this->m_Size == 0)
		{
			return;
		}
		this->m_Size--;
	}

	// 获取数组容量
	int getCapacity()
	{
		return this->m_Capacity;
	}

	// 获取数组大小
	int getSize()
	{
		return this->m_Size;
	}

	// 析构
	~MyArray()
	{
		if (this->pAddress != NULL)
		{
			delete[] this->pAddress;
			this->pAddress = NULL;
			this->m_Capacity = 0;
			this->m_Size = 0;
		}
	}

private:
	T *pAddress;	// 指向一个堆空间，这个空间存储真正的数据
	int m_Capacity; // 容量
	int m_Size;		// 大小
};

void printIntArray(MyArray<int> &arr)
{
	for (int i = 0; i < arr.getSize(); i++)
	{
		cout << arr[i] << " ";
	}
	cout << endl;
}

// 测试内置数据类型
void test01()
{
	MyArray<int> array1(10);
	for (int i = 0; i < 10; i++)
	{
		array1.Push_back(i);
	}
	cout << "array1打印输出：" << endl;
	printIntArray(array1);
	cout << "array1的大小：" << array1.getSize() << endl;
	cout << "array1的容量：" << array1.getCapacity() << endl;

	cout << "--------------------------" << endl;

	MyArray<int> array2(array1);
	array2.Pop_back();
	cout << "array2打印输出：" << endl;
	printIntArray(array2);
	cout << "array2的大小：" << array2.getSize() << endl;
	cout << "array2的容量：" << array2.getCapacity() << endl;
}

// 测试自定义数据类型
class myType
{
public:
	myType() {}
	myType(string name, int age)
	{
		this->m_Name = name;
		this->m_Age = age;
	}

public:
	string m_Name;
	int m_Age;
};

void printmyTypeArr(MyArray<myType> &myTypeArr)
{
	for (int i = 0; i < myTypeArr.getSize(); i++)
	{
		cout << "姓名：" << myTypeArr[i].m_Name << " 年龄： " << myTypeArr[i].m_Age << endl;
	}
}

void test02()
{
	// 创建数组
	MyArray<myType> pArray(10);
	myType p1("孙悟空", 30);
	myType p2("韩信", 20);
	myType p3("妲己", 18);
	myType p4("王昭君", 15);
	myType p5("赵云", 24);

	// 插入数据
	pArray.Push_back(p1);
	pArray.Push_back(p2);
	pArray.Push_back(p3);
	pArray.Push_back(p4);
	pArray.Push_back(p5);
	printmyTypeArr(pArray);
	
	cout << "pArray的大小：" << pArray.getSize() << endl;
	cout << "pArray的容量：" << pArray.getCapacity() << endl;
}

#endif