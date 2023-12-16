#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
using namespace std;
/*
指针的基本概念：可以通过指针间接访问内存
    内存编号是从0开始记录的，一般用十六进制数字表示
    可以利用指针变量保存地址
    指针变量的定义和使用
    指针变量定义语法： 数据类型 * 变量名；
*/
// 值传递
void swap1(int a, int b)
{
    int temp = a;
    a = b;
    b = temp;
}
// 地址传递
void swap2(int *p1, int *p2)
{
    int temp = *p1;
    *p1 = *p2;
    *p2 = temp;
}
//冒泡排序函数
void bubbleSort(int * arr, int len)  //int * arr 也可以写为int arr[]
{
	for (int i = 0; i < len - 1; i++)
	{
		for (int j = 0; j < len - 1 - i; j++)
		{
			if (arr[j] > arr[j + 1])
			{
				int temp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = temp;
			}
		}
	}
}
//打印数组函数
void printArray(int arr[], int len)
{
	for (int i = 0; i < len; i++)
	{
		cout << arr[i] << " ";
	}
    cout << endl;
}

int main()
{
    // 1、指针的定义
    int a = 10, b = 20; // 定义整型变量a
    // 指针定义语法： 数据类型 * 变量名 ;
    int *p;
    // 指针变量赋值
    //  普通变量存放的是数据,指针变量存放的是地址
    p = &a;             // 指针指向变量a的地址
    cout << &a << endl; // 打印数据a的地址
    cout << p << endl;  // 打印指针变量p

    // 2、指针的使用
    //  指针变量可以通过" * "操作符，操作指针变量指向的内存空间，这个过程称为解引用
    cout << "*p = " << *p << endl;

    // 3 指针的大小
    // 所有指针类型在32位操作系统下是4个字节，64位是8个字节
    cout << sizeof(p) << endl;
    cout << sizeof(char *) << endl;
    cout << sizeof(float *) << endl;
    cout << sizeof(double *) << endl;

    // 4 空指针和野指针
    // 空指针：指针变量指向内存中编号为0的空间，用于初始化指针变量
    // 注意：空指针指向的内存是不可以访问的
    int *p1 = NULL;
    // 访问空指针报错
    // 内存编号0 ~255为系统占用内存，不允许用户访问
    // cout << *p1 << endl; // 因此会报段错误：Segmentation fault (core dumped)
    // 野指针：指针变量指向非法的内存空间
    // 指针变量p指向内存地址编号为0x1100的空间
    int *p2 = (int *)0x1100;
    // 访问野指针报错
    // cout << *p2 << endl; // 因此会报段错误：Segmentation fault (core dumped)

    // 5 const修饰指针有三种情况
    // const修饰指针 — 常量指针
    // const修饰常量 — 指针常量
    // const即修饰指针，又修饰常量
    // 5.1 const修饰的是指针，指针指向可以改，指针指向的值不可以更改
    const int *p11 = &a;
    p11 = &b; // 正确
    //*p11 = 100;  报错
    // 5.2 const修饰的是常量，指针指向不可以改，指针指向的值可以更改
    int *const p12 = &a;
    // p12 = &b; //错误
    *p12 = 100; // 正确
    // 5.3 const既修饰指针又修饰常量,均不可更改
    const int *const p13 = &a;
    // p13 = &b; //错误
    //*p13 = 100; //错误

    // 6 指针和数组: 利用指针访问数组中元素
    int arr[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    int *parr = arr; // 指向数组的指针
    cout << "下标第一个元素： " << arr[0] << endl;
    cout << "指针访问第一个元素： " << *parr << endl;
    for (int i = 0; i < 10; i++)
    {
        // 利用指针遍历数组
        cout << *parr << endl;
        // 数组名代表的是第一个元素的地址，指针变量加一后，指针增加的量是他指向数据类型的字节数
        parr++;
    }

    // 7 指针和函数：利用指针作函数参数，可以修改实参的值
    cout << "a = " << a << endl;
    cout << "b = " << b << endl;
    swap1(a, b); // 值传递不会改变实参
    cout << "a = " << a << endl;
    cout << "b = " << b << endl;
    swap2(&a, &b); // 地址传递会改变实参
    cout << "a = " << a << endl;
    cout << "b = " << b << endl;

    // 8 指针传递：当数组名传入到函数作为参数时，被退化为指向首元素的指针
    int arr1[10] = { 4,3,6,9,1,2,10,8,7,5 };
	int len = sizeof(arr1) / sizeof(int);
    printArray(arr1, len);
	bubbleSort(arr1, len);
	printArray(arr1, len);

    return 0;
}
