#include <iostream>
#include <cstring>
#include <vector>
#include <cstdlib> //控制程序
using namespace std;
/*
函数:将一段经常使用的代码封装起来，减少重复代码
一个较大的程序，一般分为若干个程序块，每个模块实现特定的功能。

函数的定义一般主要有5个步骤：
    返回值类型 ：一个函数可以返回一个值。在函数定义中
    函数名：给函数起个名称
    参数列表：使用该函数时，传入的数据
    函数体语句：花括号内的代码，函数内需要执行的语句
    return表达式： 和返回值类型挂钩，函数执行完后，返回相应的数据
        返回值类型 函数名 （参数列表）
        {
            函数体语句

            return表达式
        }

常见的函数样式有4种
    无参无返
    有参无返
    无参有返
    有参有返
*/

// 函数声明 告诉编译器函数名称及如何调用函数。函数的实际主体可以单独定义
// 函数的声明可以多次，但是函数的定义只能有一次
bool simple(string name);

// 函数定义
int add(int num1, int num2) // 定义中的num1,num2称为形式参数，简称形参
{
    int sum = num1 + num2;
    return sum;
}

// 函数定义里小括号内称为形参，函数调用时传入的参数称为实参
// 所谓值传递，就是函数调用时实参将数值传入给形参，值传递时，如果形参在函数内发生改变，并不会影响实参
void swap(int num1, int num2)
{
    cout << "交换前：" << endl;
    cout << "num1 = " << num1 << endl;
    cout << "num2 = " << num2 << endl;

    int temp = num1;
    num1 = num2;
    num2 = temp;

    cout << "交换后：" << endl;
    cout << "num1 = " << num1 << endl;
    cout << "num2 = " << num2 << endl;

    // return ; 当函数声明时候，不需要返回值，可以不写return
}

int main()
{
    // 1 函数调用
    string name = "Izumi Sakai";
    if (simple(name))
    {
        cout << "First function run." << endl;
    }
    int a = 10, b = 20;
    int sum = add(a, b); // 调用时的a，b称为实际参数，简称实参
    cout << "sum = " << sum << endl;
    
    // 2 值传递
    // 实参无法通过函数内改变
    cout << "交换前：" << endl;
    cout << "mian中的 a = " << a << endl;
    cout << "mian中的 b = " << b << endl;
    swap(a, b);
    cout << "交换后：" << endl;
    cout << "mian中的 a = " << a << endl;
    cout << "mian中的 b = " << b << endl;

    return 0;
}

// 函数定义(也可放在主函数之前不需声明)
bool simple(string name)
{
    cout << "Hello " << name << endl;
    return true;
}