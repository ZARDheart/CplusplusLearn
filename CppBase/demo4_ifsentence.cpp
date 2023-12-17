#include <iostream>
#include <string>
using namespace std;
/*
选择结构：执行满足条件的语句
if语句的三种形式
单行格式if语句
多行格式if语句
多条件的if语句
三目运算符
switch多条件分支语句
*/

#define pi 3.1415926

int main()
{
	// 1 单行格式if语句：if(条件){ 条件满足执行的语句 }
	if (true)
	{
		cout << "单行格式if语句条件满足" << endl;
	}

	// 2 多行格式if语句：if(条件){ 条件满足执行的语句 }else{ 条件不满足执行的语句 };
	if (false)
	{
		cout << "多行格式if语句条件满足" << endl;
	}
	else
	{
		cout << "多行格式if语句条件不满足" << endl;
	}

	// 3 多条件的if语句：if(条件1){ 条件1满足执行的语句 }else if(条件2){条件2满足执行的语句}... else{ 都不满足执行的语句}
	cout << "请输入你的考试成绩：" << endl;
	int a = 0;
	cin >> a;
	if (a >= 85)
	{
		// 嵌套
		if (a == 100)
		{
			cout << "恭喜你考了满分！" << endl;
		}
		else
		{
			cout << "你的成绩为优秀。" << endl;
		}
	}
	else if (a >= 75)
	{
		cout << "你的成绩为良好。" << endl;
	}
	else if (a >= 60)
	{
		cout << "你的成绩为合格。" << endl;
	}
	else
	{
		cout << "你的成绩为不合格。" << endl;
	}

	// 4 三目运算符：逻辑表达式的值为真，运行前者表达式并返回，反之运行后者表达式并返回
	// 和if语句比较，三目运算符优点是短小整洁，缺点是如果用嵌套，结构不清晰。运算时用来代替2是很好的选择
	int b = 10, c = 0;
	c = a > b ? a + b : a - b;
	cout << "三目运算符：" << c << endl;
	// 三目运算符返回的是一个变量，可以接着赋值
	cout << "a：" << a << endl;
	cout << "b：" << b << endl;
	a < b ? a : b = 100;
	cout << "a：" << a << endl;
	cout << "b：" << b << endl;

	// 5 switch多条件分支语句
	// switch语句中表达式类型只能是整型或者字符型
	// case里如果没有break，那么程序会一直向下执行
	// 与if语句比，对于多条件判断时，switch的结构清晰，执行效率高，缺点是switch不可以判断区间
	int d = 0;
	cout << "请输入星期数：" << endl;
	cin >> d;
	switch (d)
	{
	case 1:
		cout << "今天是星期一" << endl;
		break;
	case 2:
		cout << "今天是星期二" << endl;
		break;
	case 3:
		cout << "今天是星期三" << endl;
		break;
	case 4:
		cout << "今天是星期四" << endl;
		break;
	case 5:
		cout << "今天是星期五" << endl;
		break;
	case 6:
		cout << "今天是星期六" << endl;
		break;
	case 7:
		cout << "今天是星期天" << endl;
		break;
	default:
		cout << "输入错误！" << endl;
		break;
	}

	return 0;
}
