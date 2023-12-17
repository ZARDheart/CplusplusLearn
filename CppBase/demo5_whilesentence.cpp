#include <iostream>
#include <ctime> //时间库，计时用
#include <cmath>
using namespace std;
/*
while循环语句,满足循环条件，执行循环语句
语法： while(循环条件){ 循环语句 }

do…while循环语句
语法： do{ 循环语句 } while(循环条件);
与while的区别在于do…while会先执行一次循环语句，再判断循环条件
*/

int main()
{
	// 计算1+2+...+n并统计计算时间

	// 1 while循环语句
	clock_t start = clock(); // 获得当前时间
	int i = 0, sum = 0, n = 100000;
	while (i <= n)
	{
		sum += i;
		i++;
	}
	cout << "1+2+...+" << n << "=" << sum << endl;
	double pt = ((double)(clock() - start)) / CLOCKS_PER_SEC; // 当前时间-开始标记时间，转化为秒
	cout << pt << "s" << endl;

	// 2 do…while循环语句
	i = 0, sum = 0;
	do
	{
		sum += i;
		i++;
	} while (i <= n);
	cout << "1+2+...+10=" << sum << endl;

	// 案例：列出100～999中的所有水仙花数，水仙花数的定义：各个位数上的三次方之和等于本身，例如1^3+5^3+3^3=153
	i = 100;
	while (i <= 999)
	{
		int num1 = i % 10;
		int num10 = (i / 10) % 10;
		int num100 = i / 100;
		if (pow(num1, 3) + pow(num10, 3) + pow(num100, 3) == i)
			cout << i << " ";
		i++;
	}

	return 0;
}
