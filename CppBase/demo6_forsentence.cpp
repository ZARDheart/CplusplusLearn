#include <iostream>
#include <ctime> //时间库，计时用
using namespace std;
/*
for循环语句，满足循环条件，执行循环语句
语法： for(起始表达式;条件表达式;末尾循环体) { 循环语句; }

break语句用于跳出选择结构或者循环结构
continue语句在循环语句中，跳过本次循环中余下尚未执行的语句，继续执行下一次循环
*/

int main()
{
    // 计算1+2+...+n并统计计算时间

    // 1 while循环语句
    int sum = 0;
    clock_t start = clock(); // 获得当前时间
    for (int i = 0; i <= 100000000; i++)
    {
        sum += i;
    }
    cout << "1+2+...+100000000=" << sum << endl;
    double pt = ((double)(clock() - start)) / CLOCKS_PER_SEC;
    cout << pt << "s" << endl;

    // 2 break语句和continue语句
    int sum2 = 0;
    for (int i = 0; i <= 100; i++)
    {
        if (i > 100)
            break;
        else if (i > 10 && i < 20)
            continue;
        sum2 += i;
    }
    // 11+12+...19=135   5050-135=4915
    cout << "1+2+...10+20+....+100=" << sum2 << endl;

    // 3 基于范围的for循环
    double p[] = {5.865, 5.5984, 9.5984, 85.845};
    for (double x : p)
    {
        cout << x << endl;
    };

    // 4 循环嵌套，打印乘法表
    for (int i = 1; i < 10; i++)
    {
        for (int j = 1; j <= i; j++)
        {
            cout << j << "x" << i << "=" << i * j << "\t";
        }
        cout << endl;
    }

    return 0;
}
