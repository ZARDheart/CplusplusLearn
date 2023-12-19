#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

int main()
{
    double x = 9;
    // 1.开平方
    // double sqrt(double x);
    cout << "sqrt(9): " << sqrt(x) << endl;

    // 2.求常数e的x次方
    // double exp(double x);
    cout << "exp(9): " << exp(x) << endl;

    // 3.求常数x的y次方
    // double pow(double x, double y);
    cout << "pow(9,2): " << pow(x, 2) << endl;

    // 4.求对数lnx、lgx
    // double log(double x);//求对数lnx
    // double log10(double x);//求对数lgx
    cout << "log(9): " << log(x) << endl;
    cout << "log10(100): " << log10(100) << endl;

    // 5.求x绝对值
    // int abs(x);//整数型
    // double fabs(double x);//浮点型
    cout << "abs(-9): " << abs(-9) << endl;
    cout << "fabs(-9.9): " << fabs(-9.9) << endl;

    // 6.取整函数
    // double ceil(double x);//向上取整 返回的是大于或等于x的最小整数
    // double floor(double x);//向下取整 返回的是小于或等于x的最大整数
    // double fix(double x);//朝零方向取整
    // double round(double x);//四舍五入到最近的整数
    cout << "ceil(9.6): " << ceil(9.6) << endl;
    cout << "floor(9.6): " << floor(9.6) << endl;
    cout << "round(9.6): " << round(9.6) << endl;

    // 7.产生随机数
    srand((unsigned int)time(NULL)); // 使用时记得加随机数种子，否则每次洗牌都一样
    // int rand(void); //产生一个0~RAND_MAX之间的随机数。int:32767; unsigned int 双字节是 65535，四字节是 4294967295。
    // f = rand() / double(RAND_MAX); // 0～1 之间的浮点数
    // r = rand() % 50;               // 产生一个在[0,50)区间内的随机数
    // r = a + rand() % (b - a + 1);  // 产生[a,b]区间的随机数
    for (int i = 0; i < 10; i++)
        cout << rand() % 50 << " ";
    cout << endl;

    // 8.取整与取余
    // double fmod (double x, double y);//返回两参数相除的余数
    // double modf (double value, double* iptr);//将参数的整数部分通过指针回传
    cout << "fmod(9.9,2): " << fmod(9.9, 2) << endl;
    cout << "modf(9.9,2): " << modf(9.9, &x) << "---" << x << endl;

    // 9.三角函数  圆周率可用"M_PI"表示(3.14159)，需引入文件头<cmath>
    // double sin(double x);//正弦
    // double cos(double x);//余弦
    // double tan(double x);//正切
    cout << "sin(5*M_PI): " << sin(5 * M_PI) << endl;
    cout << "cos(5*M_PI): " << cos(5 * M_PI) << endl;
    cout << "tan(5*M_PI): " << tan(5 * M_PI) << endl;

    // 10.反三角函数
    // double asin(double x);//反正弦 [−π/2, π/2]
    // double acos(double x);//反余弦 [0, π]
    // double atan(double x);//反正切（主值）   [−π/2, π/2]
    // double atan2(double x);//反正切（整圆值） [−π, π]
    cout << "asin(1): " << asin(1) << endl;
    cout << "acos(1): " << acos(1) << endl;
    cout << "atan(1): " << atan(1) << endl;
    cout << "atan2(1,2.0): " << atan2(1, 2.0) << endl;

    // 11.该函数返回两个参数的平方总和的平方根 sqrt(x^2+y^2)
    // double hypot(double x, double y);
    cout << "hypot(3,4): " << hypot(3, 4) << endl;

    // 12.最值
    // double min(double x, double y);
    // double max(double x, double y);
    cout << "min(78,56.484): " << min(7.2, 56.0) << endl;
    cout << "max(78,56.484): " << max(7.2, 56.0) << endl;

    // 13.双曲三角函数
    // double sinh (double);
    // double cosh (double);
    // double tanh (double);
    cout << "sinh(1): " << sinh(1) << endl;
    cout << "cosh(1): " << cosh(1) << endl;
    cout << "tanh(1): " << tanh(1) << endl;

    return 0;
}