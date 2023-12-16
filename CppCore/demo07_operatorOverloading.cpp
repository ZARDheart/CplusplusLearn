#include <iostream>
using namespace std;
/*
运算符重载：对已有的运算符重新进行定义，赋予其另一种功能，以适应不同的数据类型
*/

class Time
{
public:
    int hours;
    int minutes;

    Time(int h = 0, int min = 0)
    {
        hours = h;
        minutes = min;
    }
    Time(double min) // 强制类型转换构造函数
    {
        hours = (int)min / 60;
        minutes = (int)min % 60;
    }

    // 1 成员函数实现+/*运算符重载
    Time operator+(const Time &t) const;
    Time operator*(int n);
    friend Time operator*(int n, const Time &t);

    // 2 重载<<输出运算符
    // 成员函数 实现不了  p << cout 不是我们想要的效果
    // void operator<<(Time& p){
    //}
    // 使用友元声明，是为了防止使用到私有成员
    friend ostream &operator<<(ostream &os, const Time &t);
    // 与下面函数的功能相同
    void ShowTime()
    {
        cout << "Time:" << hours << ":" << minutes << endl;
    }

    // 3 递增运算符重载，实现时间增加
    // 前置++
    Time &operator++()
    {
        // 先++
        minutes++;
        hours = hours + minutes / 60;
        minutes = minutes % 60;
        // 再返回
        return *this;
    }
    // 后置++
    Time operator++(int)
    {
        // 先返回
        Time temp = *this;
        // 记录当前本身的值，然后让本身的值加1，但是返回的是以前的值，达到先返回后++；
        minutes++;
        hours = hours + minutes / 60;
        minutes = minutes % 60;
        return temp;
    }

    // 4 重载关系运算符，可以让两个自定义类型对象进行对比操作
    bool operator==(Time &p);
    bool operator!=(Time &p);
    bool operator>(Time &p);
    bool operator<(Time &p);

    // 5 函数调用运算符 () 也可以重载
    //     由于重载后使用的方式非常像函数的调用，因此称为仿函数
    //     仿函数没有固定写法，非常灵活
    Time operator()(const Time &t1, const Time &t2)
    {
        Time t_add;
        t_add.minutes = (t1.minutes + t2.minutes) % 60;
        t_add.hours = t1.hours + t2.hours + (t1.minutes + t2.minutes) / 60;

        return t_add;
    }
};

// 1 成员函数 +/* 运算符重载
Time Time::operator+(const Time &t) const
{
    int h = t.hours;
    int min = t.minutes;
    Time t_add;
    t_add.minutes = (minutes + min) % 60;
    t_add.hours = hours + h + (minutes + min) / 60;

    return t_add;
}
// 1.1 全局函数实现 + 运算符重载
// Time operator+(const Time &t)
// {
//     int h = t.hours;
//     int min = t.minutes;
//     Time t_add;
//     t_add.minutes = (minutes + min) % 60;
//     t_add.hours = hours + h + (minutes + min) / 60;

//     return t_add;
// }

// 1.2 运算符重载 可以再发生函数重载
Time Time::operator*(int n)
{
    Time t_mul;
    t_mul.minutes = (minutes * n) % 60;
    t_mul.hours = hours * n + (minutes * n) / 60;

    return t_mul;
}

Time operator*(int n, const Time &t)
{
    Time t_mul;
    t_mul.minutes = (t.minutes * n) % 60;
    t_mul.hours = t.hours * n + (t.minutes * n) / 60;
    cout << "operator*重载！" << endl;

    return t_mul;
}

// 2 全局函数实现左移重载  ostream对象只能有一个
ostream &operator<<(ostream &os, const Time &t)
{
    os << "Time:" << t.hours << ":" << t.minutes << endl;

    return os; // 为了使引用时可以连续使用<<运算符
}

// 4 重载关系运算符，可以让两个自定义类型对象进行对比操作
bool Time::operator==(Time &p)
{
    if (this->hours + 60 * this->minutes == p.hours + 60 * p.minutes)
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool Time::operator!=(Time &p)
{
    if (this->hours + 60 * this->minutes != p.hours + 60 * p.minutes)
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool Time::operator>(Time &p)
{
    if (this->hours + 60 * this->minutes > p.hours + 60 * p.minutes)
    {
        return true;
    }
    else
    {
        return false;
    }
}
bool Time::operator<(Time &p)
{
    if (this->hours + 60 * this->minutes < p.hours + 60 * p.minutes)
    {
        return true;
    }
    else
    {
        return false;
    }
}

int main()
{
    Time t1(8, 32), t2(10, 41);

    // 1 +/* 运算符重载
    Time t = t1 + t2;
    // t=t1.operator+(t2);    //两种使用方法相同
    t.ShowTime();
    t = t * 2;
    t.ShowTime();
    // 重载
    t = 2 * t;

    // 2 全局函数实现左移重载
    // 下面三种使用等价
    // t=operator*(2,t);
    // t.ShowTime();
    cout << t << endl;
    t = 93.33; // 强制类型转换,可直接赋值
    t.ShowTime();

    // 3 递增运算符重载，实现时间增加
    Time t3(8, 59);
    t = t3++;
    t.ShowTime();
    t3.ShowTime();
    t = ++t3;
    t.ShowTime();
    t3.ShowTime();

    // 4 重载关系运算符，可以让两个自定义类型对象进行对比操作
    cout << (t == t3) << (t != t3) << (t1 > t2) << (t1 < t2) << endl;

    // 5 函数调用运算符
    Time t4;
    cout << t4(t1, t2);
    //匿名对象调用  
	cout << "Time()(100,100) = " << Time()(t1, t2) << endl;

    return 0;
}