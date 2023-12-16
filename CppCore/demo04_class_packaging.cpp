#include <iostream>
using namespace std;
/*
类和对象
    C++面向对象的三大特性为：封装、继承、多态
    C++认为万事万物都皆为对象，对象上有其属性和行为

封装是C++面向对象三大特性之一，封装的意义：
    将属性和行为作为一个整体，表现生活中的事物
        语法： class 类名{ 访问权限： 属性 / 行为 };
    将属性和行为加以权限控制，访问权限有三种：
        public 公共权限
        protected 保护权限
        private 私有权限
*/

// 圆周率
const double PI = 3.14;

// 1 封装的意义1
// class代表设计一个类：封装一个圆类，求圆的周长
class Circle
{
public: // 访问权限
    // 属性：半径
    int m_r;

    // 行为：获取到圆的周长
    // 类内声明函数
    double calculateZC();
};

// 类外定义
double Circle::calculateZC()
{
    return 2 * PI * m_r;
}

// 2 三种权限
// 公共权限  public     类内可以访问  类外可以访问
// 保护权限  protected  类内可以访问  类外不可以访问
// 私有权限  private    类内可以访问  类外不可以访问
class Person
{
    // 默认权限为私有
    string motto = "This is a private string.";
    // 汽车  保护权限
protected:
    string m_Car;

    // 银行卡密码  私有权限
private:
    int m_Password;

    // 姓名  行为 公共权限
public:
    string m_Name;
    // 类内可以访问
    void func()
    {
        m_Car = "拖拉机";
        m_Password = 123456;
        cout << "m_Name " << m_Name << endl;
        cout << "m_Car " << m_Car << endl;
        cout << "m_Password " << m_Password << endl;
        cout << "motto " << motto << endl;
    }
    // 将所有成员属性设置为私有，可以自己控制读写权限
    void setmotto(string str)
    {
        motto = str;
    }
    string getmotto()
    {
        return motto;
    }
};

int main()
{
    // 1 封装
    // 通过圆类，创建圆的对象，c1就是一个具体的圆
    Circle c1;
    c1.m_r = 10; // 给圆对象的半径 进行赋值操作
    // 2 * pi * 10 = = 62.8
    cout << "圆的周长为： " << c1.calculateZC() << endl;

    // 2 三种权限
    Person p;
    // 公共权限类外可以访问
    p.m_Name = "李四";
    // p.m_Car = "奔驰";  //保护权限类外访问不到
    // p.motto = "This is a private string.";
    // p.m_Password = 123; //私有权限类外访问不到
    p.func();
    p.setmotto("This is a private string.");
    cout << "p.motto: " << p.getmotto() << endl;

    // 3 对象数组(使用时必须有默认构造函数)
    Person stu[5];
    stu[1] = p;
    cout << "stu[1].motto: " << stu[1].getmotto() << endl;

    return 0;
}