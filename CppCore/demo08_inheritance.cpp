#include <iostream>
using namespace std;
/*
继承是C++面向对象三大特性之一，
    定义一些类时，下级别的不同成员除了拥有上一级的共性
    这个时候我们就可以考虑利用继承的技术，减少重复代码

继承的基本语法
    class A : public B;
    A 类称为子类 或 派生类
    B 类称为父类 或 基类

派生类中的成员，包含两大部分：
    一类是从基类继承过来的，一类是自己增加的成员。
    从基类继承过过来的表现其共性，而新增的成员体现了其个性。

继承方式一共有三种：
    公共继承
    保护继承
    私有继承
*/

//  1 父/基类：
class Base
{
public:
    int m_public_A;
    static int m_A;
    static void func1()
    {
        cout << "Base - func1()调用" << endl;
    }
    void func()
    {
        cout << "Base - func()调用" << endl;
    }
    // 如果派生类的构造函数没有显式调用基类的构造函数，那么基类必须有默认构造函数。
    // 这是因为如果派生类的构造函数没有显式调用基类构造函数，编译器会尝试调用基类的默认构造函数来初始化基类部分。
    Base()
    {
        m_public_A = 100;
        cout << "Base构造函数!" << endl;
    }
    ~Base()
    {
        cout << "Base析构函数!" << endl;
    }

protected:
    int m_protected_A;

private:
    int m_private_A; // 私有成员只是被隐藏了，但是还是会继承下去
};
int Base::m_A = 100;

// 2 子类/派生类
// 2.1 公共继承
class son1 : public Base
{
public:
    // 5 同名成员访问：当子类与父类出现同名的成员，访问子类同名成员 直接访问即可，访问父类同名成员 需要加作用域
    int m_public_A;
    static int m_A;
    static void func1()
    {
        cout << "son1 - func1()调用" << endl;
    }
    void func()
    {
        cout << "son1 - func()调用" << endl;
        m_public_A = 10;    // （公共继承的情况下）父类中的公共权限，子类中依然为公共权限
        m_protected_A = 10; // （公共继承的情况下）父类中的保护权限，子类中依然为保护权限
        // m_private_A = 10;// err （公共继承的情况下）父类中的私有权，子类不可访问
    }
};
int son1::m_A = 300;

// 2.2 保护继承
class son2 : protected Base
{
public:
    void func()
    {
        m_public_A = 10;    // （保护继承的情况下）父类中的 公共权限，子类中变为 保护权限
        m_protected_A = 10; // （保护继承的情况下）父类中的 保护权限，子类中依然为 保护权限
        // m_private_A = 10;// err （保护继承的情况下）父类中的私有权，子类不可访问
    }
};

// 2.3 私有继承
class son3 : private Base
{
public:
    // 7 继承中构造和析构顺序：先调用父类构造函数，再调用子类构造函数，析构顺序与构造相反
    son3()
    {
        cout << "Son3构造函数!" << endl;
    }
    ~son3()
    {
        cout << "Son3析构函数!" << endl;
    }
    void func()
    {
        m_public_A = 10;    // （私有继承的情况下）父类中的 公共权限，子类中变为 私有权限
        m_protected_A = 10; // （私有继承的情况下）父类中的 保护权限，子类中变为 私有权限
        // m_private_A = 10;// err （私有继承的情况下）父类中的私有权，子类不可访问
    }
};

class Base1
{
public:
    int m_public_A;
    Base1()
    {
        m_public_A = 150;
        cout << "Base构造函数!" << endl;
    }

protected:
    int m_protected_A;
};

// 6 C++允许一个类继承多个类 语法： class 子类 ：继承方式 父类1 ， 继承方式 父类2...
// 多继承可能会引发父类中有同名成员出现，需要加作用域区分,C++实际开发中不建议用多继承
class Son : public Base1, public Base
{
public:
    Son()
    {
        m_C = 300;
        m_D = 400;
    }

public:
    int m_C;
    int m_D;
};

// 7 菱形继承：两个派生类继承同一个基类，又有某个类同时继承者两个派生类
class Animal
{
public:
    int m_Age;
};
// 继承前加virtual关键字后，变为虚继承
// 此时公共的父类Animal称为虚基类
class Sheep : virtual public Animal
{
};
class Tuo : virtual public Animal
{
};
class SheepTuo : public Sheep, public Tuo
{
};

int main()
{
    // 1 公有继承
    son1 s1;
    s1.m_public_A = 10; // m_public_A 在父类子类中都是公共权限，类外可以访问
    // s1.m_protected_A = 10;// err  m_protected_A 在父类和子类中都是保护权限，类外不可以访问

    // 2 保护继承
    son2 s2;
    // s2.m_public_A = 10;//err  m_public_A 在子类中都是保护权限，类外不可以访问
    // s2.m_protected_A = 10;// err  m_protected_A 在父类和子类中都是保护权限，类外不可以访问

    // 3 私有继承
    son3 s3;
    // s3.m_public_A = 10;//err  m_public_A 在子类中都是私有权限，类外不可以访问
    // s3.m_protected_A = 10;// err  m_protected_A 在父类中是保护权限，在子类中是私有权限，类外不可以访问

    // 4 继承中的对象模型：在继承之中，子类会将父类中所有的非静态变量继承
    // 打印输出一下子类类型的大小：
    cout << "size of son = " << sizeof(son1) << endl;
    // 打印结果为 16 = 4 * 4，即 4 个 int 型变量的大小
    // 注：子类会继承父类的私有权限下的变量，但是编译器将它隐藏了，子类无法访问

    // 5 同名成员访问：当子类与父类出现同名的成员，访问子类同名成员 直接访问即可，访问父类同名成员 需要加作用域
    cout << "Son1下的m_public_A = " << s1.m_public_A << endl;
    cout << "Base下的m_public_A = " << s1.Base::m_public_A << endl;
    // 当子类与父类拥有同名的成员函数，子类会隐藏父类中同名成员函数，加作用域可以访问到父类中同名函数
    s1.func();
    s1.Base::func();
    // 同名静态成员访问，处理方式和非静态处理方式一样，只不过有两种访问的方式（通过对象 和 通过类名）
    // 通过对象访问
    cout << "Son1下m_A = " << s1.m_A << endl;
    cout << "Base下m_A = " << s1.Base::m_A << endl;
    // 通过类名访问
    cout << "Son1下m_A = " << son1::m_A << endl;
    cout << "Base下m_A = " << son1::Base::m_A << endl;
    // 同名静态成员函数也是一样的
    son1::func1();
    son1::Base::func1();

    // 6 多继承容易产生成员同名的情况，通过使用类名作用域可以区分调用哪一个基类的成员
    Son s;
    cout << "sizeof Son = " << sizeof(s) << endl; // 基类1 3int 基类2 2int 子类 2int 4*7
    cout << s.Base1::m_public_A << endl;
    cout << s.Base::m_public_A << endl;

    // 7 菱形继承带来的主要问题是子类继承两份相同的数据，导致资源浪费以及毫无意义
    SheepTuo st;
    st.Sheep::m_Age = 100;
    st.Tuo::m_Age = 200;
    cout << "st.Sheep::m_Age = " << st.Sheep::m_Age << endl;
    cout << "st.Tuo::m_Age = " << st.Tuo::m_Age << endl;
    cout << "st.m_Age = " << st.m_Age << endl;

    // 8 继承中构造和析构顺序：先调用父类构造函数，再调用子类构造函数，析构顺序与构造相反

    return 0;
}
