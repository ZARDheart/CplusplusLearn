#include <iostream>
using namespace std;
/*
多态是C++面向对象三大特性之一，多态分为两类
    静态多态: 函数重载 和 运算符重载属于静态多态，复用函数名
    动态多态: 派生类和虚函数实现运行时多态
静态多态和动态多态区别：
    静态多态的函数地址早绑定 - 编译阶段确定函数地址
    动态多态的函数地址晚绑定 - 运行阶段确定函数地址
*/

// 多态实现抽象计算器类
// 多态优点：代码组织结构清晰，可读性强，利于前期和后期的扩展以及维护
class AbstractCalculator
{
public:
    // 函数前面加上virtual关键字，变成虚函数，那么编译器在编译的时候就不能确定函数调用 - 地址晚绑定
    virtual int getResult()
    {
        return 0;
    }
    // 正常函数中编译阶段确定函数地址 - 地址早绑定
    void run()
    {
        cout << "基类在跑" << endl;
    }

    int m_Num1;
    int m_Num2;
};
// 加法计算器
class AddCalculator : public AbstractCalculator
{
public:
    // 重写父类虚函数
    int getResult()
    {
        return m_Num1 + m_Num2;
    }
    void run()
    {
        cout << "加法在跑" << endl;
    }
};
// 减法计算器
class SubCalculator : public AbstractCalculator
{
public:
    int getResult()
    {
        return m_Num1 - m_Num2;
    }
    void run()
    {
        cout << "减法在跑" << endl;
    }
};
// 乘法计算器
class MulCalculator : public AbstractCalculator
{
public:
    int getResult()
    {
        return m_Num1 * m_Num2;
    }
    void run()
    {
        cout << "乘法在跑" << endl;
    }
};
// 除法计算器
class DivCalculator : public AbstractCalculator
{
public:
    int getResult()
    {
        return m_Num1 / m_Num2;
    }
    void run()
    {
        cout << "除法在跑" << endl;
    }
};

// 我们希望传入什么对象，那么就调用什么对象的函数 - 调用的时候才绑定函数地址
// 如果函数地址在编译阶段就能确定，那么静态联编
// 如果函数地址在运行阶段才能确定，就是动态联编
// 3.1 父类引用指向子类对象
void DoCalculate(AbstractCalculator &Calculator)
{
    // 地址已经确定，静态
    Calculator.run();
    // 现在才绑定，动态
    cout << Calculator.getResult() << endl;
}

// 纯虚函数和抽象类
// 在多态中，通常父类中虚函数的实现是毫无意义的，主要都是调用子类重写的内容
// 因此可以将虚函数改为纯虚函数。语法：virtual 返回值类型 函数名 （参数列表）= 0 ;
// 当类中有了纯虚函数，这个类也称为抽象类
// 抽象类特点：
//     无法实例化对象，只能作为基类
//     子类必须重写抽象类中的纯虚函数，否则也属于抽象类
class Base
{
public:
    // 类中只要有一个纯虚函数就称为抽象类
    virtual void func() = 0;
    Base()
    {
        cout << "Base 构造函数调用！" << endl;
    }
    // 多态使用时，如果子类中有属性开辟到堆区，那么父类指针在释放时无法调用到子类的析构代码,造成内存泄露
    // 解决方式：将父类中的析构函数改为虚析构或者纯虚析构
    // 和包含普通纯虚函数的类一样，包含了纯虚析构函数的类也是一个抽象类。不能够被实例化
    virtual ~Base()
    {
        cout << "Base虚析构函数调用！" << endl;
    }
    //  virtual ~Base() = 0;
    // 如果子类中没有堆区数据，可以不写为虚析构或纯虚析构
};

class Son : public Base
{
public:
    // 子类必须重写父类中的（所有）纯虚函数，否则也属于抽象类
    virtual void func()
    {
        cout << "func调用" << endl;
    };

    Son(string name)
    {
        cout << "Son构造函数调用！" << endl;
        m_Name = new string(name);
    }
    ~Son()
    {
        cout << "Son析构函数调用!" << endl;
        if (this->m_Name != NULL)
        {
            delete m_Name;
            m_Name = NULL;
        }
    }

public:
    string *m_Name;
};

int main()
{
    // 多态满足条件：
    // 1、有继承关系
    // 2、子类重写父类中的虚函数
    // 3、多态使用：父类指针或引用指向子类对象
    // 创建加法计算器
    AddCalculator Calculator1;
    Calculator1.m_Num1 = 10;
    Calculator1.m_Num2 = 10;
    DoCalculate(Calculator1);
    // 创建减法计算器
    SubCalculator Calculator2;
    Calculator2.m_Num1 = 10;
    Calculator2.m_Num2 = 10;
    DoCalculate(Calculator2);
    // 创建除法计算器
    DivCalculator Calculator3;
    Calculator3.m_Num1 = 10;
    Calculator3.m_Num2 = 10;
    // 3.1 父类引用指向子类对象
    AbstractCalculator &Calculator4 = Calculator3;
    cout << Calculator4.m_Num1 << " / " << Calculator4.m_Num2 << " = " << Calculator4.getResult() << endl;
    // 创建加法计算器
    // 3.2 父类指针指向子类对象
    AbstractCalculator *abc = new AddCalculator;
    abc->m_Num1 = 10;
    abc->m_Num2 = 10;
    cout << abc->m_Num1 << " + " << abc->m_Num2 << " = " << abc->getResult() << endl;
    delete abc; // 用完了记得销毁
    // 创建减法计算器
    abc = new SubCalculator;
    abc->m_Num1 = 10;
    abc->m_Num2 = 10;
    cout << abc->m_Num1 << " - " << abc->m_Num2 << " = " << abc->getResult() << endl;
    delete abc;
    // 创建乘法计算器
    abc = new MulCalculator;
    abc->m_Num1 = 10;
    abc->m_Num2 = 10;
    cout << abc->m_Num1 << " * " << abc->m_Num2 << " = " << abc->getResult() << endl;
    delete abc;

    // 抽象类
    Base *base = NULL;
    // base = new Base; // 错误，抽象类无法实例化对象
    base = new Son("son");
    base->func();
    delete base; // 通过父类指针去释放，给基类增加一个虚析构函数解决通过父类指针释放子类对象

    return 0;
}