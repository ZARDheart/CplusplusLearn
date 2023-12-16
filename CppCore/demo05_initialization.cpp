#include <iostream>
using namespace std;
/*
对象的初始化和清理也是两个非常重要的安全问题
c++利用了构造函数和析构函数解决上述问题，这两个函数将会被编译器自动调用，完成对象初始化和清理工作。
    编译器提供的构造函数和析构函数是空实现。

构造函数：主要作用在于创建对象时为对象的成员属性赋值，构造函数由编译器自动调用，无须手动调用。
    语法：类名(){}
    构造函数，没有返回值也不写void
    函数名称与类名相同
    构造函数可以有参数，因此可以发生重载
    程序在调用对象时候会自动调用构造，无须手动调用,而且只会调用一次
构造函数调用规则如下：
    如果用户定义有参构造函数，c++不再提供默认无参构造，但是会提供默认拷贝构造
    如果用户定义拷贝构造函数，c++不会再提供其他构造函数

析构函数：主要作用在于对象销毁前系统自动调用，执行一些清理工作。
    语法： ~类名(){}
    析构函数，没有返回值也不写void
    函数名称与类名相同,在名称前加上符号 ~
    析构函数不可以有参数，因此不可以发生重载
    程序在对象销毁前会自动调用析构，无须手动调用,而且只会调用一次
*/

// 构造函数分类
//  按照参数分类分为 有参和无参构造 无参又称为默认构造函数
//  按照类型分类分为 普通构造和拷贝构造
class Person
{
public:
    // 1 无参（默认）构造函数
    Person()
    {
        cout << "无参构造函数!" << endl;
        // age = 0;
        // 如果属性有在堆区开辟的,并在析构函数手动释放，必须在所有构造函数初始化，
        // 否则将会报free(): invalid pointer Aborted (core dumped)，即析构函数释放无效指针
        m_height = new int;
        mut_B = 0;
    }
    // 2 有参构造函数
    Person(int a, int height)
    {
        age = a;
        m_height = new int(height);
        num = 0;
        cout << "有参构造函数!" << endl;
        mut_B = 0;
    }
    // 有参构造函数重载
    Person(int a)
    {
        age = a;
        num = 0;
        m_height = new int;
        cout << "有参构造函数重载!" << endl;
        mut_B = 0;
    }
    // 初始化列表方式初始化
    Person(int a, int height, int c) : age(a), num(c)
    {
        m_height = new int(height);
        cout << "有参构造初始化列表方式初始化!" << endl;
        mut_B = 0;
    }
    // 3 拷贝构造函数
    Person(const Person &p)
    {
        age = p.age;
        num = p.num;
        cout << "拷贝构造函数!" << endl;
        // 3.1 如果不利用深拷贝在堆区创建新内存，会导致浅拷贝带来的重复释放堆区问题
        m_height = new int(*p.m_height);
        mut_B = 0;
    }
    // 4 析构函数 析构函数在释放内存之前调用
    ~Person()
    {
        cout << "析构函数!" << endl;
        // 4.1 在析构函数中手动释放堆区
        if (m_height != NULL)
        {
            delete m_height;
        }
    }

public:
    int age;
    // 5 如果属性有在堆区开辟的，一定要自己提供拷贝构造函数，防止浅拷贝带来的问题
    //     浅拷贝：简单的赋值拷贝操作
    //     深拷贝：在堆区重新申请空间，进行拷贝操作
    int *m_height;
    int num;

    // 6 静态成员变量
    // 静态成员变量特点：
    // 1 在编译阶段分配内存
    // 2 类内声明，类外初始化
    // 3 所有对象共享同一份数据
    static int m_A;
    // 静态成员函数特点：
    // 1 程序共享一个函数
    // 2 静态成员函数只能访问静态成员变量
    static void staticfunc()
    {
        cout << "staticfunc调用" << endl;
        m_A = 100;
        // num = 100; //错误，不可以访问非静态成员变量
    }
    // 注意：在C++中，类内的成员变量和成员函数分开存储，且只有非静态成员变量才属于类的对象上
    // 非静态成员变量占对象空间，静态成员变量不占对象空间
    // 成员函数也不占对象空间，所有函数共享一个函数实例

    // 7 每一个非静态成员函数只会诞生一份函数实例，也就是说多个同类型的对象会共用一块代码
    // c++通过提供特殊的对象指针，this指针区分那个对象调用自己。this指针指向被调用的成员函数所属的对象
    // this指针是隐含每一个非静态成员函数内的一种指针,不需要定义，直接使用即可,this指针的用途：
    //     当形参和成员变量同名时，可用this指针来区分
    void printAge(int age)
    {
        this->age = age;
        cout << this->age << endl;
    }
    //     在类的非静态成员函数中返回对象本身，可使用return *this
    Person returnself()
    {
        return *this;
    }
    Person &returnself2()
    {
        return *this;
    }

    // 8 空指针访问成员函数
    void ShowPerson()
    {
        if (this == NULL)
        {
            return;
        }
        cout << age << endl;
    }

    // 9 const修饰成员函数 this指针的本质是一个指针常量Person* const this，指针的指向不可修改，
    // 但是指向的值可以修改，如果想让指针指向的值也不可以修改，需要声明常函数 相当于让const Person* const this
    //     常函数：
    //         成员函数后加const后我们称为这个函数为常函数
    //         常函数内不可以修改成员属性
    //         成员属性声明时加关键字mutable后，在常函数中依然可以修改
    void ShowPerson() const
    {
        // this = NULL; //不能修改指针的指向 Person* const this;
        // this->age = 30; //但是this指针指向的对象的数据是可以修改的
        // const修饰成员函数，表示指针指向的内存空间的数据不能修改，除了mutable修饰的变量
        this->mut_B = 60;
    }
    mutable int mut_B; // 可修改 可变的

private:
    // 私有静态成员变量与函数也和普通的一样私有权限
    static int m_B;
    static void func2()
    {
        cout << "func2调用" << endl;
    }
};
// 类外初始化
int Person::m_A = 10;
int Person::m_B = 15;

// 值传递的方式给函数参数传值，相当于Person p1 = 参数;
void doWork(Person p1)
{
    cout << p1.age << endl;
}

// 以值方式返回局部对象
Person doWork2()
{
    Person p1(20, 120);
    cout << (int *)&p1 << endl;
    return p1;
}

// 9 C++类中的成员可以是另一个类的对象，我们称该成员为 对象成员
class group
{
public:
    // 初始化列表可以告诉编译器调用哪一个构造函数
    group(string name, Person p11, Person p12, Person p13) : groupname(name), p1(p11), p2(p12), p3(p13)
    {
        cout << "group构造" << endl;
    }

    ~group()
    {
        cout << "group析构" << endl;
    }
    string groupname;
    Person p1;
    Person p2;
    Person p3;
};

int main()
{
    // 构造函数的调用
    // 1 调用无参构造函数
    // 如果用户提供有参构造，编译器不会提供默认构造，会提供拷贝构造,此时如果用户自己没有提供默认构造，会出错
    Person p; // 调用无参构造函数

    // 2 调用有参构造函数
    // 2.1  括号法，常用
    Person p1(10, 60);
    Person p11(10, 60, 100);
    // 注意1：调用无参构造函数不能加括号，如果加了编译器认为这是一个函数声明
    // Person p2();
    // 2.2 显式法
    Person p2 = Person(10, 60);
    // Person(10)单独写就是匿名对象  当前行结束之后，马上析构
    // 2.3 隐式转换法，只能在只有一个参数时使用
    Person p4 = 10; // Person p4 = Person(10);

    // 3 调用拷贝函数
    //     使用一个已经创建完毕的对象来初始化一个新对象
    //     值传递的方式给函数参数传值
    //     以值方式返回局部对象
    // 3.1 使用一个已经创建完毕的对象来初始化一个新对象
    //  如果不写拷贝构造，编译器会自动添加拷贝构造，并且做浅拷贝操作
    //  如果用户只提供拷贝构造，编译器不会提供其他构造函数，因此上面那些全会出错
    // 括号法
    Person p5(p1);
    // 显式法
    Person p3 = Person(p2);
    // 隐式转换法
    Person p6 = p4; // Person p6 = Person(p4);
    // 注意2：不能利用 拷贝构造函数 初始化匿名对象 编译器认为是对象声明
    // Person p5(p4);
    // 3.2 值传递的方式给函数参数传值;
    doWork(p2);
    // 3.3 以值方式返回局部对象
    Person p7 = doWork2();
    cout << (int *)&p7 << endl;

    cout << endl;
    // 4 当类中成员有其他类对象时，构造的顺序是 ：先调用对象成员的构造，再调用本类构造
    // 析构顺序与构造相反，下面多出来三个构造与析构，应该是
    group g("g1", p1, p2, p3);

    // 5 静态成员变量两种访问方式
    // 5.1 通过对象
    Person ps1;
    ps1.m_A = 100;
    cout << "ps1.m_A = " << ps1.m_A << endl;
    Person ps2;
    ps2.m_A = 200;
    cout << "ps1.m_A = " << ps1.m_A << endl;
    cout << "ps2.m_A = " << ps2.m_A << endl; // 所有对象共享同一份数据
    // 5.2 通过类名
    cout << "m_A = " << Person::m_A << endl;
    // cout << "m_B = " << Person::m_B << endl; //私有权限访问不到
    // 5.3 通过对象
    Person pf1;
    pf1.staticfunc();
    // 5.4 通过类名
    Person::staticfunc();

    // 6 this指针指向被调用的成员函数所属的对象
    p11.printAge(14);
    Person pt = p11.returnself();
    // 先定义后复制会出现两次析构（复制的时候析构原对象pt）的段错误，
    // 原因是在返回对象时发生了深拷贝，导致内存重复释放（浅拷贝时不会）。
    // Person pt;
    // pt = p11.returnself(); // free(): double free detected in tcache 2   Aborted (core dumped)
    cout << "pt.age = " << pt.age << endl;
    // 可以尝试使用返回引用的方式来避免对象的多次复制
    Person &pt2 = p11.returnself2();
    cout << "pt2.age = " << pt2.age << endl;

    // 7 空指针访问成员函数
    Person *pn = NULL;
    pn->staticfunc(); // 空指针，可以调用成员函数
    // pn->ShowPerson(); // 但是如果成员函数中用到了this指针，就不可以了 Segmentation fault (core dumped)

    // 8 常对象：
    //     声明对象前加const称该对象为常对象
    //     常对象只能调用常函数
    const Person person; // 常对象
    // person.age = 50; //常对象不能修改成员变量的值,但是可以访问
    person.mut_B = 100; // 但是常对象可以修改mutable修饰成员变量
    // 常对象访问成员函数
    person.ShowPerson(); // 常对象只能调用const的函数
    person.staticfunc();
    // person.printAge(); // 不等调用正常的函数
    cout << person.mut_B << endl;

    /* 9 动态分配内存,对象没有名称，但可通过指针访问，new申请的对象，则只有调用到delete时才会执行析构函数，如果程序退出而没有执行delete则会造成内存泄漏。
    new创建类对象与不new区别：
        new创建类对象需要指针接收，一处初始化，多处使用
        new创建类对象使用完需delete销毁
        new创建对象直接使用堆空间，而局部不用new定义类对象则使用栈空间
        new对象指针用途广泛，比如作为函数返回值、函数参数等
        频繁调用场合并不适合new，就像new申请和释放内存一样*/
    Person *stu3 = new Person(10, 60, 100);
    delete stu3; // 动态分配对象此时才调用析构函数

    // 对象数组(使用时必须有默认构造函数)
    Person stu[5];
     // 先定义后复制会出现两次析构
    // stu[0] = p1;
    // stu[1] = p2;
    stu[1].printAge(121);

    cout << endl;
    // 10 程序结束销毁所有对象，调用析构函数
    return 0;
}