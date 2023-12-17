#include <iostream>
#include <string>
using namespace std;
/*
结构体属于用户自定义的数据类型，允许用户存储不同的数据类型
语法：struct 结构体名 { 结构体成员列表 }；

通过结构体创建变量的方式有三种（定义结构体时的关键字是struct，不可省略，创建结构体变量时，struct可以省略）：
struct（可省略） 结构体名 变量名
struct（可省略） 结构体名 变量名 = { 成员1值 ， 成员2值…}
定义结构体时顺便创建变量
*/

// 结构体定义
struct student
{
    // 成员列表
    string name; // 姓名
    int age;     // 年龄
    int score;   // 分数
} stu3;          // 结构体变量创建方式3

// 结构体嵌套
struct teacher
{
    // 成员列表
    int id;             // 职工编号
    string name;        // 教师姓名
    int age;            // 教师年龄
    struct student stu; // 子结构体 学生
};

// 值传递
void printStudent(student stu)
{
    stu.age = 28;
    cout << "子函数中 姓名：" << stu.name << " 年龄： " << stu.age << " 分数：" << stu.score << endl;
}

// 地址传递
void printStudent2(student *stu)
{
    stu->age = 28;
    cout << "子函数中 姓名：" << stu->name << " 年龄： " << stu->age << " 分数：" << stu->score << endl;
}

// const使用场景：地址传递，但不能修改
void printStudent3(const student *stu) // 加const防止函数体中的误操作
{
    // stu->age = 100; //操作失败，因为加了const修饰
    cout << "姓名：" << stu->name << " 年龄：" << stu->age << " 分数：" << stu->score << endl;
}

// C++结构体也可以有成员函数
// 事实上，在 C++ 中，结构体和类的主要区别在于默认的访问权限和继承权限：
// 结构体的默认访问权限是公共的（public），而类的默认访问权限是私有的（private）。
struct infor
{
    // 成员变量
    string name;
    int age;
    double num;

    // 成员函数
    // 构造函数
    infor() {}
    infor(string name, int age, double num) : name(name), age(age), num(num) {}
    // 功能函数
    double de(int n)
    {
        return n + num;
    }
};

int main()
{
    // 1 结构体变量创建方式1
    struct student stu1; // struct 关键字可以省略
    stu1.name = "张三";
    stu1.age = 18;
    stu1.score = 100;
    cout << "姓名：" << stu1.name << " 年龄：" << stu1.age << " 分数：" << stu1.score << endl;

    // 2 结构体变量创建方式2
    student stu2 = {"李四", 19, 60};
    cout << "姓名：" << stu2.name << " 年龄：" << stu2.age << " 分数：" << stu2.score << endl;

    // 3 结构体变量创建方式3 定义时创建
    stu3.name = "王五";
    stu3.age = 18;
    stu3.score = 80;
    cout << "姓名：" << stu3.name << " 年龄：" << stu3.age << " 分数：" << stu3.score << endl;

    // 4 结构体数组：将自定义的结构体放入到数组中方便维护
    struct student arr[3] =
        {
            {"张三", 18, 80},
            {"李四", 19, 60},
            {"王五", 20, 70}};
    for (int i = 0; i < 3; i++)
    {
        cout << "姓名：" << arr[i].name << " 年龄：" << arr[i].age << " 分数：" << arr[i].score << endl;
    }

    // 5 结构体指针：通过指针访问结构体中的成员
    struct student stu = {
        "张三",
        18,
        100,
    };
    struct student *p = &stu;
    p->score = 80; // 指针通过 -> 操作符可以访问成员
    cout << "姓名：" << p->name << " 年龄：" << p->age << " 分数：" << p->score << endl;

    // 6 结构体中的成员可以是另一个结构体
    // 每个老师辅导一个学员，一个老师的结构体中，记录一个学生的结构体
    struct teacher t1;
    t1.id = 10000;
    t1.name = "老王";
    t1.age = 40;
    t1.stu.name = "张三";
    t1.stu.age = 18;
    t1.stu.score = 100;
    cout << "教师 职工编号： " << t1.id << " 姓名： " << t1.name << " 年龄： " << t1.age << endl;
    cout << "辅导学员 姓名： " << t1.stu.name << " 年龄：" << t1.stu.age << " 考试分数： " << t1.stu.score << endl;

    // 7 结构体作为函数参数
    student stu4 = {"张三", 18, 100};
    // 值传递
    printStudent(stu4);
    cout << "主函数中 姓名：" << stu4.name << " 年龄： " << stu4.age << " 分数：" << stu4.score << endl;
    // 地址传递
    printStudent2(&stu4);
    cout << "主函数中 姓名：" << stu4.name << " 年龄： " << stu4.age << " 分数：" << stu4.score << endl;
    // const修饰
    printStudent3(&stu4);

    // 8 结构体成员函数
    // 使用构造函数创建结构体
    infor wang("Wang", 14, 14.5);
    // 调用成员函数
    cout << wang.de(5) << endl;

    return 0;
}
