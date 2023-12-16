#include <iostream>
using namespace std;
/*
友元的目的就是单独地让一个 函数或者类 访问另一个类中私有成员 关键字为 friend
三种实现
    全局函数做友元
    类做友元
    成员函数做友元
*/
// 先声明一下
class Building;
// 放这里是因为成员函数做友元需要先定义类
class goodfriend2
{
public:
    // 定义放后面，因为要先定义Building
    goodfriend2();
    // 只让visit函数作为Building的好朋友，可以访问Building中私有内容
    void visit();
    void visit2();

private:
    Building *building;
};

class Building
{
    // 1 全局函数做友元 告诉编译器 goodfriend 是 Building类的好朋友，可以访问类中的私有内容
    friend void goodfriend(Building *building);
    // 2 类做友元 告诉编译器 goodGay类是Building类的好朋友，可以访问到Building类中私有内容
    friend class goodGay;
    // 3 成员函数做友元 告诉编译器  goodfriend2类中的visit成员函数 是Building好朋友，可以访问私有内容
    friend void goodfriend2::visit();

public:
    Building()
    {
        this->m_SittingRoom = "客厅";
        this->m_BedRoom = "卧室";
    }

public:
    string m_SittingRoom; // 客厅

private:
    string m_BedRoom; // 卧室
};

void goodfriend(Building *building)
{
    cout << "好基友正在访问： " << building->m_SittingRoom << endl;
    cout << "好基友正在访问： " << building->m_BedRoom << endl;
}

class goodGay
{
public:
    goodGay()
    {
        building = new Building;
    }
    void visit()
    {
        cout << "好基友正在访问" << building->m_SittingRoom << endl;
        cout << "好基友正在访问" << building->m_BedRoom << endl;
    }

private:
    Building *building;
};

goodfriend2::goodfriend2()
{
    building = new Building;
}
void goodfriend2::visit()
{
    cout << "好基友正在访问" << building->m_SittingRoom << endl;
    cout << "好基友正在访问" << building->m_BedRoom << endl;
}
void goodfriend2::visit2()
{
    cout << "好基友正在访问" << building->m_SittingRoom << endl;
    // cout << "好基友正在访问" << building->m_BedRoom << endl; // 访问不到
}

int main()
{
    // 1 全局函数做友元
    Building b;
    goodfriend(&b);

    // 2 类做友元
    goodGay gg;
    gg.visit();

    // 3 成员函数做友元
    goodfriend2 gg2;
    gg2.visit();

    return 0;
}