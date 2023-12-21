#include <iostream>
#include <set>
using namespace std;

/*
set所有元素都会在插入时自动被排序
本质：set/multiset属于关联式容器，底层结构是用二叉树实现。
set和multiset区别：
    set不允许容器中有重复的元素,multiset允许容器中有重复的元素
    set插入数据的同时会返回插入结果，表示插入是否成功,multiset不会检测数据，因此可以插入重复数据
*/

void printSet(set<int> &s)
{
    for (set<int>::iterator it = s.begin(); it != s.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;
}
class MyCompare
{
public:
    bool operator()(int v1, int v2)
    {
        return v1 > v2;
    }
};

int main()
{
    // 1 set构造和赋值
    //     set<T> st; //默认构造函数：
    //     set(const set &st); //拷贝构造函数
    //     set& operator=(const set &st); //重载等号操作符
    set<int> s1;
    s1.insert(10);
    s1.insert(30);
    s1.insert(30); // 插入相同的,失败
    s1.insert(20);
    s1.insert(40);
    printSet(s1);
    // 拷贝构造
    set<int> s2(s1);
    printSet(s2);
    // 赋值
    set<int> s3;
    s3 = s2;
    printSet(s3);

    // 2 set统计set容器大小以及交换set容器
    //     size(); //返回容器中元素的数目
    //     empty(); //判断容器是否为空
    //     swap(st); //交换两个集合容器
    if (s1.empty())
    {
        cout << "s1为空" << endl;
    }
    else
    {
        cout << "s1不为空" << endl;
        cout << "s1的大小为： " << s1.size() << endl;
    }
    s2.insert(100);
    cout << "交换前" << endl;
    printSet(s1);
    printSet(s2);
    cout << "交换后" << endl;
    s1.swap(s2);
    printSet(s1);
    printSet(s2);

    // 3 set插入和删除数据
    //     insert(elem); //在容器中插入元素。
    //     clear(); //清除所有元素
    //     erase(pos); //删除pos迭代器所指的元素，返回下一个元素的迭代器。
    //     erase(beg, end); //删除区间[beg,end)的所有元素 ，返回下一个元素的迭代器。
    //     erase(elem); //删除容器中值为elem的元素。
    // 插入
    s1.insert(100);
    s1.insert(300);
    s1.insert(200);
    s1.insert(400);
    printSet(s1);
    // 删除
    s1.erase(s1.begin());
    printSet(s1);
    s1.erase(30);
    printSet(s1);
    // 清空
    // s1.erase(s1.begin(), s1.end());
    s1.clear();
    printSet(s1);

    // 4 对set容器进行查找数据以及统计数据
    //     find(key); //查找key是否存在,若存在，返回该键的元素的迭代器；若不存在，返回set.end();
    //     count(key); //统计key的元素个数  count （对于set，结果为0或者1）
    // 查找
    set<int>::iterator pos = s1.find(20);
    if (pos != s1.end())
    {
        cout << "找到了元素 ： " << *pos << endl;
    }
    else
    {
        cout << "未找到元素" << endl;
    }
    // 统计
    int num = s1.count(20);
    cout << "num = " << num << endl;

    // 5 set和multiset区别
    set<int> s;
    pair<set<int>::iterator, bool> ret = s.insert(10);
    if (ret.second)
    {
        cout << "第一次插入成功!" << endl;
    }
    else
    {
        cout << "第一次插入失败!" << endl;
    }
    // 重复插入，失败
    ret = s.insert(10);
    if (ret.second)
    {
        cout << "第二次插入成功!" << endl;
    }
    else
    {
        cout << "第二次插入失败!" << endl;
    }
    // multiset
    multiset<int> ms;
    for (int i = 0; i < 10; i++)
        ms.insert(10); // multiset可以重复插入
    for (multiset<int>::iterator it = ms.begin(); it != ms.end(); it++)
        cout << *it << " ";
    cout << endl;

    // 6 pair对组创建 成对出现的数据，利用对组可以返回两个数据 两种创建方式：
    //     pair<type, type> p ( value1, value2 );
    //     pair<type, type> p = make_pair( value1, value2 );
    pair<string, int> p(string("Tom"), 20);
    cout << "姓名： " << p.first << " 年龄： " << p.second << endl;
    pair<string, int> p2 = make_pair("Jerry", 10);
    cout << "姓名： " << p2.first << " 年龄： " << p2.second << endl;

    // 7 set容器排序 set容器默认排序规则为从小到大，利用仿函数，可以改变排序规则
    s1.insert(10);
    s1.insert(40);
    s1.insert(20);
    s1.insert(30);
    s1.insert(50);
    // 默认从小到大
    printSet(s1);
    // 指定排序规则
    set<int, MyCompare> s4;
    s4.insert(10);
    s4.insert(40);
    s4.insert(20);
    s4.insert(30);
    s4.insert(50);
    for (set<int, MyCompare>::iterator it = s4.begin(); it != s4.end(); it++)
        cout << *it << " ";
    cout << endl;

    return 0;
}
