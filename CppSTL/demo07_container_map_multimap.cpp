#include <iostream>
#include <vector>
#include <map>
using namespace std;
/*
map中所有元素都是pair
    pair中第一个元素为key（键值），起到索引作用，第二个元素为value（实值）
    所有元素都会根据元素的键值自动排序
本质：map/multimap属于关联式容器，底层结构是用二叉树实现。
优点：
    可以根据key值快速找到value值
map和multimap区别：
    map不允许容器中有重复key值元素
    multimap允许容器中有重复key值元素
*/

struct Info
{
    string name;
    int num;
    Info() {}
    Info(string name, int num) : name(name), num(num) {}
};
void printMap(map<int, int> &m)
{
    // 使用迭代器访问时，iter->first指向元素的键，iter->second指向键对应的值
    for (map<int, int>::iterator it = m.begin(); it != m.end(); it++)
    {
        cout << "key = " << it->first << " value = " << it->second << endl;
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
    // 0 pair对组创建 成对出现的数据，利用对组可以返回两个数据 两种创建方式：
    //     pair<type, type> p ( value1, value2 );
    //     pair<type, type> p = make_pair( value1, value2 );
    pair<string, int> p(string("Tom"), 20);
    cout << "姓名： " << p.first << " 年龄： " << p.second << endl;
    pair<string, int> p2 = make_pair("Jerry", 10);
    cout << "姓名： " << p2.first << " 年龄： " << p2.second << endl;

    // 1 map构造和赋值操作 函数原型：
    //     map<T1, T2> mp; //map默认构造函数:
    //     map(const map &mp); //拷贝构造函数
    //     map& operator=(const map &mp); //重载等号操作符
    map<int, int> m; // 默认构造
    m.insert(pair<int, int>(1, 10));
    m.insert(pair<int, int>(2, 20));
    m.insert(pair<int, int>(3, 30));
    printMap(m);
    map<int, int> m2(m); // 拷贝构造
    map<int, int> m3;
    m3 = m2; // 赋值

    // 2 map容器大小以及交换map容器 函数原型：
    //     size(); //返回容器中元素的数目
    //     empty(); //判断容器是否为空
    //     swap(st); //交换两个集合容器
    if (m.empty())
    {
        cout << "m为空" << endl;
    }
    else
    {
        cout << "m不为空" << endl;
        cout << "m的大小为： " << m.size() << endl;
    }
    m2.insert(pair<int, int>(4, 100));
    cout << "交换前" << endl;
    printMap(m);
    printMap(m2);
    cout << "交换后" << endl;
    m.swap(m2);
    printMap(m);
    printMap(m2);

    cout << "3 ---------------------------------" << endl;

    // 3 map容器进行插入数据和删除数据
    //     insert(elem); //在容器中插入元素。
    //     clear(); //清除所有元素
    //     erase(pos); //删除pos迭代器所指的元素，返回下一个元素的迭代器。
    //     erase(beg, end); //删除区间[beg,end)的所有元素 ，返回下一个元素的迭代器。
    //     erase(key); //删除容器中值为key的元素。
    // 插入
    // 第一种插入方式
    m.insert(pair<int, int>(4, 40));
    // 第二种插入方式
    m.insert(make_pair(5, 50));
    // 第三种插入方式
    m.insert(map<int, int>::value_type(6, 60));
    // 第四种插入方式
    m[7] = 70;
    printMap(m);
    // 删除
    m.erase(m.begin());
    printMap(m);
    m.erase(3);
    printMap(m);
    // 清空
    m2.erase(m2.begin(), m2.end());
    m2.clear();
    printMap(m2);

    cout << "4 ---------------------------------" << endl;

    // 4 对map容器进行查找数据以及统计数据
    //     find(key); //查找key是否存在,若存在，返回该键的元素的迭代器；若不存在，返回set.end();
    //     count(key); //统计key的元素个数 count （对于map，结果为0或者1）
    // 查找
    map<int, int>::iterator pos = m.find(4);
    if (pos != m.end())
    {
        cout << "找到了元素 key = " << (*pos).first << " value = " << (*pos).second << endl;
    }
    else
    {
        cout << "未找到元素" << endl;
    }
    // 统计
    int num = m.count(5);
    cout << "num = " << num << endl;

    cout << "5 ---------------------------------" << endl;
    // 5 map容器默认排序规则为 按照key值进行 从小到大排序，利用仿函数，可以改变排序规则
    map<int, int, MyCompare> mf;
    mf.insert(make_pair(1, 10));
    mf.insert(make_pair(2, 20));
    mf.insert(make_pair(3, 30));
    mf.insert(make_pair(4, 40));
    mf.insert(make_pair(5, 50));
    for (map<int, int, MyCompare>::iterator it = mf.begin(); it != mf.end(); it++)
    {
        cout << "key:" << it->first << " value:" << it->second << endl;
    }

    cout << "6 ---------------------------------" << endl;
    // 使用下标访问map容器与使用下标访问vector的行为截然不同：用下标访问map中不存在的元素将导致在map容器中添加一个新的元素，
    // 这个元素的键即为该下标值，键所对应的值为空。
    // 自定义键值
    map<Info *, int> stus;
    for (int i = 1; i <= 5; i++)
    {
        Info *stu;
        stu = new Info("stu", i);
        stu->name = "stu" + to_string(i);
        stu->num = 220 + i;
        stus[stu] = 90 + i; // 第四种插入方式
        cout << stus[stu] << " ";
        // stus.insert(pair<Info*,int>(&stu,90 + i));
    }
    cout << endl;
    map<Info *, int>::iterator it;
    for (it = stus.begin(); it != stus.end(); it++)
    {
        Info *stu = it->first;
        int score = it->second;
        cout << (*stu).name << " " << (*stu).num << " " << score << endl;
    }

    // 其他功能函数
    // key_comp() 返回比较元素 key 的函数
    // lower_bound() 返回键值>=给定元素的第一个位置
    // max_size() 返回可以容纳的最大元素个数
    // rbegin() 返回一个指向 map 尾部的逆向迭代器
    // rend() 返回一个指向 map 头部的逆向迭代器
    // upper_bound() 返回键值>给定元素的第一个位置
    // value_comp() 返回比较元素 value 的函数

    return 0;
}
