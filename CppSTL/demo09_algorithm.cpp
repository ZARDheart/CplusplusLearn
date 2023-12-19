#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <numeric>

using namespace std;

/*
STL- 常用算法 算法主要是由头文件<algorithm> <functional> <numeric>组成。
    <algorithm>是所有STL头文件中最大的一个，范围涉及到比较、 交换、查找、遍历操作、复制、修改等等
    <numeric>体积很小，只包括几个在序列上面进行简单数学运算的模板函数
    <functional>定义了一些模板类,用以声明函数对象。

常用遍历算法
    for_each //遍历容器
    transform //搬运容器到另一个容器中
常用查找算法
    find //查找元素
    find_if //按条件查找元素
    adjacent_find //查找相邻重复元素
    binary_search //二分查找法
    count //统计元素个数
    count_if //按条件统计元素个数
常用排序算法
    sort //对容器内元素进行排序
    random_shuffle //洗牌 指定范围内的元素随机调整次序
    merge // 容器元素合并，并存储到另一容器中
    reverse // 反转指定范围的元素
常用拷贝和替换算法
    copy // 容器内指定范围的元素拷贝到另一容器中
    replace // 将容器内指定范围的旧元素修改为新元素
    replace_if // 容器内指定范围满足条件的元素替换为新元素
    swap // 互换两个容器的元素
常用算术生成算法 属于小型算法，使用时包含的头文件为 #include <numeric>
    accumulate // 计算容器元素累计总和
    fill // 向容器中添加元素
常用集合算法
    set_intersection // 求两个容器的交集
    set_union // 求两个容器的并集
    set_difference // 求两个容器的差集
*/
// 普通函数
void MyPrintFun(int val)
{
    cout << val << " ";
}

// 函数对象
class MyPrint
{
public:
    void operator()(int val)
    {
        cout << val << " ";
    }
};

class TransForm
{
public:
    int operator()(int val)
    {
        return val;
    }
};

// Test1 遍历
void test1()
{
    cout << "Test1 遍历--------------------------------------" << endl;
    // 1 for_each 实现遍历容器
    //     for_each(iterator beg, iterator end, _func); // beg end 开始结束迭代器 _func 函数或者函数对象
    vector<int> v;
    for (int i = 0; i < 10; i++)
        v.push_back(i);
    // 遍历算法
    for_each(v.begin(), v.end(), MyPrintFun);
    cout << endl;
    for_each(v.begin(), v.end(), MyPrint());
    cout << endl;

    // 2 transform 搬运容器到另一个容器中
    //     beg1 end1 源容器开始结束迭代器 beg2 目标容器开始迭代器 _func 函数或者函数对象
    //     transform(iterator beg1, iterator end1, iterator beg2, _func);
    vector<int> vTarget;      // 目标容器
    vTarget.resize(v.size()); // 搬运的目标容器必须要提前开辟空间，否则无法正常搬运
    transform(v.begin(), v.end(), vTarget.begin(), TransForm());
    for_each(vTarget.begin(), vTarget.end(), MyPrint());
    cout << endl;
}

class Person
{
public:
    Person(string name, int age)
    {
        this->m_Name = name;
        this->m_Age = age;
    }
    // 重载==
    bool operator==(const Person &p)
    {
        if (this->m_Name == p.m_Name && this->m_Age == p.m_Age)
        {
            return true;
        }
        return false;
    }

public:
    string m_Name;
    int m_Age;
};

// >5的谓词
class GreaterFive
{
public:
    bool operator()(int val)
    {
        return val > 5;
    }
};

// m_Age大于20的谓词
class Greater20
{
public:
    bool operator()(Person &p)
    {
        return p.m_Age > 20;
    }
};

void printVector(vector<int> &v)
{
    for_each(v.begin(), v.end(), MyPrintFun);
    cout << endl;
}

// Test2 查找
void test2()
{
    cout << "Test2 查找--------------------------------------" << endl;
    vector<int> v;
    for (int i = 0; i < 10; i++)
        v.push_back(i);
    // 3 find 查找指定元素，找到返回指定元素的迭代器，找不到返回结束迭代器end()
    //     // 按值查找元素，找到返回指定位置迭代器，找不到返回结束迭代器位置
    //     // beg 开始迭代器 end 结束迭代器 value 查找的元素
    //     find(iterator beg, iterator end, value);
    // 查找容器中是否有 5 这个元素
    vector<int>::iterator it = find(v.begin(), v.end(), 5);
    if (it == v.end())
    {
        cout << "没有找到!" << endl;
    }
    else
    {
        cout << "找到:" << *it << endl;
    }
    // 自定义数据类型
    vector<Person> vp;
    // 创建数据
    Person p1("aaa", 10);
    Person p2("bbb", 20);
    Person p3("ccc", 30);
    Person p4("ddd", 40);
    vp.push_back(p1);
    vp.push_back(p2);
    vp.push_back(p3);
    vp.push_back(p3);
    vp.push_back(p4);
    vector<Person>::iterator itp = find(vp.begin(), vp.end(), p2); // 需要重载==
    if (itp == vp.end())
    {
        cout << "没有找到!" << endl;
    }
    else
    {
        cout << "找到此人： 姓名:" << itp->m_Name << " 年龄: " << itp->m_Age << endl;
    }

    // 4 find_if 按条件查找元素
    //     // 按值查找元素，找到返回指定位置迭代器，找不到返回结束迭代器位置
    //     // beg 开始迭代器 end 结束迭代器 _Pred 函数或者谓词（返回bool类型的仿函数）
    //     find_if(iterator beg, iterator end, _Pred);
    it = find_if(v.begin(), v.end(), GreaterFive()); // 自定义查找规则
    if (it == v.end())
    {
        cout << "没有找到!" << endl;
    }
    else
    {
        cout << "找到大于5的数字:" << *it << endl;
    }
    itp = find_if(vp.begin(), vp.end(), Greater20());
    if (itp == vp.end())
    {
        cout << "没有找到!" << endl;
    }
    else
    {
        cout << "找到大于20岁: 姓名:" << itp->m_Name << " 年龄: " << itp->m_Age << endl;
    }

    // 5 adjacent_find 查找相邻重复元素
    //     // 查找相邻重复元素,返回相邻元素的第一个位置的迭代器 beg 开始迭代器 end 结束迭代器
    //     adjacent_find(iterator beg, iterator end);
    v.push_back(11);
    v.push_back(11);
    v.push_back(12);
    // 查找相邻重复元素
    it = adjacent_find(v.begin(), v.end());
    if (it == v.end())
    {
        cout << "找不到!" << endl;
    }
    else
    {
        cout << "找到相邻重复元素为:" << *it << endl;
    }

    // 6 binary_search 二分查找指定元素是否存在 注意: 在无序序列中不可用
    //     bool binary_search(iterator beg, iterator end, value);
    //     // 查找指定的元素，查到 返回true 否则false， beg 开始迭代器 end 结束迭代器 value 查找的元素
    bool ret = binary_search(v.begin(), v.end(), 2);
    if (ret)
    {
        cout << "找到了" << endl;
    }
    else
    {
        cout << "未找到" << endl;
    }

    // 7 count 统计元素个数 beg 开始迭代器 end 结束迭代器  value 统计的元素
    //     count(iterator beg, iterator end, value);
    int num = count(v.begin(), v.end(), 11);
    cout << "11的个数为： " << num << endl;
    num = count(vp.begin(), vp.end(), p3); // 统计自定义数据类型时候，需要配合重载 operator==
    cout << "p3 num = " << num << endl;

    // 8 count_if 按条件统计元素个数 beg 开始迭代器 end 结束迭代器  _Pred 谓词
    //     count_if(iterator beg, iterator end, _Pred);
    num = count_if(v.begin(), v.end(), GreaterFive());
    cout << "大于5的个数为： " << num << endl;
    num = count_if(vp.begin(), vp.end(), Greater20());
    cout << ">20岁的个数：" << num << endl;
}

// Test3 排序
void test3()
{
    cout << "Test3 排序--------------------------------------" << endl;
    // 9 sort 对容器内元素进行排序
    //     // 按值查找元素，找到返回指定位置迭代器，找不到返回结束迭代器位置
    //     sort(iterator beg, iterator end); // 默认升序排序
    //     // beg 开始迭代器 end 结束迭代器 _Pred 谓词
    //     sort(iterator beg, iterator end, _Pred);
    vector<int> v = {5, 1, 7, 9, 19, 20, 2, 3, 4, 8, 6, 13, 14, 16, 17, 18};
    printVector(v);
    // sort默认从小到大排序
    sort(v.begin(), v.end());
    printVector(v);
    // 从大到小排序 使用STL内建函数对象
    sort(v.begin(), v.end(), greater<int>());
    printVector(v);

    // 10 random_shuffle 洗牌 指定范围内的元素随机调整次序 beg 开始迭代器 end 结束迭代器
    //     random_shuffle(iterator beg, iterator end);
    srand((unsigned int)time(NULL)); // 使用时记得加随机数种子，否则每次洗牌都一样
    random_shuffle(v.begin(), v.end());
    printVector(v);

    // 11 merge 两个容器元素合并，并存储到另一容器中,两个容器必须是有序的（同序）,目标仍然有序
    //     // beg1 容器1开始迭代器
    //     // end1 容器1结束迭代器
    //     // beg2 容器2开始迭代器
    //     // end2 容器2结束迭代器
    //     // dest 目标容器开始迭代器
    //     merge(iterator beg1, iterator end1, iterator beg2, iterator end2, iterator dest);
    vector<int> v1 = {12, 13, 14, 16, 17, 18};
    vector<int> v2 = {2, 3, 4, 6, 7, 8};
    vector<int> vtarget;
    // 目标容器需要提前开辟空间
    vtarget.resize(v1.size() + v2.size());
    // 合并  需要两个有序序列
    merge(v1.begin(), v1.end(), v2.begin(), v2.end(), vtarget.begin());
    printVector(vtarget);

    // 12 reverse 将容器内元素进行反转 beg 开始迭代器 end 结束迭代器
    //     reverse(iterator beg, iterator end);
    cout << "反转前： " << endl;
    printVector(v);
    cout << "反转后： " << endl;
    reverse(v.begin(), v.end());
    printVector(v);
}

// Test4 拷贝和替换
void test4()
{
    cout << "Test4 拷贝和替换--------------------------------------" << endl;
    // 13 copy 容器内指定范围的元素拷贝到另一容器中
    //     // 按值查找元素，找到返回指定位置迭代器，找不到返回结束迭代器位置
    //     // beg 开始迭代器 end 结束迭代器 dest 目标起始迭代器
    //     copy(iterator beg, iterator end, iterator dest);
    vector<int> v;
    for (int i = 0; i < 10; i++)
    {
        v.push_back(i + 1);
    }
    vector<int> v2;
    v2.resize(v.size());
    copy(v.begin(), v.end(), v2.begin());
    printVector(v2);

    // 14 replace 将容器内指定范围的旧元素修改为新元素
    //     // beg 开始迭代器 end 结束迭代器 oldvalue 旧元素 newvalue 新元素
    //     replace(iterator beg, iterator end, oldvalue, newvalue);
    cout << "替换前：" << endl;
    printVector(v);
    // 将容器中的2 替换成 2000
    cout << "替换后：" << endl;
    replace(v.begin(), v.end(), 2, 2000);
    printVector(v);

    // 15 replace_if 将区间内满足条件的元素，替换成指定元素
    //     // beg 开始迭代器 end 结束迭代器  _pred 谓词 newvalue 替换的新元素
    //     replace_if(iterator beg, iterator end, _pred, newvalue);
    cout << "替换前：" << endl;
    printVector(v);
    // 将容器中大于等于的5 替换成 300
    cout << "替换后：" << endl;
    replace_if(v.begin(), v.end(), GreaterFive(), 300);
    printVector(v);

    // 16 swap 互换两个容器的元素 c1容器1 c2容器2
    //     swap(container c1, container c2);
    cout << "交换前： " << endl;
    printVector(v);
    printVector(v2);
    cout << "交换后： " << endl;
    swap(v, v2);
    printVector(v);
    printVector(v2);
}

// Test5 算术生成
void test5()
{
    cout << "Test5 算术生成--------------------------------------" << endl;
    // 17 accumulate 计算区间内 容器元素累计总和
    //     // beg 开始迭代器 end 结束迭代器 value 起始值/额外累加值
    //     accumulate(iterator beg, iterator end, value);
    vector<int> v;
    for (int i = 0; i <= 100; i++)
        v.push_back(i);
    int total = accumulate(v.begin(), v.end(), 0);
    cout << "total = " << total << endl;

    // 18 fill 向容器中填充指定的元素
    //     fill(iterator beg, iterator end, value);
    //     // beg 开始迭代器 end 结束迭代器 value 填充的值
    vector<int> v1;
    v1.resize(10);
    // 填充
    fill(v1.begin(), v1.end(), 100);
    printVector(v1);
}

// Test6 集合算法
void test6()
{
    cout << "Test6 集合算法--------------------------------------" << endl;
    // 19 set_intersection 求两个容器的交集 注意:两个集合必须是有序序列
    //     // beg1 容器1开始迭代器 end1 容器1结束迭代器
    //     // beg2 容器2开始迭代器 end2 容器2结束迭代器
    //     // dest 目标容器开始迭代器
    //     set_intersection(iterator beg1, iterator end1, iterator beg2, iterator end2, iterator dest);
    vector<int> v1;
    vector<int> v2;
    for (int i = 0; i < 10; i++)
    {
        v1.push_back(i);
        v2.push_back(i + 5);
    }
    printVector(v1);
    printVector(v2);
    vector<int> vTarget;
    // 取两个里面较小的值给目标容器开辟空间
    vTarget.resize(min(v1.size(), v2.size()));
    // 返回目标容器的最后一个元素的迭代器地址
    vector<int>::iterator itEnd =
        set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), vTarget.begin());
    printVector(vTarget);

    // 20 set_union 求两个集合的并集 注意:两个集合必须是有序序列
    //     // beg1 容器1开始迭代器 end1 容器1结束迭代器
    //     // beg2 容器2开始迭代器 end2 容器2结束迭代器
    //     // dest 目标容器开始迭代器
    //     set_union(iterator beg1, iterator end1, iterator beg2, iterator end2, iterator dest);
    // 取两个容器的和给目标容器开辟空间
    vector<int> vTarget1;
    vTarget1.resize(v1.size() + v2.size());
    // 返回目标容器的最后一个元素的迭代器地址
    itEnd = set_union(v1.begin(), v1.end(), v2.begin(), v2.end(), vTarget1.begin());
    printVector(vTarget1);

    // 21 set_difference 求两个集合的差集 注意:两个集合必须是有序序列
    //     // beg1 容器1开始迭代器 end1 容器1结束迭代器
    //     // beg2 容器2开始迭代器 end2 容器2结束迭代器
    //     // dest 目标容器开始迭代器
    //     set_difference(iterator beg1, iterator end1, iterator beg2, iterator end2, iterator dest);
    // 取两个里面较大的值给目标容器开辟空间
    vector<int> vTarget2;
    vTarget2.resize(max(v1.size(), v2.size()));
    // 返回目标容器的最后一个元素的迭代器地址
    cout << "v1与v2的差集为： " << endl; // v1中不是v1和v2的交集
    itEnd = set_difference(v1.begin(), v1.end(), v2.begin(), v2.end(), vTarget2.begin());
    printVector(vTarget2);
    cout << "v2与v1的差集为： " << endl;
    itEnd = set_difference(v2.begin(), v2.end(), v1.begin(), v1.end(), vTarget2.begin());
    printVector(vTarget2);
}

int main()
{
    // Test1 遍历
    test1();
    // Test2 查找
    test2();
    // Test3 排序
    test3();
    // Test4 拷贝和替换
    test4();
    // Test5 算术生成
    test5();
    // Test6 集合算法
    test6();

    return 0;
}