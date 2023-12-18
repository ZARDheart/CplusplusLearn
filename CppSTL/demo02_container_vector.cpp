#include <iostream>
#include <string>
#include <vector>
using namespace std;
/*
vector 数据结构和数组非常相似，也称为单端数组
vector与普通数组不同之处在于数组是静态空间，而vector可以动态扩展
动态扩展：并不是在原空间之后续接新空间，而是找更大的内存空间，然后将原数据拷贝新空间，释放原空间
vector容器的迭代器是支持随机访问的
*/
void printVector(vector<int> &v)
{

    for (vector<int>::iterator it = v.begin(); it != v.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;
}
// 重载
void printVector(vector<double> &v)
{

    for (vector<double>::iterator it = v.begin(); it != v.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;
}

int main()
{
    // 1 vector构造函数 函数原型：
    //     vector<T> v; //采用模板实现类实现，默认构造函数
    //     vector(v.begin(), v.end()); //将v[begin(), end())区间中的元素拷贝给本身。
    //     vector(n, elem); //构造函数将n个elem拷贝给本身。
    //     vector(const vector &vec); //拷贝构造函数。
    vector<int> v1; // 无参构造
    for (int i = 0; i < 10; i++)
    {
        // 尾插
        v1.push_back(i);
    }
    printVector(v1);
    vector<int> v2(v1.begin(), v1.end());
    printVector(v2);
    vector<int> v3(10, 100);
    printVector(v3);
    // 显式构造
    vector<double> vn = vector<double>(5, 0.123);
    printVector(vn);
    vector<int> v4(v3);
    printVector(v4);
    // 使用数组切片
    double arr2[] = {0.1, 0.2, 0.3, 4.5};
    vector<double> vd4(arr2, arr2 + 4);
    printVector(vd4);

    cout << "2----------------------------" << endl;

    // 2 vector赋值操作 函数原型：
    //     vector& operator=(const vector &vec);//重载等号操作符
    //     assign(beg, end); //将[beg, end)区间中的数据拷贝赋值给本身。
    //     assign(n, elem); //将n个elem拷贝赋值给本身。
    v2 = v1;
    printVector(v2);
    v3.assign(v1.begin() + 1, v1.end() - 3);
    printVector(v3);
    v4.assign(10, 100);
    printVector(v4);

    cout << "3----------------------------" << endl;

    // 3  vector数据存取操作
    //     at(int idx); //返回索引idx所指的数据
    //     operator[]; //返回索引idx所指的数据
    //     front(); //返回容器中第一个数据元素
    //     back(); //返回容器中最后一个数据元素
    for (int i = 0; i < v1.size(); i++)
    {
        cout << v1[i] << " ";
    }
    cout << endl;

    for (int i = 0; i < v1.size(); i++)
    {
        cout << v1.at(i) << " ";
    }
    cout << endl;
    cout << "v1的第一个元素为： " << v1.front() << endl;
    cout << "v1的最后一个元素为： " << v1.back() << endl;

    cout << "4----------------------------" << endl;

    // 4 对vector容器的容量和大小操作
    // 函数原型：
    //     empty(); //判断容器是否为空
    //     capacity(); //容器的容量
    //     size(); //返回容器中元素的个数
    //     resize(int num); //重新指定容器的长度为num，若容器变长，则以默认值0填充新位置。
    //     resize(int num, elem); //重新指定容器的长度为num，若容器变长，则以elem值填充新位置。
    //     ​//resize如果容器变短，则末尾超出容器长度的元素被删除
    if (v1.empty())
    {
        cout << "v1为空" << endl;
    }
    else
    {
        cout << "v1不为空" << endl;
        cout << "v1的容量 = " << v1.capacity() << endl;
        cout << "v1的大小 = " << v1.size() << endl;
    }
    // resize 重新指定大小 ，若指定的更大，默认用0填充新位置，可以利用重载版本替换默认填充
    v1.resize(15, 10);
    printVector(v1);
    // resize 重新指定大小 ，若指定的更小，超出部分元素被删除
    v1.resize(6);
    printVector(v1);

    cout << "5----------------------------" << endl;

    // 5 对vector容器进行插入、删除操作
    // 函数原型：
    //     push_back(ele); //尾部插入元素ele
    //     pop_back(); //删除最后一个元素
    //     insert(const_iterator pos, ele); //迭代器指向位置pos插入元素ele
    //     insert(const_iterator pos, int count,ele);//迭代器指向位置pos插入count个元素ele
    //     erase(const_iterator pos); //删除迭代器指向的元素
    //     erase(const_iterator start, const_iterator end);//删除迭代器从start到end之间的元素
    //     clear(); //删除容器中所有元素
    printVector(v1);
    // 尾插
    v1.push_back(123456789);
    // 尾删
    printVector(v1);
    v1.pop_back();
    printVector(v1);
    // 插入
    v1.insert(v1.begin(), 100);
    printVector(v1);
    v1.insert(v1.begin(), 2, 1000);
    printVector(v1);
    // 删除
    v1.erase(v1.begin());
    printVector(v1);
    v1.erase(v1.begin() + 1, v1.begin() + 3);
    printVector(v1);
    // 清空
    v1.clear(); // v1.erase(v1.begin(), v1.end());
    printVector(v1);

    cout << "6----------------------------" << endl;

    // 6 vector互换容器
    //      swap(vec); // 将vec与本身的元素互换
    // 互换容器
    printVector(v2);
    printVector(v3);
    cout << "互换后" << endl;
    v2.swap(v3);
    printVector(v2);
    printVector(v3);

    cout << "7----------------------------" << endl;

    // 7 vector预留空间 减少vector在动态扩展容量时的扩展次数
    //     reserve(int len);//容器预留len个元素长度，预留位置不初始化，元素不可访问
    //      如果数据量较大，可以一开始利用reserve预留空间
    vector<int> v;
    // 预留空间
    v.reserve(100000);
    int num = 0; // 统计开辟空间的次数
    int *p = NULL;
    // 统计开辟新空间的次数
    for (int i = 0; i < 100000; i++)
    {
        v.push_back(i);
        // 不相等说明首地址已经更换
        if (p != &v[0])
        {
            // p指向首地址
            p = &v[0];
            num++;
        }
    }
    cout << "num:" << num << endl;

    return 0;
}