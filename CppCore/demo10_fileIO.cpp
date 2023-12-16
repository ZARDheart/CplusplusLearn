#include <iostream>
#include <cstring>
#include <sstream> //字符串流支持
#include <fstream> //读写文本
#include <cstdlib> //控制程序
using namespace std;

/*
C++中对文件操作需要包含头文件 < fstream >
文件类型分为两种：
    文本文件 - 文件以文本的ASCII码形式存储在计算机中
    二进制文件 - 文件以文本的二进制形式存储在计算机中，用户一般不能直接读懂它们
操作文件的三大类:
    ofstream：写操作
    ifstream： 读操作
    fstream ： 读写操作
*/

// 以二进制文件读写一个类的实例对象
class Person
{
public:
    // 使用string容易出现问题
    char m_Name[64];
    int m_Age;
};

int main()
{
    double a = 1.23456789;

    // 1 写入数据:
    ofstream testFile;
    testFile.open("../config/demo10_test.txt", ios::out); // 文件夹里没有自动创建
    // 文件打开方式：
    //     ios::in	为读文件而打开文件
    //     ios::out	为写文件而打开文件
    //     ios::ate	初始位置：文件尾
    //     ios::app	追加方式写文件
    //     ios::trunc	如果文件存在先删除，再创建
    //     ios::binary	二进制方式
    if (!testFile.is_open()) // 文件是否打开
    {
        cout << "文件打开失败" << endl;
        exit(EXIT_FAILURE); // 终止程序
    }
    testFile << fixed;     // 开始写入,与cout相同
    testFile.precision(5); // 写入小数精度
    testFile << "Test Text!" << endl
             << "line2 ：" << a; // 写入数据（覆盖）
    testFile.close();

    // 2 读取数据
    // 2.1 以字符读取
    ifstream ifs;
    ifs.open("../config/demo10_test.txt", ios::in);
    if (!ifs.is_open())
    {
        cout << "文件打开失败" << endl;
        exit(EXIT_FAILURE); // 终止程序
    }
    // 第一种方式 字符数组，遇到空格和换行会换行（就是以空格和换行分开字符串）
    // char buf[1024] = {0};
    // while (ifs >> buf)
    // {
    //     cout << buf << endl;
    // }
    // 第二种 按行读取，遇到换行会换行（就是以换行分开字符串）
    char buf[1024] = {0};
    while (ifs.getline(buf, sizeof(buf)))
    {
        cout << buf << endl;
    }
    // 第三种 字符串，与第二种相同
    // string buf;
    // while (getline(ifs, buf))
    // {
    //     cout << buf << endl;
    // }
    // 第四种 逐个字符读取
    // char c;
    // while ((c = ifs.get()) != EOF)
    // {
    //     cout << c;
    // }
    ifs.close();
    cout << endl;
    // 2.2 按行和空格读取特定格式（每行固定且含义已知）的数据：
    ifs.open("../config/demo10_desk.txt", ios::in);
    while (!ifs.eof())
    {
        string s;
        getline(ifs, s); // 读取一行的内容到字符串s中
        if (!s.empty())  // 如果不是空行就可以分析数据了
        {
            stringstream ss; // 字符串流
            ss << s;
            // 字符串格式:  时间戳 rgb图像路径 时间戳 深度图像路径
            double t;
            string sRGB, sD;
            ss >> t;
            cout << t << " ";
            ss >> sRGB;
            cout << sRGB << " ";
            ss >> t;
            cout << t << " ";
            ss >> sD;
            cout << sD << endl;
        }
    }
    ifs.close();
    cout << endl;
    // 2.3 只包含数字，以空格和换行分开逐个数字读取（最后一个数字后面要有空格或者换行）
    char filename[50] = "../config/demo10_test1.txt";
    ifs.open(filename);
    double wt = 0, sum = 0;
    int count = 0;
    ifs >> wt;
    while (ifs.good()) // 读取数据（数字）操作成功否
    {
        count++;
        sum += wt;
        ifs >> wt; // 逐个数据读取
    }
    // 已经走到结尾，说明全部是数字
    if (ifs.eof())
        cout << "End of file." << endl;
    else if (ifs.fill())
        cout << "Data type error." << endl;
    else
        cout << "Unknowm reason." << endl;
    if (count == 0)
        cout << "No data.";
    else
    {
        cout << count << " number" << endl;
        cout << "sum: " << sum << endl;
    }
    ifs.close();

    // 3 以二进制的方式对文件进行读写操作
    // 打开方式要指定为 ios::binary
    // 3.1 写文件
    //     函数原型 ：ostream& write(const char * buffer,int len);
    //     参数解释：字符指针buffer指向内存中一段存储空间。len是读写的字节数
    ofstream ofs("../config/person.txt", ios::out | ios::binary);
    if (!ofs.is_open())
    {
        cout << "文件打开失败" << endl;
        exit(EXIT_FAILURE); // 终止程序
    }
    Person p = {"张三", 18};
    ofs.write((const char *)&p, sizeof(p));
    ofs.close();
    // 3.2 读文件
    //     函数原型：istream& read(char *buffer,int len);
    //     参数解释：字符指针buffer指向内存中一段存储空间。len是读写的字节数
    ifs.open("../config/person.txt", ios::in | ios::binary);
    if (!ifs.is_open())
    {
        cout << "文件打开失败" << endl;
        exit(EXIT_FAILURE); // 终止程序
    }
    Person pr;
    ifs.read((char *)&pr, sizeof(pr));
    cout << "姓名： " << pr.m_Name << " 年龄： " << pr.m_Age << endl;

    return 0;
}
