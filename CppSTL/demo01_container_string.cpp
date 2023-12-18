#include <iostream>
#include <string>
using namespace std;

/*
string本质：string是C++风格的字符串，而string本质上是一个类
string和char * 区别：
    char * 是一个指针
    string是一个类，类内部封装了char*，管理这个字符串，是一个char*型的容器。
特点：string 类内部封装了很多成员方法
    例如：查找find，拷贝copy，删除delete 替换replace，插入insert
string管理char*所分配的内存，不用担心复制越界和取值越界等，由类内部进行负责

3.1.2 string构造函数
string(); //创建一个空的字符串 例如: string str;
string(const char* s); //使用字符串s初始化
string(const string& str); //使用一个string对象初始化另一个string对象
string(int n, char c); //使用n个字符c初始化
*/

int main()
{
    // 1 string构造函数
    // string(); //调用无参构造函数 创建一个空的字符串 例如: string str;
    string s1;
    cout << "str1 = " << s1 << endl;

    // string(const char* s); //使用字符串s初始化，c_string转换成了string
    const char *str = "hello world";
    string s2(str);
    cout << "str2 = " << s2 << endl;

    // string(const string& str); //调用拷贝构造函数，使用一个string对象初始化另一个string对象
    string s3(s2);
    cout << "str3 = " << s3 << endl;

    // string(int n, char c); //使用n个字符c初始化
    string s4(10, 'a');
    cout << "str3 = " << s3 << endl;

    // 2 赋值操作 赋值的函数原型：
    //     string& operator=(const char* s); //char*类型字符串 赋值给当前的字符串
    //     string& operator=(const string &s); //把字符串s赋给当前的字符串
    //     string& operator=(char c); //字符赋值给当前的字符串
    //     string& assign(const char *s); //把字符串s赋给当前的字符串
    //     string& assign(const char *s, int n); //把字符串s的前n个字符赋给当前的字符串
    //     string& assign(const string &s); //把字符串s赋给当前字符串
    //     string& assign(int n, char c); //用n个字符c赋给当前字符串
    string str1 = "hello world";
    cout << "str1 = " << str1 << endl;
    string str2 = str1;
    cout << "str2 = " << str2 << endl;
    string str3;
    str3 = 'a';
    cout << "str3 = " << str3 << endl;
    string str4;
    str4.assign("hello c++");
    cout << "str4 = " << str4 << endl;
    string str5;
    str5.assign("hello c++", 5);
    cout << "str5 = " << str5 << endl;
    string str6;
    str6.assign(str5);
    cout << "str6 = " << str6 << endl;
    string str7;
    str7.assign(5, 'x');
    cout << "str7 = " << str7 << endl;

    // 3 string字符串拼接 函数原型：
    //     string& operator+=(const char* str); //重载+=操作符
    //     string& operator+=(const char c); //重载+=操作符
    //     string& operator+=(const string& str); //重载+=操作符
    //     string& append(const char *s); //把字符串s连接到当前字符串结尾
    //     string& append(const char *s, int n); //把字符串s的前n个字符连接到当前字符串结尾
    //     string& append(const string &s); //同operator+=(const string& str)
    //     string& append(const string &s, int pos, int n);//字符串s中从pos开始的n个字符连接到字符串结尾
    str1 = "我";
    str1 += "爱玩游戏";
    cout << "str1 = " << str1 << endl;
    str1 += ':';
    cout << "str1 = " << str1 << endl;
    str2 = "Sky of Light";
    str1 += str2;
    cout << "str1 = " << str1 << endl;
    str3 = "I";
    str3.append(" love ");
    str3.append("game Sky !", 8); // 前n个字符连接到当前字符串结尾
    // str3.append(str2);
    cout << "str3 = " << str3 << endl;
    str3.append(str2, 3, 9); // 从下标3位置开始 ，截取9个字符，拼接到字符串末尾
    cout << "str3 = " << str3 << endl;

    // 4 string查找和替换
    // 查找：查找指定字符串是否存在
    //     int find(const string& str, int pos = 0) const; //查找str第一次出现位置,从pos开始查找
    //     int find(const char* s, int pos = 0) const; //查找s第一次出现位置,从pos开始查找
    //     int find(const char* s, int pos, int n) const; //从pos位置查找s的前n个字符第一次位置
    //     int find(const char c, int pos = 0) const; //查找字符c第一次出现位置
    //     int rfind(const string& str, int pos = npos) const; //查找str最后一次位置,从pos开始查找
    //     int rfind(const char* s, int pos = npos) const; //查找s最后一次出现位置,从pos开始查找
    //     int rfind(const char* s, int pos, int n) const; //从pos查找s的前n个字符最后一次位置
    //     int rfind(const char c, int pos = 0) const; //查找字符c最后一次出现位置
    str1 = "abcdefgde";
    int pos = str1.find("de");
    if (pos == -1)
    {
        cout << "未找到" << endl;
    }
    else
    {
        cout << "pos = " << pos << endl;
    }
    pos = str1.rfind("de");
    cout << "pos = " << pos << endl;
    // 替换：在指定的位置替换字符串（指定的字符片段，整体替换成新的字符串）
    //     string& replace(int pos, int n, const string& str); //替换从pos开始n个字符为字符串str
    //     string& replace(int pos, int n,const char* s); //替换从pos开始的n个字符为字符串s
    str1.replace(2, 2, "1111"); // 指定的字符片段(2，2):cd->1111，整体替换成新的字符串
    cout << "str1 = " << str1 << endl;

    // 5 string字符串比较:字符串比较是按字符的ASCII码进行对比
    //     = 返回 0
    //     > 返回 1
    //     < 返回 -1
    // 函数原型：
    //     int compare(const string &s) const; //与字符串s比较
    //     int compare(const char *s) const; //与字符串s比较
    // 字符串对比主要是用于比较两个字符串是否相等，判断谁大谁小的意义并不是很大
    s1 = "hello";
    s2 = "aello";
    int ret = s1.compare(s2);
    if (ret == 0)
    {
        cout << "s1 等于 s2" << endl;
    }
    else if (ret > 0)
    {
        cout << "s1 大于 s2" << endl;
    }
    else
    {
        cout << "s1 小于 s2" << endl;
    }

    // 6 string字符存取.string中单个字符存取方式有两种
    //     char& operator[](int n); //通过[]方式取字符
    //     char& at(int n); //通过at方法获取字符
    str1 = "hello world";
    for (int i = 0; i < str1.size(); i++)
    {
        cout << str1[i] << " ";
    }
    cout << endl;
    for (int i = 0; i < str1.size(); i++)
    {
        cout << str1.at(i) << " ";
    }
    cout << endl;
    // 字符修改
    str1[0] = 'x';
    str1.at(1) = 'x';
    cout << str1 << endl;

    // 7 string插入和删除对string字符串进行插入和删除字符操作
    // 函数原型：插入和删除的起始下标都是从0开始
    //     string& insert(int pos, const char* s); //插入字符串
    //     string& insert(int pos, const string& str); //插入字符串
    //     string& insert(int pos, int n, char c); //在指定位置插入n个字符c
    //     string& erase(int pos, int n = npos); //删除从Pos开始的n个字符
    str1.insert(1, "111");
    cout << str1 << endl;
    str1.erase(1, 3); // 从1号位置开始3个字符
    cout << str1 << endl;

    // 8 string子串：从字符串中获取想要的子串
    // 函数原型：
    //     string substr(int pos = 0, int n = npos) const; //返回由pos开始的n个字符组成的字符串
    str1 = "abcdefg";
    string subStr = str1.substr(1, 3);
    cout << "subStr = " << subStr << endl;
    string email = "hello@sina.com";
    pos = email.find("@");
    string username = email.substr(0, pos);
    cout << "username: " << username << endl;

    return 0;
}