#include <iostream>
#include <cstring>
using namespace std;
/*
数组：是一个集合，里面存放了相同类型的数据元素
    数组中的每个数据元素都是相同的数据类型
    数组是由连续的内存位置组成的
    数组名的命名规范与变量名命名规范一致，不要和变量重名
    数组中下标是从0开始索引
*/

int main()
{
    // 1 一维数组
    // 一维数组定义的三种方式：
    // 数据类型 数组名[ 数组长度 ];
    // 数据类型 数组名[ 数组长度 ] = { 值1，值2 ...};
    // 数据类型 数组名[ ] = { 值1，值2 ...};
    // 1.1 定义方式1
    int score[10];
    // 利用下标赋值
    score[0] = 100;
    score[1] = 99;
    score[2] = 85;
    // 利用下标输出
    cout << score[0] << endl;
    cout << score[1] << endl;
    cout << score[2] << endl;
    // 1.2 第二种定义方式
    // 如果{}内不足10个数据，剩余数据用0补全
    int score2[10] = {100, 90, 80, 70, 60, 50, 40, 30};
    // 一个一个输出太麻烦，因此可以利用循环进行遍历
    for (int i = 0; i < 10; i++)
    {
        cout << score2[i] << endl;
    }
    // 1.3 定义方式3
    // 数据类型 数组名[] =  {值1，值2 ，值3 ...};
    int score3[] = {100, 90, 80, 70, 60, 50, 40, 30, 20, 10};
    // 1.4 一维数组名称的用途：
    //     可以统计整个数组在内存中的长度
    //     可以获取数组在内存中的首地址
    cout << score3 << endl;            // 输出数组首地址（16进制）
    cout << (long long)score3 << endl; // 十进制
    // cout << "数组中第一个元素地址为： " << (int)&score3[0] << endl;     // 十进制
    // 上面这一行报错如下 ：
    // cast from ‘int*’ to ‘int’ loses precision [-fpermissive]
    // 这里是因为基于Linux内核的64位系统上指针类型占用8个字节，而int类型占用4个字节，所以会出现loses precision。
    cout << "数组中第一个元素地址为： " << (long long)&score3[0] << endl; // 十进制
    cout << sizeof(score3) << endl;                                       // 整个数组所占内存字节数
    cout << sizeof(score3) / sizeof(score3[0]) << endl;                   // 数组元素个数
    // score3 = 100; 错误，数组名是常量，因此不可以赋值

    // 1.5 字符数组与字符串
    char arr3[] = {'5', 'u', 'a', '\0', 'e'};
    // 字符数组名保存的是首字符的地址，但是cout遇到字符类型的数据会一直按地址递增打印下去，直到遇到空字符（\0）
    cout << arr3 << endl;
    // 输出字符数组首地址
    printf("%p\n", arr3);
    // 本质上字符串就是带有'\0'结尾的字符数组
    char dog[7] = {'t', 'r', 97, 'm', 's'};             // 这是字符数组
    char cat[8] = {'f', 'a', 't', 'e', 'a', 's', '\0'}; // 这是字符串！
    cout << dog << endl
         << cat << endl;
    // 另一种字符串定义方法
    char brid[11] = "Mr. Cheeps";
    cout << brid << endl;
    cout << "ab"
            "cd"
         << endl; // 字符串自动拼接
                  // 终端获取字符串
    char name[20];
    cin.getline(name, 20); // 读取整行，换行符结束
    cout << name << endl;
    // 使用string类字符串
    string str1, str2;
    str1 = "slam ";
    str2 = "is difficult";
    cout << str1 + str2 << endl;          // string拼接
    cout << strcmp("abc", "abc") << endl; // 字符串比较
    cout << strcmp("fat", cat) << endl;
    cout << strcmp("fateasaa", cat) << endl;

    // 2 二维数组，在一维数组上，多加一个维度。
    // 2.1 二维数组定义的四种方式：
    //     数据类型 数组名[ 行数 ][ 列数 ];
    //     数据类型 数组名[ 行数 ][ 列数 ] = { {数据1，数据2 } ，{数据3，数据4 } };
    //     数据类型 数组名[ 行数 ][ 列数 ] = { 数据1，数据2，数据3，数据4};
    //     数据类型 数组名[ ][ 列数 ] = { 数据1，数据2，数据3，数据4};
    int tp[2][3] =
        {
            {100, 2, 003},
            {4, 50, 60000}};
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cout << tp[i][j] << "\t";
        }
        cout << endl;
    }
    // 智能分配
    int tp1[2][3] = {100, 2, 003, 4, 50, 60000};
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cout << tp1[i][j] << "\t";
        }
        cout << endl;
    }
    // 在定义二维数组时，如果初始化了数据，可以省略行数
    int tp2[][3] = {100, 2, 003, 4, 50, 60000};
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cout << tp2[i][j] << "\t";
        }
        cout << endl;
    }
    // 2.2 二维数组数组名
    //     查看二维数组所占内存空间
    //     获取二维数组首地址
    // 二维数组数组名
    int arr[2][3] =
        {
            {1, 2, 3},
            {4, 5, 6}};
    cout << "二维数组大小： " << sizeof(arr) << endl;
    cout << "二维数组一行大小： " << sizeof(arr[0]) << endl;
    cout << "二维数组元素大小： " << sizeof(arr[0][0]) << endl;
    cout << "二维数组行数： " << sizeof(arr) / sizeof(arr[0]) << endl;
    cout << "二维数组列数： " << sizeof(arr[0]) / sizeof(arr[0][0]) << endl;
    // 地址
    cout << "二维数组首地址：" << arr << endl;
    cout << "二维数组第一行地址：" << arr[0] << endl;
    cout << "二维数组第二行地址：" << arr[1] << endl;
    cout << "二维数组第一个元素地址：" << &arr[0][0] << endl;
    cout << "二维数组第二个元素地址：" << &arr[0][1] << endl;

    // 3 案例：
    // 冒泡排序：最常用的排序算法，对数组内元素进行排序
    // 比较相邻的元素。如果第一个比第二个大，就交换他们两个。
    // 对每一对相邻元素做同样的工作，执行完毕后，找到第一个最大值。
    // 重复以上的步骤，每次比较次数-1，直到不需要比较
    // 示例： 将数组 { 4,2,8,0,5,7,1,3,9 } 进行升序排序
    int arr1[9] = {4, 2, 8, 0, 5, 7, 1, 3, 9};
    for (int i = 0; i < 9 - 1; i++)
    {
        for (int j = 0; j < 9 - 1 - i; j++)
        {
            if (arr1[j] > arr1[j + 1])
            {
                int temp = arr1[j];
                arr1[j] = arr1[j + 1];
                arr1[j + 1] = temp;
            }
        }
    }
    for (int i = 0; i < 9; i++)
    {
        cout << arr[i] << endl;
    }

    return 0;
}
