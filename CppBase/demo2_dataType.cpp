#include <iostream>
// 字符串库
#include <string>

using namespace std;
/*
数据类型：C++规定在创建一个变量或者常量时，必须要指定出相应的数据类型，否则无法给变量分配内存
*/

int main()
{
	// 数据类型：
	// 1 整型：整型变量表示的是整数类型的数据
	// C++中能够表示整型的类型有以下几种方式，区别在于所占内存空间不同（1个字节8个bit）：
	// short(短整型)	2字节16bit	(-2^15 ~ 2^15-1)
	short int int1 = 1;
	// int(整型)	4字节32bit	(-2^31 ~ 2^31-1)
	int int2 = 2;
	// long(长整形)	Windows为4字节，Linux为4字节(32位)，8字节(64位)	(-2^31 ~ 2^31-1)
	long int int3 = 3;
	// long long(长长整形)	8字节	(-2^63 ~ 2^63-1)
	long long int int4 = 4;
	// sizeof关键字可以统计数据类型所占内存大小
	cout << sizeof(short) << "  " << sizeof(int2) << endl; // 查看变量字节数
	// 以上默认是有符号的，其最高位是正负号，无符号整型在其前面声明unsigned
	// 长度与有符号是一样的，只是由于其不带符号位，因此范围相比有符号为0 ~ 2^n-1,n为bit数
	unsigned int int5 = 5;

	// 2 浮点型：用于表示小数
	// 浮点型变量分为两种：两者的区别在于表示的有效数字范围不同
	// 单精度 float	    4字节	7位有效数字
	// 双精度 double	8字节	15～16位有效数字
	float f0 = 0.123;
	// 加f的区别在于限定常量为单精度，否则为双精度
	float f1 = 0.123f;
	double f2 = 1.123456789;
	double f3 = 3.154484142e13;
	double f4 = 3.154484142e-13;
	// 输出默认显示6位有效数字，cout.precision()修改
	cout.precision(8);
	cout << sizeof(float) << "," << sizeof(f2) << ",f2=" << f2 << ",f3=" << f3 << endl;

	// 3 字符型：字符型变量用于显示单个字符，单引号内只能有一个字符，不可以是字符串
	// C和C++中字符型变量只占用1个字节
	// 字符型变量并不是把字符本身放到内存中存储，而是将对应的ASCII编码放入到存储单元
	char ch = 'b';
	cout << "ch=" << ch << "," << sizeof(char) << "," << (int)ch << endl; // 查看字符b对应的ASCII码
	// 可以直接用ASCII给字符型变量赋值
	ch = 97;
	cout << ch << "----ASCII:" << (int)ch << ",A----ASCII:" << (int)'A' << endl; // a==97,A==65 97-65=32

	// 4 字符串：用于表示一串字符,本质上是字符数组
	char str1[] = "Linux"; // 与C语言通用
	string str2 = "Linux"; // #include<string>
	cout << str1 << "  " << str2 << endl;

	// 5 布尔型：代表真或假的值，bool类型占1个字节大小
	bool b1 = true;
	bool b2 = false;
	cout << b1 << "  " << b2 << "  " << sizeof(bool) << endl; // 1 and 0

	// 6 转义字符：用于表示一些不能显示出来的ASCII字符
	// 转义字符	含义	ASCII码值（十进制）
	// \a	警报	007
	// \b	退格(BS) ，将当前位置移到前一列	008
	// \f	换页(FF)，将当前位置移到下页开头	012
	// \n	换行(LF) ，将当前位置移到下一行开头	010
	// \r	回车(CR) ，将当前位置移到本行开头	013
	// \t	水平制表(HT) （跳到下一个TAB位置）	009
	// \v	垂直制表(VT)	011
	// \\	代表一个反斜线字符""	092
	// \’	代表一个单引号（撇号）字符	039
	// \"	代表一个双引号字符	034
	// \?	代表一个问号	063
	// \0	数字0	000
	// \ddd	8进制转义字符，d范围0~7	3位8进制
	// \xhh	16进制转义字符，h范围0~9，a~f，A~F	3位16进制
	cout << "\\\ta-\n-b\"\?" << endl;
	cout << "\a" << endl;

	return 0;
}
