#include <iostream>
#include <string>
#include <cmath>

using namespace std;
/*
常量、变量、常量、关键字以及标识符：
*/

// 常量：用于记录程序中不可更改的数据
#define PI 3.1415926 /// 宏常量（宏函数参考函数demo）

// 关键字：C++中预先保留的单词（标识符）
// 提示：在给变量或者常量起名称时候，不要用C++得关键字，否则会产生歧义
/*
using 	namespace 	void 	return
int 	short 	long 	float 	double 	char 	sizeof
bool 	false 	true 	signed 	unsigned 	struct	enum
if	else 	switch 	case 	for 	do 	while 	continue	break 	
auto 	goto 	try 	catch
new		delete 	static 	extern 	const 	export
class 	public 	private 	protected 	virtual 	friend 	this
template typename typedef

asm  inline typeid dynamic_cast union 	mutable	
explicit static_cast	operator		
volatile const_cast wchar_t throw	
default		register 	reinterpret_cast	
*/

int main()
{
	// 1 输出：
	cout << "Hello Linux" << endl;

	// 标识符命名规则：C++规定给标识符（变量、常量）命名时，有一套自己的规则
	// 标识符不能是关键字
	// 标识符只能由字母、数字、下划线组成
	// 第一个字符必须为字母或下划线
	// 标识符中字母区分大小写

	// 2 变量：给一段指定的内存空间起名，方便操作这段内存
	// 语法：数据类型  变量名 = 初始值
	int a = 2;

	// 3 常量：const修饰的变量
	const double e = 2.7182818284;

	// 4 输入：cin 输入时，忽略空格和换行符
	cout << "请输入一个整型数字：a=";
	cin >> a;
	cout << "第一个运算：pi*a+e=" << PI * a + e << endl;

	return 0;
}