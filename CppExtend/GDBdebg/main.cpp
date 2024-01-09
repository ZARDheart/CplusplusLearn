#include <iostream>
#include <unistd.h>

#include "testcore.h"

/*
gdb几个简单命令即可定位段错误的位置:
例如， 执行程序为 ./testcore  123, 以此例子为例，
1.linux命令行敲入            gdb ./testcore
2.如果有参数，则gdb继续敲入    set arg 123
3.运行，敲入                 r
4.敲入                      bt
即可看到在哪个文件哪个函数中发生段错误 
*/

int main(int argc,char **argv)
{
	std::cout<<argv[1]<<std::endl;
    test();
    
    return 0;
}
