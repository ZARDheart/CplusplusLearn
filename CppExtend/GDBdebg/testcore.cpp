#include <iostream>
#include <unistd.h>

#include "testcore.h"

void test()
{
    while (1)
    {
        char *p = new char();
        delete p;
        p = nullptr;
        // 写一个段错误的例子，使用空指针
        *p = '2';
        sleep(1);
    }
}
