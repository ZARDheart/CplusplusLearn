// 文件注释消息 --------------单行注释
/**
 * @file demo1_data_calu.cpp
 * @author WangBin
 * @brief C++学习demo
 * @version 0.1
 * @date 2021-12-06
 * https://namic00.github.io/
 * @copyright Copyright (c) 2021
 * -------------------多行注释
 */

#include <iostream>

// 命名空间：大型应用程序经常使用来自不同厂商的开发库，几乎不可避免会使用相同的名字，
// 也就是说一个库中定义的名字可能与其他库中的名字相同而产生冲突，使得程序员不能组合各自独立的开发库到一个程序中
// 命名空间就是用来限定名字的解析和使用范围的，原理是将全局作用域划分为一个一个的命名空间，
// 每个命名空间是一个独立的作用域，在不同命名空间内部定义的名字彼此之间互不影响，从而有效的避免了命名空间污染。
// 用法：
// namespace 命名空间名
// {
//      作用域
// }
using namespace std;

/*
这是一个第一个C++学习代码, 主要是初试以及注释的使用：
*/

/*注释技巧：
（1）变量前一行加三斜杠“///”注释，作为变量的查看说明（其他地方调用时可查）;
（2）变量同一行后加“///<”注释，效果同上;
（3）函数或类前加：见开头以及主函数上方内容，效果与变量说明类似;
（4） 一些特殊标记：*/
// fixme: 标识处代码需要修正，甚至代码是错误的，不能工作，需要修复，如何修正会在说明中简略说明 // FIXME
// bug: 代码存在已知的错误, 现在的代码中没有错误能运行, 但是由于用户输入导致的错误 // BUG
// hack: 变通方法, 差强人意的解决方案，补锅踩雷填坑 /*HACK*/
// note: 笔记，特别是编写者的想法意图和灵感 // NOTE: 
// NOTICE: 强调值得注意的地方,
// xxx: 代码有问题或具有误导性, 需引起警惕 // XXX
// test: 测试 // TEST
// done: 已经解决了的todo bug fixme将其变为done // DONE
// Todo: 标识处有功能代码待编写，即将需要完成的任务或实现的功能 // TODO
// ZARD：用户（我）自定义的标签

// Test
/** @brief 函数或类的说明
 *
 * @param[in]  pi1 输入参数说明
 * @param[in]  pi2 输入参数说明
 * @param[in]  argc 终端输入参数数量（包括可执行文件）
 * @param[in]  argv 终端输入参数字符串数组地址
 * ...
 * @param[out] po1 输出参数说明
 * ...
 *
 * @return int 返回值说明   */
int main(int argc, char **argv)
{
    std::cout << "Hello world!" << std::endl;
    return 0;
}
