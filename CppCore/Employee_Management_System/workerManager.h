#ifndef WORKMANAGER_H_
#define WORKMANAGER_H_
// 避免多次引用此头文件
#include <iostream>
#include <cstring>
#include <fstream>
#include <cstdlib>
#include "Worker.h"
using namespace std;

/*
基于多态的企业职工管理系统
1、管理系统需求
    职工管理系统可以用来管理公司内所有员工的信息
    本教程主要利用C++来实现一个基于多态的职工管理系统
公司中职工分为三类：普通员工、经理、老板，显示信息时，需要显示职工编号、职工姓名、职工岗位、以及职责
    普通员工职责：完成经理交给的任务
    经理职责：完成老板交给的任务，并下发任务给员工
    老板职责：管理公司所有事务
2、管理系统中需要实现的功能如下：
    * 退出管理程序：退出当前管理系统
    - 增加职工信息：实现批量添加职工功能，将信息录入到文件中，职工信息为：职工编号、姓名、部门编号
    - 显示职工信息：显示公司内部所有职工的信息
    - 删除离职职工：按照编号删除指定的职工
    - 修改职工信息：按照编号修改职工个人信息
    - 查找职工信息：按照职工的编号或者职工的姓名进行查找相关的人员信息
    - 按照编号排序：按照职工编号，进行排序，排序规则由用户指定
    - 清空所有文档：清空文件中记录的所有职工信息 （清空前需要再次确认，防止误删）
需根据用户不同的选择，完成不同的功能！

本案例设计到的知识点为：面向对象中的封装、继承、多态以及文件IO流，
在设计中有Worker职工的基类，以及分别派生类为普通员工、经理、以及总裁，基类中有纯虚函数子类分别作了实现。
然后有个文件管理类，对用户做出不同的选择分别做不同的处理。可以对系统进行基本的增删改查功能。
*/
#define FILENAME "../config/empFile.txt"

class WorkerManager
{
public:
    // 构造函数
    WorkerManager();
    // 磁盘数据IO
    void Save();
    void Load();

    // 展示菜单
    void Show_Menu();
    // 0 退出系统
    void ExitSystem();
    // 1 显示所有员工的信息
    void Show_Worker();
    // 2 添加一个员工
    int IsExist(int id);
    void Add_Worker();
    // 3 删除一个员工
    void Del_Worker();
    // 4 修改某个编号职工的信息
    void Update_Worker();
    void Search_Worker();
    void Sort_Worker();
    void Clear_File();

    // 析构函数
    ~WorkerManager();

public:
    // 记录职工人数
    int m_EmpNum;
    // 职工数组指针（父类指针指向不同的子类，体现出多态的优势）
    Worker **m_EmpArray;
    // 文件是否为空
    bool m_FileIsEmpty;
};

#endif
