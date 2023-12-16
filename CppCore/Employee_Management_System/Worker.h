#ifndef WORKER_H_
#define WORKER_H_
#include <iostream>
#include <cstring>
using namespace std;

/*
用于创建不同的职工类
*/

// 1 职工抽象类
class Worker
{
public:
    // 显示个人信息
    virtual void showInfo() = 0;

    // 获取岗位名称
    virtual string getDeptName() = 0;

    // 职工编号
    int m_Id;
    // 职工姓名
    string m_Name;
    // 部门编号
    int m_DeptId;
};

// 1.1 老板类
class Boss : public Worker
{
public:
    // 构造函数
    Boss(int id, string name, int dId);

    // 显示个人信息
    virtual void showInfo();

    // 获取岗位名称
    virtual string getDeptName();
};

// 1.2 经理类
class Manager : public Worker
{
public:
    // 构造函数
    Manager(int id, string name, int dId);

    // 显示个人信息
    virtual void showInfo();

    // 获取岗位名称
    virtual string getDeptName();
};

// 1.3 普通员工
class Employee : public Worker
{
public:
    // 构造函数
    Employee(int id, string name, int dId);

    // 显示个人信息
    virtual void showInfo();

    // 获取岗位名称
    virtual string getDeptName();
};

// 1.4 多态方便拓展其他职工

#endif