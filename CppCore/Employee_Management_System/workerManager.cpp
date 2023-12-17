#include "workerManager.h"
using namespace std;

WorkerManager::WorkerManager()
{
    ifstream ifs;
    ifs.open(FILENAME, ios::in);
    // 1 文件不存在情况
    if (!ifs.is_open())
    {
        // cout << "文件不存在" << endl;
        this->m_EmpNum = 0;
        this->m_FileIsEmpty = true;
        this->m_EmpArray = NULL;
        ifs.close();
        return;
    }
    // 2 文件存在，并且没有记录
    char ch;
    ifs >> ch;
    if (ifs.eof())
    {
        // cout << "文件为空!" << endl;
        this->m_EmpNum = 0;
        this->m_FileIsEmpty = true;
        this->m_EmpArray = NULL;
        ifs.close();
        return;
    }
    // 3 文件中有记录，导入到成员变量
    this->Load();
}

void WorkerManager::Load()
{
    // 1 获取职工个数
    string name;
    int id, dId, num = 0;
    // 光标移动到文件首
    ifstream ifs;
    ifs.open(FILENAME, ios::in);
    while (ifs >> id && ifs >> name && ifs >> dId) // 往后移
    {
        num++;
    }
    cout << "当前职工个数为：" << num << endl;
    this->m_EmpNum = num;
    ifs.close();

    // 2 开辟空间，导入数据
    this->m_EmpArray = new Worker *[num];
    ifs.open(FILENAME, ios::in);
    int index = 0;
    while (ifs >> id && ifs >> name && ifs >> dId) // 往后移
    {
        Worker *worker = NULL;
        if (dId == 1) // 总裁
        {
            worker = new Boss(id, name, dId);
        }
        else if (dId == 2) // 2经理
        {
            worker = new Manager(id, name, dId);
        }
        else // 普通员工
        {
            worker = new Employee(id, name, dId);
        }
        m_EmpArray[index] = worker;
        index++;
        // cout << "姓名： " << worker->m_Name << " 部门编号： " << worker->m_DeptId << endl;
    }
    ifs.close();
    this->m_FileIsEmpty = false;
}

void WorkerManager::Save()
{
    // 将所有员工写入磁盘
    ofstream ofs;
    ofs.open(FILENAME, ios::out);
    for (int i = 0; i < this->m_EmpNum; i++)
    {
        ofs << this->m_EmpArray[i]->m_Id << " " << this->m_EmpArray[i]->m_Name << " " << this->m_EmpArray[i]->m_DeptId << endl;
    }

    ofs.close();
}

void WorkerManager::Show_Menu()
{
    cout << "**************************************************" << endl;
    cout << "*********  Welcome to the Worker Manager *********" << endl;
    cout << "*********  Please Select an Option       *********" << endl;
    cout << "*********  0. Exit                       *********" << endl;
    cout << "*********  1. Show all workers           *********" << endl;
    cout << "*********  2. Add a new worker           *********" << endl;
    cout << "*********  3. Remove a worker            *********" << endl;
    cout << "*********  4. Update a worker            *********" << endl;
    cout << "*********  5. Search a worker            *********" << endl;
    cout << "*********  6. Sort by ID                 *********" << endl;
    cout << "*********  7. Clear all data             *********" << endl;
    cout << "**************************************************" << endl;
}

void WorkerManager::ExitSystem()
{
    cout << "Goodbye!" << endl;
    exit(EXIT_FAILURE); // 终止程序
}

void WorkerManager::Show_Worker()
{
    if (this->m_FileIsEmpty)
    {
        cout << "文件不存在或记录为空！" << endl;
        return;
    }

    for (int i = 0; i < m_EmpNum; i++)
    {
        cout << "职工编号： " << this->m_EmpArray[i]->m_Id << " \t职工姓名： " << this->m_EmpArray[i]->m_Name << " \t岗位：" << this->m_EmpArray[i]->getDeptName() << endl;
    }
}

// 判断职工是否存在  如果存在返回职工所在数组中的位置，不存在返回-1
int WorkerManager::IsExist(int id)
{
    int index = -1;
    for (int i = 0; i < this->m_EmpNum; i++)
    {
        if (this->m_EmpArray[i]->m_Id == id)
        {
            // 找到职工
            index = i;

            break;
        }
    }
    return index;
}

void WorkerManager::Add_Worker()
{
    cout << "请输入增加职工数量： " << endl;
    int addNum = 0;
    cin >> addNum;
    if (addNum > 0)
    {
        // 计算新空间大小
        int newSize = this->m_EmpNum + addNum;

        // 开辟新空间
        Worker **newSpace = new Worker *[newSize];

        // 将原空间下内容存放到新空间下
        if (this->m_EmpArray != NULL)
        {
            for (int i = 0; i < this->m_EmpNum; i++)
            {
                newSpace[i] = this->m_EmpArray[i];
            }
        }

        // 输入新数据
        for (int i = 0; i < addNum; i++)
        {
            int id;
            string name;
            int dSelect;

            cout << "请输入第" << i + 1 << "个新职工编号：" << endl;
            cin >> id;
            cin.get();

            if (IsExist(id) != -1)
            {
                cout << "此职工编号已经存在，请重新输入第" << i + 1 << "个新职工信息！" << endl;
                i--;
                continue;
            }

            cout << "请输入第" << i + 1 << "个新职工姓名：" << endl;
            cin >> name;
            cin.get();

            cout << "请选择该职工的岗位：" << endl;
            cout << "1、老板" << endl;
            cout << "2、经理" << endl;
            cout << "3、普通职工" << endl;
            cin >> dSelect;
            cin.get();

            Worker *worker = NULL;
            switch (dSelect)
            {
            case 1: // 老板
                worker = new Boss(id, name, dSelect);
                break;
            case 2: // 经理
                worker = new Manager(id, name, dSelect);
                break;
            case 3: // 普通员工
                worker = new Employee(id, name, dSelect);
                break;
            default:
                cout << "输入职位错误，请重新输入第" << i + 1 << "个新职工信息！" << endl;
                i--;
                continue;
            }

            newSpace[this->m_EmpNum + i] = worker;
        }

        // 释放原有空间
        delete[] this->m_EmpArray;
        // 更改新空间的指向
        this->m_EmpArray = newSpace;
        // 更新新的个数
        this->m_EmpNum = newSize;

        // 更新到文件中
        this->Save();
        // 提示信息
        cout << "成功添加" << addNum << "名新职工！" << endl;

        // 文件当前不为空
        this->m_FileIsEmpty = false;
    }
    else
    {
        cout << "输入有误" << endl;
    }
}

void WorkerManager::Del_Worker()
{
    if (this->m_FileIsEmpty)
    {
        cout << "文件不存在或记录为空！" << endl;
    }
    else
    {
        // 按照职工编号删除
        cout << "请输入想要删除职工编号：" << endl;
        int id = 0;
        cin >> id;

        int index = this->IsExist(id);

        if (index != -1) // 说明职工存在，并且要删除掉index位置上的职工
        {

            for (int i = index; i < this->m_EmpNum - 1; i++)
            {
                // 数据前移
                this->m_EmpArray[i] = this->m_EmpArray[i + 1];
            }
            this->m_EmpNum--; // 更新数组中记录人员个数
            // 数据同步更新到文件中
            this->Save();

            cout << "删除成功！" << endl;
        }
        else
        {
            cout << "删除失败，未找到该职工！" << endl;
        }
    }
}

void WorkerManager::Update_Worker()
{
    if (this->m_FileIsEmpty)
    {
        cout << "文件不存在或记录为空！" << endl;
    }
    else
    {
        // 按照职工编号修改
        cout << "请输入想要修改职工编号：" << endl;
        int id = 0;
        cin >> id;
        int index = this->IsExist(id);
        if (index != -1) // 说明职工存在，并且要删除掉index位置上的职工
        {
            int newId = 0, dSelect = 0;
            string newName = "";

            cout << "查到" << id << "号职工，请输入新职工号： " << endl;
            cin >> newId;

            cout << "请输入新姓名： " << endl;
            cin >> newName;

            cout << "请输入岗位： " << endl;
            cout << "1、老板" << endl;
            cout << "2、经理" << endl;
            cout << "3、普通职工" << endl;
            cin >> dSelect;

            Worker *worker = NULL;
            switch (dSelect)
            {
            case1:
                worker = new Employee(newId, newName, dSelect);
                break;
            case2:
                worker = new Manager(newId, newName, dSelect);
                break;
            case 3:
                worker = new Boss(newId, newName, dSelect);
                break;
            default:
                cout << "输入职位错误，请重新修改第" << id << "号职工信息！" << endl;
                return;
            }

            // 更改数据 到数组中
            delete this->m_EmpArray[index]; // 堆区数据不要了必须释放
            this->m_EmpArray[index] = worker;

            // 数据同步更新到文件中
            this->Save();

            cout << "修改成功！" << endl;
        }
        else
        {
            cout << "修改失败，未找到该职工！" << endl;
        }
    }
}

void WorkerManager::Search_Worker()
{
    if (this->m_FileIsEmpty)
    {
        cout << "文件不存在或记录为空！" << endl;
    }
    else
    {
        cout << "请输入查找的方式：" << endl;
        cout << "1、按职工编号查找 " << endl;
        cout << "2、按职工姓名查找 " << endl;
        int select = 0;
        cin >> select;

        if (select == 1)
        {
            // 按照编号查
            int id;
            cout << "请输入查找的职工编号： " << endl;
            cin >> id;

            int ret = IsExist(id);
            if (ret != -1)
            {
                // 找到职工
                cout << "查找成功！该职工信息如下：" << endl;
                this->m_EmpArray[ret]->showInfo();
            }
            else
            {
                cout << "查找失败，查无此人" << endl;
            }
        }
        else if (select == 2)
        {
            // 按照姓名查
            string name;
            cout << "请输入查找的姓名：" << endl;
            cin >> name;

            // 加入判断是否查到的标志
            bool flag = false; // 默认未找到职工

            for (int i = 0; i < m_EmpNum; i++)
            {
                if (this->m_EmpArray[i]->m_Name == name)
                {
                    cout << "查找成功，职工编号为： "
                         << this->m_EmpArray[i]->m_Id
                         << "号职工信息如下：" << endl;

                    flag = true;

                    // 调用显示信息接口
                    this->m_EmpArray[i]->showInfo();
                }
            }
            if (flag == false)
            {
                cout << "查找失败，查无此人！" << endl;
            }
        }
        else
        {
            cout << "输入选项有误！" << endl;
        }
    }
}

void WorkerManager::Sort_Worker()
{
    if (this->m_FileIsEmpty)
    {
        cout << "文件不存在或记录为空！" << endl;
    }
    else
    {
        cout << "请选择排序方式：" << endl;
        cout << "1、按职工号进行升序" << endl;
        cout << "2、按职工号进行降序" << endl;

        int select = 0;
        cin >> select;
        for (int i = 0; i < m_EmpNum; i++)
        {
            int minOrMax = i; // 声明最小值 或 最大值下标
            for (int j = i + 1; j < this->m_EmpNum; j++)
            {
                if (select == 1) // 升序
                {
                    if (this->m_EmpArray[minOrMax]->m_Id > this->m_EmpArray[j]->m_Id)
                    {
                        minOrMax = j;
                    }
                }
                else // 降序
                {
                    if (this->m_EmpArray[minOrMax]->m_Id < this->m_EmpArray[j]->m_Id)
                    {
                        minOrMax = j;
                    }
                }
            }

            // 判断一开始认定 最小值或最大值 是不是 计算的最小值或最大值，如果不是 交换数据
            if (i != minOrMax)
            {
                Worker *temp = this->m_EmpArray[i];
                this->m_EmpArray[i] = this->m_EmpArray[minOrMax];
                this->m_EmpArray[minOrMax] = temp;
            }
        }

        cout << "排序成功！排序后的结果为： " << endl;
        this->Save();        // 排序后结果保存到文件中
        this->Show_Worker(); // 展示所有职工
    }
}

void WorkerManager::Clear_File()
{
    cout << "确定清空？" << endl;
    cout << "1、确定" << endl;
    cout << "2、返回" << endl;

    int select = 0;
    cin >> select;

    if (select == 1)
    {
        // 清空文件
        ofstream ofs(FILENAME, ios::trunc); // 删除文件后重新创建
        ofs.close();
        // 清空成员数据
        if (this->m_EmpArray != NULL)
        {
            // 删除堆区的每个职工对象
            for (int i = 0; i < this->m_EmpNum; i++)
            {
                delete this->m_EmpArray[i];
                this->m_EmpArray[i] = NULL;
            }

            // 删除堆区数组指针
            delete[] this->m_EmpArray;
            this->m_EmpArray = NULL;
            this->m_EmpNum = 0;
            this->m_FileIsEmpty = true;
        }

        cout << "清空成功！" << endl;
    }
}

WorkerManager::~WorkerManager()
{
    if (this->m_EmpArray != NULL)
    { 
        // 删除堆区的每个职工对象
        for (int i = 0; i < this->m_EmpNum; i++)
        {
            delete this->m_EmpArray[i];
        }

        // 删除堆区数组指针
        delete[] this->m_EmpArray;
    }
}
