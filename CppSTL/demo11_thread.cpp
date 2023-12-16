#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <list>
using namespace std;

class A
{
public:
    void inMsgRecvQueue()
    {
        for (int i = 0; i < 5; i++)
        {
            cout << "inMsgRecvQueue()执行，插入一个元素" << i << endl;
            {                                                      //花括号是锁的范围
                unique_lock<mutex> sbguard(my_mutex, try_to_lock); //尝试去锁，如果没有锁定成功，我也会立即返回，并不会阻塞在那里
                if (sbguard.owns_lock())
                {
                    //拿到了锁
                    msgRecvQueue.push_back(i);
                }
                else
                {
                    //没拿到锁
                    cout << "inMsgRecvQueue()执行，但没拿到锁头，只能干点别的事" << i << endl;
                }
            }
            this_thread::sleep_for(chrono::seconds(1)); //设置等待时间1s
        }
    }

    bool outMsgLULProc(int &command)
    {
        my_mutex.lock(); //要先lock(),后续才能用unique_lock的std::adopt_lock参数
        std::unique_lock<std::mutex> sbguard(my_mutex, std::adopt_lock);
        this_thread::sleep_for(chrono::milliseconds(2000)); //休息2s

        if (!msgRecvQueue.empty())
        {
            //消息不为空
            int command = msgRecvQueue.front(); //返回第一个元素，但不检查元素是否存在
            msgRecvQueue.pop_front();           //移除第一个元素。但不返回；

            return true;
        }
        return false;
    }

    //把数据从消息队列取出的线程
    void outMsgRecvQueue()
    {
        int command = 0;
        for (int i = 0; i < 5; i++)
        {
            bool result = outMsgLULProc(command);

            if (result == true)
            {
                cout << "outMsgRecvQueue()执行，取出一个元素" << endl;
            }
            else
            {
                //消息队列为空
                cout << "inMsgRecvQueue()执行，但目前消息队列中为空 " << i << endl;
            }
            this_thread::sleep_for(chrono::seconds(1)); //设置等待时间1s
        }
        cout << "end!" << endl;
    }

private:
    list<int> msgRecvQueue; //容器（消息队列）
    mutex my_mutex;         //创建一个互斥锁
};

void proc1(int a)
{
    cout << "我是子线程,传入参数为" << a << endl;
    cout << "子线程中显示子线程id为" << this_thread::get_id() << endl;
    for (int i = 1; i <= 5; i++)
        cout << "proc1:" << i << endl;
}

mutex m; //实例化m对象，不要理解为定义变量程序实例化mutex对象m,线程调用成员函数m.lock()会发生下面 3 种情况：(1)如果该互斥量当前未上锁，则调用线程将该互斥量锁住，直到调用unlock()之前，该线程一直拥有该锁。(2)如果该互斥量当前被锁住，则调用线程被阻塞,直至该互斥量被解锁。

void proc2(int a)
{
    m.lock();
    cout << "proc2拿到锁" << endl;
    for (int i = 0; i < 5; i++)
    {
        cout << "proc2:现在a为a+" << i << "=" << a + i << endl;
        this_thread::sleep_for(chrono::seconds(1)); //设置等待时间1s
    }
    m.unlock();
    for (int i = 1; i <= 5; i++)
    {
        cout << "proc2:" << i << endl;
        this_thread::sleep_for(chrono::seconds(1)); //设置等待时间
    }
}

void proc3(int &a)
{
    this_thread::sleep_for(chrono::seconds(1));
    {                               //花括号内为锁的范围
        unique_lock<mutex> lock(m); //会自动解锁
        cout << "proc3拿到锁" << endl;
        for (int i = 0; i < 5; i++)
        {
            cout << "proc3:现在a为a+" << i << "=" << a + i << endl;
            this_thread::sleep_for(chrono::seconds(1)); //设置等待时间1s
        }
        cout << "proc3用完锁" << endl;
    }
    for (int i = 1; i <= 10; i++)
    {
        cout << "proc3:" << i << endl;
        this_thread::sleep_for(chrono::seconds(1));
    }
}

/*整个过程就相当于你在做某件事情，中途你让老王帮你办一个任务（你办的时候他同时办）（创建线程1），又叫老李帮你办一件任务（创建线程2），现在你的这部分工作做完了，需要用到他们的结果，只需要等待老王和老李处理完（join()，阻塞主线程），等他们把任务做完（子线程运行结束），你又可以开始你手头的工作了（主线程不再阻塞）。*/

/*互斥锁这样比喻，单位上有一台打印机（共享数据a），你要用打印机（线程1要操作数据a），同事老王也要用打印机(线程2也要操作数据a)，但是打印机同一时间只能给一个人用，此时，规定不管是谁，在用打印机之前都要向领导申请许可证（lock），用完后再向领导归还许可证(unlock)，许可证总共只有一个,没有许可证的人就等着在用打印机的同事用完后才能申请许可证(阻塞，线程1lock互斥锁后其他线程就无法lock,只能等线程1unlock后，其他线程才能lock)，那么，这个许可证就是互斥锁。互斥锁保证了使用打印机这一过程不被打断。*/

int main()
{
    cout << "我是主线程" << endl;
    int a = 9;
    thread th1(proc1, a); //第一个参数为函数名，第二个参数为该函数的第一个参数，如果该函数接收多个参数就依次写在后面。只要创建了线程对象（传递“函数名/可调用对象”作为参数的情况下），线程就开始执行（与主线程同时运行）
    cout << "主线程id为:" << this_thread::get_id() << endl;
    cout << "主线程中显示子线程id为:" << th1.get_id() << endl;
    cout << "-----主线程等待线程1-----" << endl;
    th1.join(); //此时主线程被阻塞直至子线程执行结束。join(), 当前线程暂停, 等待指定的线程执行结束后, 当前线程再继续。
    cout << "-----th1结束，回到主线程-----" << endl;
    cout << "3个线程同时运行：" << endl;
    thread th2(proc2, a);
    thread th3(proc3, ref(a)); //传引用参数时要std::ref()映射
    for (int i = 1; i <= 15; i++)
    {
        cout << "main:" << i << endl;
        this_thread::sleep_for(chrono::seconds(1));
    }

    A myobja;
    thread myOutMsgObj(&A::outMsgRecvQueue, &myobja);
    thread myInMsgObj(&A::inMsgRecvQueue, &myobja);
    myOutMsgObj.join();
    myInMsgObj.join();
    cout << "主线程执行！" << endl;

    return 0;
}
