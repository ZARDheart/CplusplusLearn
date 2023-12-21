#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <list>
#include <condition_variable> // std::condition_variable

using namespace std;

/*
使用多进程并发是将一个应用程序划分为多个独立的进程（每个进程只有一个线程），这些独立的进程间可以互相通信，共同完成任务。例如ROS节点
由于操作系统对进程提供了大量的保护机制，以避免一个进程修改了另一个进程的数据，使用多进程比多线程更容易写出安全的代码。

多线程并发指的是在同一个进程中执行多个线程。同一进程中的多个线程共享相同的地址空间，可以访问进程中的大部分数据，指针和引用可以在线程间进行传递。
由于缺少操作系统提供的保护机制，在多线程共享数据及通信时，就需要程序员做更多的工作以保证对共享数据段的操作是以预想的操作顺序进行的，并且要极力的避免死锁(deadlock)。

传统的C++（C++11之前）中并没有引入线程这个概念，在C++11出来之前，
如果我们想要在C++中实现多线程，需要借助操作系统平台提供的API，比如Linux的<pthread.h>，或者windows下的<windows.h>
C++11提供了语言层面上的多线程，包含在头文件<thread>中。
它解决了跨平台的问题，提供了管理线程、保护共享数据、线程间同步操作、原子操作等类。
C++11 新标准中引入了5个头文件来支持多线程编程
*/

void proc1(int a)
{
    cout << "我是子线程,传入参数为" << a << endl;
    cout << "子线程中显示子线程id为" << this_thread::get_id() << endl;
    for (int i = 1; i <= 5; i++)
    {
        cout << "proc1:" << i << endl;
        // 设置沉睡时间
        this_thread::sleep_for(chrono::milliseconds(200)); // 200ms
    }
}

mutex m; // 实例化mutex对象，线程调用成员函数m.lock()会发生下面 3 种情况：
//(1)如果该互斥量当前未上锁，则调用线程将该互斥量锁住，直到调用unlock()之前，该线程一直拥有该锁。
//(2)如果该互斥量当前被锁住，则调用线程被阻塞,直至该互斥量被解锁。
// 同一个（注意：不同的锁并不会相互影响）mutex变量上锁之后，一个时间段内，只允许一个线程访问它，
// 因此其他线程会被阻塞(当然其范围内的变量使用就不会冲突)
string str = "*";

/*互斥锁这样比喻，单位上有一台打印机（共享数据a），你要用打印机（线程1要操作数据a），
同事老王也要用打印机(线程2也要操作数据a)，但是打印机同一时间只能给一个人用，
此时，规定不管是谁，在用打印机之前都要向领导申请许可证（lock），用完后再向领导归还许可证(unlock)，
许可证总共只有一个,没有许可证的人就等着在用打印机的同事用完后才能申请许可证
(阻塞，线程1lock互斥锁后其他线程就无法lock,只能等线程1unlock后，其他线程才能lock)，
那么，这个许可证就是互斥锁。互斥锁保证了使用打印机这一过程不被打断。*/

// lock_guard
//     创建即加锁，作用域结束自动析构并解锁，无需手工解锁
//     不能中途解锁，必须等作用域结束才解锁
//     不能复制

// nique_lock 是 lock_guard 的升级加强版，它具有 lock_guard 的所有功能，同时又具有其他很多方法，
// 使用起来更强灵活方便，能够应对更复杂的锁定需要。unique_lock的特点：
//     创建时可以不锁定（通过指定第二个参数为std::defer_lock），而在需要时再锁定
//     可以随时加锁解锁
//     作用域规则同 lock_grard，析构时自动释放锁
//     不可复制，可移动
// unique_lock的第二个参数，除了可以是adopt_lock,还可以是try_to_lock与defer_lock;
// try_to_lock: 尝试去锁定，得保证锁处于unlock的状态,然后尝试现在能不能获得锁；尝试用mutx的lock()去锁定这个mutex，
//      但如果没有锁定成功，会立即返回，不会阻塞在那里
// defer_lock: 始化了一个没有加锁的mutex;

// 死锁：是指两个或两个以上的进程在执行过程中，由于竞争资源或者由于彼此通信而造成的一种阻塞的现象，若无外力作用，它们都将无法推进下去。
//     此时称系统处于死锁状态或系统产生了死锁，这些永远在互相等待的进程称为死锁进程。

void proc2(int a)
{
    m.lock();
    cout << "proc2拿到锁" << endl;
    for (int i = 0; i < 10; i++)
    {
        cout << "proc2" << str;
        this_thread::sleep_for(chrono::milliseconds(200)); // 200ms
    }
    cout << "proc2用完锁" << endl;
    m.unlock();
    for (int i = 1; i <= 10; i++)
    {
        cout << "proc2:" << i << endl;
        this_thread::sleep_for(chrono::milliseconds(200)); // 200ms
    }
}

void proc3(int &a)
{
    this_thread::sleep_for(chrono::seconds(1));
    {                               // 花括号内为锁的范围
        unique_lock<mutex> lock(m); // 会自动解锁
        cout << "proc3拿到锁" << endl;
        for (int i = 0; i < 10; i++)
        {
            cout << "proc3" << str;
            this_thread::sleep_for(chrono::milliseconds(200)); // 200ms
        }
        cout << "proc3用完锁" << endl;
    }
    for (int i = 1; i <= 10; i++)
    {
        cout << "proc3:" << i << endl;
        this_thread::sleep_for(chrono::milliseconds(200)); // 200ms
    }
}

class A
{
public:
    void inMsgRecvQueue()
    {
        for (int i = 0; i < 20; i++)
        {
            cout << "inMsgRecvQueue()执行，";
            {                                                      // 花括号是锁的范围
                unique_lock<mutex> sbguard(my_mutex, try_to_lock); // 尝试去锁，如果没有锁定成功，我也会立即返回，并不会阻塞在那里
                if (sbguard.owns_lock())
                {
                    // 拿到了锁
                    msgRecvQueue.push_back(i);
                    cout << "拿到锁，插入一个元素" << i << endl;
                }
                else
                {
                    // 没拿到锁
                    cout << "但没拿到锁头，只能干点别的事" << i << endl;
                }
            }
            this_thread::sleep_for(chrono::seconds(1)); // 设置等待时间1s
        }
    }

    // 把数据从消息队列取出的线程
    void outMsgRecvQueue()
    {
        int command = 0;
        for (int i = 0; i < 5; i++)
        {
            bool result = false;
            {
                my_mutex.lock(); // 要先lock(),后续才能用unique_lock的std::adopt_lock参数
                std::unique_lock<std::mutex> sbguard(my_mutex, std::adopt_lock);
                // 取出的时候锁住，将无法插入
                this_thread::sleep_for(chrono::milliseconds(2000)); // 休息2s

                if (!msgRecvQueue.empty())
                {
                    // 消息不为空
                    int command = msgRecvQueue.front(); // 返回第一个元素，但不检查元素是否存在
                    msgRecvQueue.pop_front();           // 移除第一个元素

                    result = true;
                }
            }
            if (result == true)
            {
                cout << "outMsgRecvQueue()执行，取出一个元素" << endl;
            }
            else
            {
                // 消息队列为空
                cout << "outMsgRecvQueue()执行，但目前消息队列中为空 " << i << endl;
            }
            this_thread::sleep_for(chrono::seconds(1)); // 设置等待时间1s
        }
        cout << "outMsgRecvQueue end!" << endl;
    }
    void printList()
    {
        // 打印看看列表中都有什么
        list<int>::iterator it;
        for (it = msgRecvQueue.begin(); it != msgRecvQueue.end(); it++)
        {
            cout << *it << " ";
        }
        cout << endl;
    }

private:
    list<int> msgRecvQueue; // 容器（消息队列）
    mutex my_mutex;         // 创建一个互斥锁
};

std::condition_variable cv;
int cargo = 0;
bool shipment_available() { return cargo != 0; }

void consume(int n)
{
    for (int i = 0; i < n; ++i)
    {
        std::unique_lock<std::mutex> lck(m); // 自动上锁
        // 第二个参数为false才阻塞（wait），阻塞完即unlock，给其它线程资源
        cv.wait(lck, shipment_available);
        // consume:
        std::cout << cargo << '\n';
        cargo = 0;
    }
}

// 因为程序边运行边创建线程是比较耗时的，所以我们通过池化的思想：
// 在程序开始运行前创建多个线程，这样，程序在运行时，只需要从线程池中拿来用就可以了．大大提高了程序运行效率．
// 一般线程池都会有以下几个部分构成：
//     线程池管理器（ThreadPoolManager）:用于创建并管理线程池，也就是线程池类
//     工作线程（WorkThread）: 线程池中线程
//     任务队列task: 用于存放没有处理的任务。提供一种缓冲机制。
//     append：用于添加任务的接口

int main()
{
    cout << "我是主线程" << endl;
    int a = 9;

    // 1 创建线程只需要把函数添加到线程当中即可。同一个函数可以代码复用，创建多个线程
    // 只要创建了线程对象（传递“函数名/可调用对象”作为参数的情况下），线程就开始执行（与主线程同时运行）
    //     形式1：
    //         std::thread myThread ( thread_fun); //函数形式为void thread_fun()
    //         myThread.join();
    //     形式2：
    //         std::thread myThread(thread_fun(100)); //函数形式为void thread_fun(int x)
    //         myThread.join();
    //     形式3：
    //         std::thread (thread_fun,1).detach();//直接创建线程，没有名字
    // 当线程启动后，一定要在和线程相关联的thread销毁前，确定以何种方式等待线程执行结束。比如上例中的join。
    //     detach方式，启动的线程自主在后台运行，当前的代码继续往下执行，不等待新线程结束。
    //     join方式，当前线程暂停, 等待指定的线程执行结束后, 当前线程再继续。
    thread th1(proc1, a); // 如果该函数接收多个参数就依次写在后面。
    // this_thread是一个类，它有4个功能函数，具体如下：
    //     函数	使用	说明
    //     get_id	std::this_thread::get_id()	获取线程id
    //     yield	std::this_thread::yield()	放弃线程执行，回到就绪状态
    //     sleep_for	std::this_thread::sleep_for(std::chrono::seconds(1));	暂停1秒
    //     sleep_until	如下	一分钟后执行吗，如下
    cout << "主线程id为:" << this_thread::get_id() << endl;
    cout << "主线程中显示子线程id为:" << th1.get_id() << endl;
    cout << "-----主线程等待线程1-----" << endl;
    th1.join(); // 此时主线程被阻塞直至子线程执行结束
    // 可以使用joinable判断是join模式还是detach模式。 th1.joinable()
    cout << "-----th1结束，回到主线程-----" << endl;

    // 2 线程锁
    cout << "3个线程同时运行：" << endl;
    thread th2(proc2, a);
    thread th3(proc3, ref(a)); // 传引用参数时要std::ref()映射 std::cref 用于包装按const引用传递的值。
    for (int i = 1; i <= 10; i++)
    {
        cout << "main:" << i << endl;
        this_thread::sleep_for(chrono::seconds(1));
    }
    th2.join();
    th3.join();

    // 3 成员函数做线程函数
    A myobja;
    thread myOutMsgObj(&A::outMsgRecvQueue, &myobja);
    thread myInMsgObj(&A::inMsgRecvQueue, &myobja);
    myOutMsgObj.join();
    myInMsgObj.join();
    cout << "主线程执行！" << endl;
    myobja.printList();

    // 4
    std::thread consumer_thread(consume, 10);
    for (int i = 0; i < 10; ++i)
    {
        // 每次cargo每次为0才运行。
        while (shipment_available())
            std::this_thread::yield();
        std::unique_lock<std::mutex> lck(m);
        cargo = i + 1;
        cv.notify_one();
    }

    consumer_thread.join();

    return 0;
}
