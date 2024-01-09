#include <iostream>
#include <vector>
#include <cmath>

using namespace std;
/**
链表是一种逻辑上连续，内存上分散的线性表数据结构，其基本单位为结点，每个结点分数据区和指针区，
数据区用于存放数据，指针区则用于指向其他结点，通过指针每个结点就被串接成了一条“链子”。
    链表分类：单链表、双链表、循环链表
    链表的存储：散乱分布，通过指针域的指针链接在一起
    链表的操作：删除节点、增加节点
 */
// 1 单向链表节点
template <class T>
class LinkListNode
{
public:
    // 数据域
    T val;
    // 指针域
    LinkListNode *next;

    // 构造函数
    LinkListNode() : val(0), next(nullptr) {}
    LinkListNode(T x) : val(x), next(nullptr) {}
    LinkListNode(T x, LinkListNode *next) : val(x), next(next) {}
};

// 单向链表类
template <class T>
class MyLinkList
{
public:
    // 头指针
    LinkListNode<T> *head;
    // 尾指针
    LinkListNode<T> *tail;
    // 链表长度
    int length;

    // 2 创建链表
    MyLinkList(vector<T> arr)
    {
        if (arr.size() == 0)
            head = nullptr;

        // 创建头指针
        head = new LinkListNode<T>(arr[0]);
        LinkListNode<T> *current = head;

        for (int i = 1; i < (int)arr.size(); i++)
        {
            // 创建下一个节点指针
            current->next = new LinkListNode<T>(arr[i]);
            current = current->next;
        }
        length = arr.size();
        tail = current;
    }

    // 3 按值删除节点
    void deleteElements(T val)
    {
        // 删除头节点
        while ((head != nullptr) && (head->val == val))
        { // 不是空链表且头节点的值与目标值相等
            LinkListNode<T> *temp = head;
            head = head->next;
            delete temp;
            length--;
        }
        // 删除非头节点
        LinkListNode<T> *cur = head; // 头节点不动，
        while ((cur != nullptr) && (cur->next != nullptr))
        { // 不是空链表且不是尾部节点
            if (cur->next->val == val)
            {                                      // 如果下一个节点的值等于目标值，跳过，且删除
                LinkListNode<T> *temp = cur->next; //
                cur->next = cur->next->next;
                delete temp;
                length--;
            }
            else
            {
                cur = cur->next;
            }
        }
    }

    // 4 获取到第index个节点数值，如果index是非法数值直接返回-1，
    // 注意index是从0开始的，第0个节点就是头结点：index: 0 1 2 3 ...length-1
    T get(int index)
    {
        if (index > length - 1 || index < 0)
        {
            cout << "index is illegal!" << endl;
            return -1;
        }
        LinkListNode<T> *cur = head;
        while (index--) // 如果--index 就会陷入死循环
        {
            cur = cur->next;
        }
        return cur->val;
    }

    // 5 在链表最前面插入一个节点，插入完成后，新插入的节点为链表的新的头结点
    void addAtHead(T val)
    {
        LinkListNode<T> *newNode = new LinkListNode<T>(val);
        newNode->next = head;
        head = newNode;
        length++;
    }

    // 6 在链表最后面添加一个节点
    void addAtTail(T val)
    {
        LinkListNode<T> *newNode = new LinkListNode<T>(val);
        LinkListNode<T> *cur = head;
        while (cur->next != nullptr)
        {
            cur = cur->next;
        }
        cur->next = newNode;
        length++;
    }

    // 7 在第index个节点之后插入一个新节点，例如index为0，那么新插入的节点为链表的新头节点。
    // 如果index 等于链表的长度，则说明是新插入的节点为链表的尾结点
    // 如果index大于链表的长度，则返回空
    void addAtIndex(int index, T val)
    {
        if (index > length)
        {
            std::cout << "index is bigger than length" << std::endl;
            return;
        }
        else if (index == length)
        {
            addAtTail(val);
            return;
        }
        else if (index == 0)
        {
            addAtHead(val);
            return;
        }
        else if (index < 0)
        {
            addAtIndex(index + length, val);
            return;
        }
        LinkListNode<T> *newNode = new LinkListNode<T>(val);
        LinkListNode<T> *cur = head;
        while (index--)
        {
            cur = cur->next;
        }
        newNode->next = cur->next;
        cur->next = newNode;
        length++;
    }

    // 8 删除链表最前面节点
    void deleteAtHead()
    {
        LinkListNode<T> *temp = head;
        head = head->next;
        delete temp;
        length--;
    }

    // 9 删除第index个节点，注意index是从0开始的
    void deleteAtIndex(int index)
    {
        if (index > length - 1)
        {
            std::cout << "index is bigger than length" << std::endl;
            return;
        }
        else if (index == 0)
        {
            deleteAtHead();
            return;
        }
        else if (index < 0)
        {
            deleteAtIndex(index + length);
            return;
        }

        LinkListNode<T> *cur = head;
        while (--index) // index>0不会陷入死循环
        {
            cur = cur->next;
        }
        LinkListNode<T> *tmp = cur->next;
        cur->next = cur->next->next;
        delete tmp;
        length--;
    }

    // 10 反转链表
    void reverseList()
    {
        // 定义一个中间链表节点temp，接收cur节点。cur做翻转节点，将next指向pre。
        LinkListNode<T> *pre = nullptr; // 初始化一个 null指针的节点
        LinkListNode<T> *cur = head;
        LinkListNode<T> *temp;
        while (cur != nullptr)
        {
            temp = cur->next;
            cur->next = pre;
            pre = cur;
            cur = temp;
        }
        head = pre;
    }

    // 11 打印链表
    void printLinkListedLinkList()
    {
        std::cout << "LinkList length:" << length << std::endl;
        LinkListNode<T> *current = head;
        while (current != nullptr)
        {
            std::cout << current->val << " ";
            // 移动到下一个节点
            current = current->next;
        }
        std::cout << std::endl;
    }

    // 析构函数手动释放每一个节点
    ~MyLinkList()
    {
        LinkListNode<T> *current = head;
        while (current != nullptr)
        {
            LinkListNode<T> *temp = current;
            current = current->next;
            delete temp;
        }
    }
};

int main()
{
    vector<int> arr = {5, 6, 4, 2, 4, 3};
    // 创建链表
    MyLinkList<int> L = MyLinkList<int>(arr);
    L.printLinkListedLinkList();
    L.deleteElements(4);
    L.printLinkListedLinkList();
    L.addAtHead(1);
    L.printLinkListedLinkList();
    L.addAtTail(8);
    L.printLinkListedLinkList();
    L.addAtIndex(2, 7);
    L.printLinkListedLinkList();
    L.addAtIndex(L.length, 100);
    L.printLinkListedLinkList();
    L.addAtIndex(-1, 99);
    L.printLinkListedLinkList();
    L.addAtIndex(-5, 98);
    L.printLinkListedLinkList();
    L.deleteAtIndex(6);
    L.printLinkListedLinkList();
    L.deleteAtIndex(-5);
    L.printLinkListedLinkList();
    L.reverseList();
    L.printLinkListedLinkList();
    std::cout << "LinkList index 0:" << L.get(0) << std::endl;
    std::cout << "LinkList index 5:" << L.get(5) << std::endl;
    std::cout << "LinkList index " << L.length - 1 << ":" << L.get(L.length - 1) << std::endl;

    return 0;
}