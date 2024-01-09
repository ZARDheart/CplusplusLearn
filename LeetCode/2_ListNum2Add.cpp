#include <iostream>
#include <cmath>
/**
 * 两数相加
给你两个 非空 的链表，表示两个非负的整数。
它们每位数字都是按照 逆序 的方式存储的，并且每个节点只能存储 一位 数字。
请你将两个数相加，并以相同形式返回一个表示和的链表。
你可以假设除了数字 0 之外，这两个数都不会以 0 开头。
    输入：l1 = [2,4,3], l2 = [5,6,4]
    输出：[7,0,8]
    解释：342 + 465 = 807.
 */
// 1 一个链表结构体
struct ListNode
{
    // 数据域
    int val;
    // 指针域
    ListNode *next;

    // 构造函数
    ListNode() : val(0), next(nullptr) {}
    ListNode(int x) : val(x), next(nullptr) {}
    ListNode(int x, ListNode *next) : val(x), next(next) {}
};

// 2 创建链表
ListNode *createLinkedList(int arr[], int size)
{
    if (size == 0)
        return nullptr;

    // 创建头指针
    ListNode *head = new ListNode(arr[0]);
    ListNode *current = head;

    for (int i = 1; i < size; i++)
    {
        // 创建下一个节点指针
        current->next = new ListNode(arr[i]);
        current = current->next;
    }

    return head;
}

// 3 打印链表
void printLinkedList(ListNode *head)
{
    ListNode *current = head;
    while (current != nullptr)
    {
        std::cout << current->val << " ";
        // 移动到下一个节点
        current = current->next;
    }
    std::cout << std::endl;
}

// 两数相加 错误示例 还原成数字就是大错特错
class Solution1
{
public:
    ListNode *addTwoNumbers(ListNode *l1, ListNode *l2)
    {
        int64_t result = getSum(l1) + getSum(l2);
        ListNode *head = new ListNode(result % 10);
        result /= 10;
        ListNode *current = head;
        while (result > 0)
        {
            current->next = new ListNode(result % 10);
            result /= 10;
            current = current->next;
        }

        return head;
    }

    // 返回int数字超过2147483647时将报错，换成int64_t
    int64_t getSum(ListNode *head)
    {
        int64_t i = 0, sum = 0;
        // 遍历链表
        ListNode *current = head;
        while (current != nullptr)
        {
            sum += current->val * pow(10, i);
            // length++;
            i++;
            current = current->next;
        }

        return sum;
    }
    // 换成int64_t也是有上限的，采用其他办法
};

// 两数相加 我的第一个解法
class Solution2
{
public:
    ListNode *addTwoNumbers(ListNode *l1, ListNode *l2)
    {
        // 判断输入空的情况
        if (l1 == nullptr && l2 == nullptr)
            return nullptr;
        if (l1 == nullptr)
            return l2;
        if (l2 == nullptr)
            return l1;

        // 定义头指针
        int sum = l1->val + l2->val;
        ListNode *head = new ListNode(sum % 10);
        sum = sum / 10;
        ListNode *current = head;
        // 遍历链表
        ListNode *current1 = l1->next;
        ListNode *current2 = l2->next;

        while (true)
        {
            // 求当前节点的和
            if (current1 != nullptr && current2 != nullptr)
            {
                sum += current1->val + current2->val;
                current1 = current1->next;
                current2 = current2->next;
            }
            else if (current1 != nullptr && current2 == nullptr)
            {
                sum += current1->val;
                current1 = current1->next;
            }
            else if (current2 != nullptr && current1 == nullptr)
            {
                sum += current2->val;
                current2 = current2->next;
            }
            else
            {
                // 两个节点都空了，把剩下的和进位
                while (sum >= 10)
                {
                    current->next = new ListNode(sum % 10);
                    sum = sum / 10;
                    current = current->next;
                }
                // 如果仍然大于0，再插入一个节点
                if (sum > 0)
                {
                    current->next = new ListNode(sum);
                    current = current->next;
                }
                break;
            }
            // 进位
            current->next = new ListNode(sum % 10);
            sum = sum / 10;
            current = current->next;
        }

        return head;
    }
};

// 官方解题
class Solution3
{
public:
    ListNode *addTwoNumbers(ListNode *l1, ListNode *l2)
    {
        ListNode *head = nullptr, *tail = nullptr;
        int carry = 0;
        while (l1 || l2)
        {
            int n1 = l1 ? l1->val : 0;
            int n2 = l2 ? l2->val : 0;
            int sum = n1 + n2 + carry;
            if (!head)
            {
                head = tail = new ListNode(sum % 10);
            }
            else
            {
                tail->next = new ListNode(sum % 10);
                tail = tail->next;
            }
            carry = sum / 10;
            if (l1)
            {
                l1 = l1->next;
            }
            if (l2)
            {
                l2 = l2->next;
            }
        }
        if (carry > 0)
        {
            tail->next = new ListNode(carry);
        }
        return head;
    }
};

// 两数相加 我的第二个解法：差别不大
class Solution
{
public:
    ListNode *addTwoNumbers(ListNode *l1, ListNode *l2)
    {
        int sum = 0;
        ListNode *head = new ListNode(0);
        ListNode *current = head;
        // 遍历链表
        ListNode *current1 = l1;
        ListNode *current2 = l2;
        while (true)
        {
            if (current1 != nullptr && current2 != nullptr)
            {
                sum += current1->val + current2->val;
                current1 = current1->next;
                current2 = current2->next;
            }
            else if (current1 != nullptr && current2 == nullptr)
            {
                sum += current1->val;
                current1 = current1->next;
            }
            else if (current2 != nullptr && current1 == nullptr)
            {
                sum += current2->val;
                current2 = current2->next;
            }
            current->val = sum % 10;
            sum = sum / 10;

            if (current1 == nullptr && current2 == nullptr)
            {
                if (sum >= 10)
                {
                    current->next = new ListNode(sum % 10);
                    if (sum / 10 > 0)
                        current->next->next = new ListNode(sum / 10);
                }
                else if (sum > 0)
                {
                    current->next = new ListNode(sum);
                }

                break;
            }
            else
            {
                current->next = new ListNode(0);
                current = current->next;
            }
        }

        return head;
    }
};

int main()
{
    Solution s;
    // int arr1[] = {1, 2, 3, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7};
    // int arr2[] = {4, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7};
    // int arr1[] = {9, 9, 9, 9, 9, 9, 9};
    // int arr2[] = {9, 9, 9, 9};
    int arr1[] = {2, 4, 3};
    int arr2[] = {5, 6, 4};
    // 创建链表1
    ListNode *list1 = createLinkedList(arr1, sizeof(arr1) / sizeof(arr1[0]));
    // 创建链表2
    ListNode *list2 = createLinkedList(arr2, sizeof(arr2) / sizeof(arr2[0]));
    printLinkedList(list1);
    printLinkedList(list2);
    struct ListNode *L = s.addTwoNumbers(list1, list2);
    printLinkedList(L);

    return 0;
}