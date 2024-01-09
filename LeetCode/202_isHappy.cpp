/*编写一个算法来判断一个数 n 是不是快乐数。

「快乐数」 定义为：
对于一个正整数，每一次将该数替换为它每个位置上的数字的平方和。
然后重复这个过程直到这个数变为 1，也可能是 无限循环 但始终变不到 1。
如果这个过程 结果为 1，那么这个数就是快乐数。
如果 n 是 快乐数 就返回 true ；不是，则返回 false 。

示例：
输入：n = 19
输出：true
解释：
1^2 + 9^2 = 82
8^2 + 2^2 = 68
6^2 + 8^2 = 100
1^2 + 0^2 + 0^2 = 1*/

#include <iostream>
#include <vector>
#include <cmath>
// #include <unordered_map>
#include <unordered_set>

using namespace std;

class Solution
{
public:
    Solution() {}

    bool isHappy(int n)
    {
        std::unordered_set<int> set;
        while (1)
        {
            // 无限循环，那么也就是说求和的过程中，sum会重复出现，用哈希表
            int t = getResult(n);
            if (set.find(t) != set.end()) // 重复出现且不等于1，所以不是快乐数
                return false;
            else
            {
                if (t == 1) // 算到1了，是快乐
                    return true;
                set.insert(t); // 插入哈希表
            }
            n = t; // 替换
        }
    }

    bool isHappy2(int n)
    {
        while (1)
        {
            // 事实上，所有的无限循环最后都会进入的4循环中
            // 4,16,37,58,89,145,42,20,4.....
            int t = getResult(n);
            if (t == 1)
                return true;
            else if (t == 4)
                return false;
            n = t;
        }
    }

    int getResult(int n)
    {
        int t = 0;
        while (n)
        {
            t += pow(n % 10, 2);
            n = n / 10;
        }

        return t;
    }
};

int main(int argc, char *argv[])
{
    Solution s;
    if (s.isHappy(atoi(argv[1])))
        cout << "There is a 快乐数." << endl;
    else
        cout << "There is not a 快乐数." << endl;

    return 0;
}
