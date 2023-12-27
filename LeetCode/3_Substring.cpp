#include <iostream>
#include <string>
#include <unordered_set>
using namespace std;
/*
给定一个字符串 s ，请你找出其中不含有重复字符的 最长子串 的长度。
示例 1:
    输入: s = "abcabcbb"
    输出: 3
    解释: 因为无重复字符的最长子串是 "abc"，所以其长度为 3。
示例 2:
    输入: s = "pwwkew"
    输出: 3
    解释: 因为无重复字符的最长子串是 "wke"，所以其长度为 3。
        请注意，你的答案必须是 子串 的长度，"pwke" 是一个子序列，不是子串。
*/

class Solution
{
public:
    // 题目理解错了
    int lengthOfLongestSubstring1(string s)
    {
        int lastmax = 0, maxlen = 0;
        string subString;
        // 将 i 的类型更改为与 s.size() 相同的类型，这样就可以避免类型不匹配的问题。
        // 具体而言，你可以将 int i 改为 decltype(s.size()) i ，或者反过来i < (int)nums.size()
        for (decltype(s.size()) i = 0; i < s.size(); i++)
        {
            if (s[i] != s[i + 1])
            {
                maxlen++;
            }
            else
            {
                if (lastmax < maxlen)
                    lastmax = maxlen;
                maxlen = 0;
            }
        }
        return lastmax;
    }

    // 官方答案
    int lengthOfLongestSubstring(string s)
    {
        // 哈希集合，记录每个字符是否出现过
        std::unordered_set<char> occ;
        int n = s.size();
        // 右指针，初始值为 -1，相当于我们在字符串的左边界的左侧，还没有开始移动
        int rk = -1, ans = 0;
        // 枚举左指针的位置，初始值隐性地表示为 -1
        for (int i = 0; i < n; ++i)
        {
            if (i != 0)
            {
                // 左指针向右移动一格，移除一个字符
                occ.erase(s[i - 1]);
            }
            while (rk + 1 < n && !occ.count(s[rk + 1]))
            {
                // 不断地移动右指针
                occ.insert(s[rk + 1]);
                ++rk;
            }
            // 第 i 到 rk 个字符是一个极长的无重复字符子串
            ans = max(ans, rk - i + 1);
        }
        return ans;
    }
};

int main()
{
    string str = "abcabcbb";
    string s = "pwwkew";
    Solution solu;
    cout << str << endl;
    cout << solu.lengthOfLongestSubstring(s) << endl;
    return 0;
}