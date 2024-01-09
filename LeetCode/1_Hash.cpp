/*给定一个整数数组 nums 和一个整数目标值 target，请你在该数组中找出 和为目标值 target 的那两个整数，并返回它们的数组下标。
你可以假设每种输入只会对应一个答案。但是，数组中同一个元素在答案里不能重复出现。
你可以按任意顺序返回答案。

示例：
输入：nums = [2,7,11,15], target = 9
输出：[0,1]
解释：因为 nums[0] + nums[1] == 9 ，返回 [0, 1] 。*/

#include <iostream>
#include <vector>
#include <unordered_map>
// #include <hash_map>

using namespace std;

class Solution
{
public:
    Solution() {}

    // 暴力穷举法
    vector<int> twoSumForce(vector<int> &nums, int target)
    {
        for (int i = 0; i < (int)nums.size(); i++)
        {
            // 第二个数必然在第一个之后
            for (int j = i + 1; j < (int)nums.size(); j++)
            {
                if (nums[i] + nums[j] == target)
                {
                    return {i, j};
                }
            }
        }
        return {-1};
    }

    // 哈系表法
    /*如果构造一种存储结构，能通过某种函数（hashFunc）使元素的存储位置与它的关键码之间能够建立一种映射的关系，
        那么在查找时通过该函数可以很快找到该元素。

        向该结构当中插入和搜索元素的过程如下：
        插入元素： 根据待插入元素的关键码，用此函数计算出该元素的存储位置，并将元素存放到此位置。
        搜索元素： 对元素的关键码进行同样的计算，把求得的函数值当作元素的存储位置，在结构中按此位置取元素进行比较，
                若关键码相等，则搜索成功。*/
    vector<int> twoSumHash(vector<int> &nums, int target)
    {
        // 定义哈希表
        std::unordered_map<int, int> map;
        for (int i = 0; i < (int)nums.size(); i++)
        {
            // 遍历当前元素，并在map中寻找是否有匹配的key
            auto iter = map.find(target - nums[i]);
            if (iter != map.end())
            {
                return {iter->second, i};
            }
            // 如果没找到匹配对，就把访问过的元素和下标加入到map中
            map.insert(pair<int, int>(nums[i], i));
        }
        return {-1};
    }
};

int main(int argc, char *argv[])
{
    vector<int> nums{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    int target = 10;
    Solution s;
    vector<int> result = s.twoSumHash(nums, target);
    if (result[0] < 0)
        cout << "There is on result." << endl;
    else
        cout << "Result: [" << result[0] << "," << result[1] << "]" << endl;
        
    return 0;
}
