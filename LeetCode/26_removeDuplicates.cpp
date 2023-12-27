/*给你一个 升序排列 的数组 nums ，请你 原地 删除重复出现的元素，使每个元素 只出现一次 ，返回删除后数组的新长度。
元素的 相对顺序 应该保持 一致 。然后返回 nums 中唯一元素的个数。
考虑 nums 的唯一元素的数量为 k ，你需要做以下事情确保你的题解可以被通过：
更改数组 nums ，使 nums 的前 k 个元素包含唯一元素，并按照它们最初在 nums 中出现的顺序排列。
nums 的其余元素与 nums 的大小不重要。
返回 k 。

示例：
输入：nums = [0,0,1,1,1,2,2,3,3,4]
输出：5, nums = [0,1,2,3,4]
解释：函数应该返回新的长度 5 ， 并且原数组 nums 的前五个元素被修改为 0, 1, 2, 3, 4 。不需要考虑数组中超出新长度后面的元素。*/

#include <iostream>
#include <vector>

using namespace std;

class Solution
{
public:
    // 这种方法是错误的，当只有1个数或者数组所有元素相同时错误
    int removeDuplicates(vector<int> &nums)
    {
        vector<int> expectedNums;
        for (int i = 0; i < (int)nums.size() - 1; i++)
        {
            if (nums[i] < nums[i + 1])
            {
                expectedNums.push_back(nums[i]);
                if (i == (int)nums.size() - 2)
                    expectedNums.push_back(nums[i + 1]);
            }
        }
        nums.swap(expectedNums);
        return (int)nums.size();
    }

    int removeDuplicates3(vector<int> &nums)
    {
        if((int)nums.size() == 1)
            return 1;
        if((int)nums.size() == 2)
        {
            if(nums[0]==nums[1])
                return 1;
            else
                return 2;
        }
        vector<int> expectedNums;
        for (int i = 0; i < (int)nums.size() - 1; i++)
        {
            if (i == (int)nums.size() - 2)
            {
                if (nums[i] < nums[i + 1])
                    expectedNums.push_back(nums[i]);
                expectedNums.push_back(nums[i+1]);
            }
            else if (nums[i] < nums[i + 1])
                    expectedNums.push_back(nums[i]);
        }
        if(expectedNums.empty())
            return 1;
        nums.swap(expectedNums);
        return (int)nums.size();
    }

    int removeDuplicates2(vector<int> &nums)
    {
        int i, k = 1;
        for (i = 1; i < (int)nums.size(); i++)
        {
            if (nums[i] > nums[k - 1])
            {
                nums[k] = nums[i];
                k++;
            }
        }
        return k;
    }
};

int main(int argc, char *argv[])
{
    vector<int> nums{0, 0, 1, 1, 1, 2, 2, 3, 3, 4};
    Solution s;
    int result = s.removeDuplicates2(nums);

    cout << result << ":[";
    for (int i = 0; i < result; i++)
    {
        if (i == result - 1)
            cout << nums[i];
        else
            cout << nums[i] << ",";
    }
    cout << "]" << endl;

    return 0;
}