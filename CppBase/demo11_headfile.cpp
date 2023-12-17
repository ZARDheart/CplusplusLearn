#include "demo11_head.h"

int main()
{
    // 数组作参数
    double ar[8] = {1.1, 2.2, 3, 4, 5, 6, 7.2, 8.1}, br[2][3] =
                                                         {
                                                             {4.2, 5.6, 8.9},
                                                             {7.6, 9.9, 7.5},
                                                         };
    int n = sizeof(ar) / sizeof(ar[0]);
    cout << avange(ar, n) << endl; // 数组名作参数
    cout << sum(br, 2) << endl;

    // 结构体作参数
    point p = {0.1, 0.2, 0.3}, pt;
    double trans[3] = {1, 2, 3};
    pt = translation(p, trans);
    cout << pt.x << "\t" << pt.y << "\t" << pt.z << endl;

    return 0;
}
