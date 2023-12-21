#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

int main() 
{
  Mat img_1 = imread("../../opencv/data/1.png");
  Mat img_2 = imread("../../opencv/data/2.png");
  assert(img_1.data != nullptr && img_2.data != nullptr);
  
  //-- 初始化
  vector<KeyPoint> keypoints_1, keypoints_2;
  Mat descriptors_1, descriptors_2;
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

  //-- 第一步:检测 Oriented FAST 角点位置
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  //-- 第二步:根据角点位置计算 BRIEF 描述子
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);
  Mat outimg1;
  drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
  imshow("ORB features", outimg1);

  //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
  vector<DMatch> matches;
  matcher->match(descriptors_1, descriptors_2, matches);

  //-- 第四步:匹配点对筛选
  // 计算最小距离和最大距离
  auto min_max = minmax_element(matches.begin(), matches.end(),
                                [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
  double min_dist = min_max.first->distance;
  double max_dist = min_max.second->distance;
  printf("-- Max Hamming distence : %f \n", max_dist);
  printf("-- Min Hamming distence : %f \n", min_dist);
  //描述子之间距离大于两倍的最小距离即认为误匹配.有时最小距离会非常小,设置经验值30作下限.
  vector<DMatch> good_matches;
  for (int i = 0; i < descriptors_1.rows; i++) 
  {
    if (matches[i].distance <= max(2 * min_dist, 30.0)) 
    {
      good_matches.push_back(matches[i]);
    }
  }

  //-- 第五步:绘制匹配结果
  Mat img_match;
  Mat img_goodmatch;
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
  drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
  imshow("all matches", img_match);
  imshow("good matches", img_goodmatch);
  cout<<"-- good/all:"<<good_matches.size()<<"/"<<matches.size()<<endl;
  waitKey(0);

  return 0;
}
