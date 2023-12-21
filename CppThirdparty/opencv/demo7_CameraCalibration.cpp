#include <opencv2/opencv.hpp>
#include <fstream> 
#include <iostream>
#include <vector>
#include <cstring>
using namespace std;
using namespace cv;

//使用initUndistortRectifyMap()函数和remap()函数校正图像
void initUndistAndRemap(vector<Mat> imgs,  //所有原图像向量
	                    Mat cameraMatrix,  //计算得到的相机内参
	                    Mat distCoeffs,    //计算得到的相机畸变系数
	                    Size imageSize,    //图像的尺寸
	                    vector<Mat> &undistImgs)  //校正后的输出图像
{
	//计算映射坐标矩阵
	Mat R = Mat::eye(3, 3, CV_32F);
	Mat mapx = Mat(imageSize, CV_32FC1);
	Mat mapy = Mat(imageSize, CV_32FC1);
	initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, imageSize, CV_32FC1, mapx, mapy);                         //内参矩阵/畸变系数/...
	for (int i = 0; i < imgs.size(); i++)//校正图像
	{
		Mat undistImg;
		remap(imgs[i], undistImg, mapx, mapy, INTER_LINEAR);
		undistImgs.push_back(undistImg);
	}
}

//用undistort()函数直接计算校正图像,自定义函数是为了处理多幅图像
void undist(vector<Mat> imgs,   //所有原图像向量
	        Mat cameraMatrix,   //计算得到的相机内参
	        Mat distCoeffs,     //计算得到的相机畸变系数
	        vector<Mat> &undistImgs)  //校正后的输出图像
{
	for (int i = 0; i < imgs.size(); i++)
	{
		Mat undistImg;
		undistort(imgs[i], undistImg, cameraMatrix, distCoeffs);//单幅图像去畸变:畸变图像/去畸变后的图像/内参矩阵/畸变系数
		undistImgs.push_back(undistImg);
	}
}

int main()
{
    // --1 标定板角点提取（与下文独立）
	Mat img0 = imread("../../opencv/data/calibrate/left01.jpg"),img2 = imread("../../opencv/data/calibrate/circle.png");
	Mat gray1, gray2;
	cvtColor(img0, gray1, COLOR_BGR2GRAY); //转化为灰度
    cvtColor(img2, gray2, COLOR_BGR2GRAY);
	Size board_size1 = Size(9, 6),         //定义数目尺寸，方格标定板内角点数目（行，列）
        board_size2 = Size(7, 7);          //圆形标定板圆心数目（行，列）
	vector<Point2f> img0_points, img2_points;//检测角点，单副图像放入vector<Point2f>
	findChessboardCorners(gray1, board_size1, img0_points);     //计算方格标定板角点
    findCirclesGrid(gray2, board_size2, img2_points);           //计算圆形标定板检点
	find4QuadCornerSubpix(gray1, img0_points, Size(5, 5));      //细化方格标定板角点坐标,Size(5, 5)为细化方格坐标领域范围
    find4QuadCornerSubpix(gray2, img2_points, Size(5, 5));      //细化圆形标定板角点坐标
	drawChessboardCorners(img0, board_size1, img0_points, true);//绘制棋盘角点检测结果
    drawChessboardCorners(img2, board_size2, img2_points, true);
	imshow("方形标定板角点检测结果", img0);                     //显示结果
    imshow("圆形标定板角点检测结果", img2);
	waitKey();
    
    // --2 相机内参及畸变标定
	vector<Mat> imgs;
	string imageName,imagepath="../../opencv/data/calibrate/";
	ifstream fin("../../opencv/data/calibrate/calibdata.txt");  //读取所有图像，文本里有各个图像文件名
	while (getline(fin,imageName))
	{
		Mat img = imread(imagepath+imageName);
		imgs.push_back(img);
	}
	//计算标定板角点坐标
	Size board_size = Size(9, 6);
	vector<vector<Point2f>> imgsPoints;    //多副图像分别放入vector<vector<Point2f>>
	for (int i = 0; i < imgs.size(); i++)
	{
		Mat img1 = imgs[i], gray1;
		cvtColor(img1, gray1, COLOR_BGR2GRAY);
		vector<Point2f> img1_points;
		findChessboardCorners(gray1, board_size, img1_points);
		find4QuadCornerSubpix(gray1, img1_points, Size(5, 5));
		imgsPoints.push_back(img1_points);
	}
	//使用棋盘格每个内角点的空间三维坐标
	Size squareSize = Size(10, 10);      //棋盘格每个方格的真实尺寸
	vector<vector<Point3f>> objectPoints;//空间三维坐标（位于一个平面内，以此为xy坐标平面）
	for (int i = 0; i < imgsPoints.size(); i++)
	{
		vector<Point3f> tempPointSet;
		for (int j = 0; j < board_size.height; j++)
		{
			for (int k = 0; k < board_size.width; k++)
			{
				Point3f realPoint;       // 假设标定板为世界坐标系的z平面，即z=0
				realPoint.x = j*squareSize.width;
				realPoint.y = k*squareSize.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		objectPoints.push_back(tempPointSet);
	}
	// 内参及畸变标定
	Size imageSize;     //图像尺寸
	imageSize.width = imgs[0].cols;
	imageSize.height = imgs[0].rows;
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));  //相机内参数矩阵
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));    //相机的5个畸变系数
	vector<Mat> rvecs,tvecs;  //每幅图像的旋转向量/平移向量
	calibrateCamera(objectPoints, imgsPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, 0);
	cout << "相机的内参矩阵=" << endl << cameraMatrix << endl;
	cout << "相机畸变系数:" << distCoeffs << endl;
	waitKey(0);
    
    // --3 图像去畸变（承接2）
	vector<Mat> undistImgs;
	//使用initUndistortRectifyMap()函数和remap()函数校正图像
	initUndistAndRemap(imgs,cameraMatrix,distCoeffs,imageSize,undistImgs);//畸变图像/前文计算得到的内参矩阵/畸变系数/图像尺寸/去畸变后的图像,自定义函数是为了处理多副图像
	//undist(imgs, cameraMatrix, distCoeffs, undistImgs);//用undistort()函数直接计算校正图像,自定义函数是为了处理多副图像
	//显示校正前后的图像
	for (int i = 0; i < imgs.size(); i++)
	{
		string windowNumber = to_string(i);
		imshow("未校正图像"+windowNumber, imgs[i]);
		imshow("校正后图像"+windowNumber, undistImgs[i]);
	}
	waitKey(0);
    
    // --4 单目投影：根据成像模型及空间点三位坐标计算图像二维坐标
    //使用上文相机标定时的第一张图像，各项参数都是标定时得到的
	Mat rvec = rvecs[0],tvec = tvecs[0];//第一张图像相机坐标系与世界坐标系之间的关系
	vector<Point3f> PointSets=objectPoints[0];//生成第一张图像中内角点的三维世界坐标(方便比较投影估计与真实投影)
	//根据三维坐标和相机与世界坐标系时间的关系估计内角点像素坐标
	vector<Point2f> imagePoints; //存放二维坐标
	projectPoints(PointSets, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);//输入三维点/旋转及平移向量/前文计算得到的内参矩阵和畸变矩阵/输出二维点
	//计算估计值和图像中计算的真实时之间的平均误差
	float e = 0;
	for (int i = 0; i < imagePoints.size(); i++)
	{
		float eX = pow(imagePoints[i].x - imgsPoints[0][i].x, 2);//图像中内角点的真实坐标误差
		float eY = pow(imagePoints[i].y - imgsPoints[0][i].y, 2);
		e = e + sqrt(eX + eY);
	}
	cout << "估计坐标与真实坐标之间的误差" << e/imagePoints.size() << endl;
    
    // --5 单目位姿估计
	solvePnP(PointSets, imgsPoints[0], cameraMatrix, distCoeffs, rvec, tvec);//三维坐标/像素坐标/内参矩阵/畸变参数/旋转向量/平移向量
	cout << "世界坐标系变换到相机坐标系的旋转向量：" << endl << rvec << endl<<"平移向量："<<tvec<<endl;
	Mat R, rvecRansac, tvecRansac, RRansac;
	Rodrigues(rvec, R);//旋转向量转换旋转矩阵
	cout << "旋转向量转换成旋转矩阵：" << endl << R << endl;
	//用PnP+Ransac算法计算旋转向量和平移向量
	solvePnPRansac(PointSets,imgsPoints[0],cameraMatrix,distCoeffs,rvecRansac,tvecRansac);
	Rodrigues(rvecRansac, RRansac);
	cout << "旋转向量转换成旋转矩阵：" << endl << RRansac  << endl<<"平移向量："<<tvecRansac<<endl;

	return 0;
}
