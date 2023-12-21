#include <iostream>
#include "opencv2/opencv.hpp"
#include <ctime> //时间库，计时用
#include <cstring>
using namespace std;
using namespace cv;

VideoCapture cap;
VideoWriter outputVideo;

int writevideo()
{
    cap.open(0);
    //cap.set(CAP_PROP_FRAME_WIDTH, 1024);
	//cap.set(CAP_PROP_FRAME_HEIGHT, 768);


	if (!cap.isOpened())
	{
		return -1;
	}
	int w = (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
	int h = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);

	int frameRate = cap.get(CV_CAP_PROP_FPS);
	cout << frameRate << endl;
	int frameNum = 0;

	Mat frame, img;

	//保存视频的路径
	string outputVideoPath = "../../opencv/data/test1.mp4";
	//outputVideo.open(outputVideoPath, CV_FOURCC('M', 'J', 'P', 'G'), 24.0, Size(640, 480));	//这是保存为avi的
	outputVideo.open(outputVideoPath, CV_FOURCC('D', 'I', 'V', 'X'), 30.0, Size(640, 480));
	while (1)
	{
		cap >> frame;

		resize(frame, frame, Size(640, 480));

		outputVideo.write(frame);


		frameNum += 1;
		cout << frameNum << endl;

		imshow("show", frame);
		//按键开始，暂停 	1000/25=40s 
		if (waitKey(30) == 27 || frameNum == 250)// 空格暂停
		{
			//waitKey(0);
			break;
		}

	}
	outputVideo.release();
	return 0;
}

int main() 
{
    namedWindow( "Example", WINDOW_AUTOSIZE );
    VideoCapture cap1;               //视频类
    cap1.open("../../opencv/data/testvideo.avi"); 
    //cap1.open("../../opencv/data/test.mp4");       //不能读mp4,待解决
    int P=0;
    if (cap1.isOpened())
    {
        P=cap1.get(CAP_PROP_FRAME_COUNT);
        cout<<"视频总帧数："<<P<<endl;
        cout<<"视频宽度："<<cap1.get(CAP_PROP_FRAME_HEIGHT)<<endl;
        cout<<"视频高度："<<cap1.get(CAP_PROP_FRAME_HEIGHT)<<endl;
        cout<<"视频帧率："<<cap1.get(CAP_PROP_FPS)<<endl;
    }
    else
    {
        cout << "请确认视频文件名称是否正确!" << endl;
        
        return -1;
    }
    
    ofstream f;
    f.open("/home/zrad/Desktop/OutputData/temp.txt");
    f << fixed;
    for(int i=0;;i++) 
    {
        Mat frame;         //用于存放每一帧
        cap1.read(frame);
        //cap>>frame;//等价上一句
        if( frame.empty() ) 
        {
            cout<<"BREAK end!"<<endl;
            break;
        };
        imshow( "Example", frame );
        //waitKey(1000/24);//每帧多少毫秒
        waitKey(2000/cap1.get(CAP_PROP_FPS));
        
        string filename="/home/zrad/Desktop/OutputData/rgb/image"+to_string(i)+".jpg";
        imwrite(filename, frame);//保存图片
        
        double pt=(double)(clock())/CLOCKS_PER_SEC;//当前时间，转化为秒
        f<<pt<<" "<<"rgb/image"<<i<<".jpg"<<endl;
    }
    f.close();
    
    //writevideo();
    
    return 0;
}
