#include <iostream>
#include <opencv2/opencv.hpp>
#include <typeinfo>
using namespace std;
using namespace cv;

void createAlphaMat(Mat &mat)//创建带alpha通道的Mat
{
    for(int i = 0; i < mat.rows; ++i) 
    {
        for(int j = 0; j < mat.cols; ++j) 
        {
            Vec4b&rgba = mat.at<Vec4b>(i, j);
            rgba[0]= UCHAR_MAX;      //蓝色通道
            rgba[1]= saturate_cast<uchar>((float (mat.cols - j)) / ((float)mat.cols) *UCHAR_MAX);
            rgba[2]= saturate_cast<uchar>((float (mat.rows - i)) / ((float)mat.rows) *UCHAR_MAX);
            rgba[3]= saturate_cast<uchar>(0.5 * (rgba[1] + rgba[2]));
        }
    }
}

int main()
{
    //  --1 imread：
    Mat srcImage = imread("../../opencv/data/透明Alpha值图.png"); //imread(filename,flags)函数读取指定路径下的图像.如果图像不能读取（由于文件丢失、权限不正确、不受支持或格式无效），函数返回空矩阵（Mat:：data==NULL）。flags读取图像形式：灰度：IMREAD_GRAYSCALE/RGB:IMREAD_COLOR(默认)/IMREAD_ ANYCOLOR通道数由文件实际通道数(不超过3)/IMREAD ANYDEPTH允许加载超过8bit深度/IMREAD UNCHANGED等于将IMREAD_ ANYCOLOR和IMREAD_ ANYDEPTH组合起来
    cout << "图像宽为" << srcImage.cols << ",高为" << srcImage.rows<<",通道数为" << srcImage.channels() << endl;
    //通道分离与合并
    Mat mv0,mv1,mv2,result,mv[3];
    split(srcImage,mv); //分离
    mv1=mv[1];
    imshow("分离绿通道",mv1);
    merge(mv,3,result);
    imshow("3通道合并",result);
    
    // --2 imshow:
    namedWindow("示例1",WINDOW_NORMAL);//创建窗口，窗口名称-窗口模式，WINDOW_NORMAL:可调窗口大小/WINDOW_AUTOSIZE：根据图像大小显示/WINDOW_FULLSCREEN:全屏显示...
    imshow("示例1",srcImage);    // 用cv::imshow显示图像，整型映射到矩阵[0,255],浮点型映射到[0,1],可放在同名窗口中，使用原窗口模式。若无同名窗口则自动创建(WINDOW_AUTOSIZE模式)
    waitKey(0);                  // waitKey函数，它显示指定的图像毫秒。否则，它不会显示图像。例如waitKey（0）将显示窗口无限地直到任何按键（它适用于图像显示）。waitKey（25）将显示一个帧持续25毫秒，之后显示器将自动关闭。（如果你把它放在一个循环里读视频，将逐帧显示视频）
    //destroyWindow("示例1");      //清理某一个窗口
    //destroyAllWindow();        //清理所有窗口
    
    // --3 inwrite:
    imwrite("../../opencv/data/demo2_save图.png",srcImage);//filename 文件的名称，CV_16U可存为:png/jpg/tiff...,CV_32F可存为:PFM/TIFF/...,4通道可存为png格式。img：要保存的参数图像。一般就是Mat类型的。params：特定格式保存的参数编码，有默认值 std::vector<int>());一般不要填。
    
    // --4 实例：
    //创建带alpha(透明度)通道的Mat
    Mat mat(480, 640, CV_8UC4);
    createAlphaMat(mat);
    vector<int>compression_params;
    compression_params.push_back(IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    try
    {
        imwrite("../../opencv/data/透明Alpha值图.png", mat, compression_params);//保存图片
        imshow("生成的png图",mat);
        cout<<"PNG图片文件的alpha数据保存完毕~\n可以在工程目录下查看!"<<endl;
        waitKey(0);
    }
    catch(runtime_error& ex) 
    {
        fprintf(stderr,"图像数据转换成PNG格式发生错误：%s\n", ex.what());
        return 1;
    }
    
    return 0;
}
