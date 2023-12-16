#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int main()
{
    //（1）point类：
    Point3d p(201,152,255),p1(1,2,3);        //也有Point2d/point2f/point2i/point3i...
    Point3d p2(p1);                          //复制构造
    cout<<"point类："<<endl<<p.x<<","<<p<<endl;    //单个及整体输出
    cout<<p.dot(p1)<<","<<p2.cross(p1)<<endl<<endl;//点乘，叉乘
    vector<Point2f> p3;
    
    //（2）scalar四维点类：
    Scalar s(1,2,3,4);
    Scalar s1(s);
    cout<<"scalar四维点类"<<endl<<s.mul(s1)<<","<<s.conj()<<endl<<endl; //元素相乘,共轭
    
    //（3）vector固定向量类：
    Vec3d V1{1.2,5.6,8.4},//4个double元素,还有各种其他Vector类型,如Vec3i/Vec2f...,用一种模式表示为Vec{2,3,4,5,6}{b,s,i,f,d}
        V2{3,2,3};
    cout<<"vector固定向量类:"<<V1[1]<<endl<<V1.cross(V2)<<endl<<V1.dot(V2)<<endl<<endl;//访问元素/叉乘/内积
    
    //（4）Matx固定矩阵类（作模板用）：
    Matx33d m3d(3,2,1,4,6,5,7,8,9);          //类型模式：Matx{1,2,3...6}{1,2,3...6}{f,d}
    Matx34d m34d(1,2,3,4,5,6,7,8,9,10,11,12);
    Matx33d m33d(m3d),m5d = Matx33d::all(5); //全5元素阵
    cout<<"Matx固定矩阵类:"<<endl<<m34d<<endl<<m5d<<endl;
    Matx23d m23d = Matx23d::zeros();         //全零矩阵
    Matx16f m16f = Matx16f::ones();          //全一-矩阵
    Matx33f m33f1 = Matx33f::eye(),          //创建一个单位矩阵
        m33f2 = Matx33f::randu(1,9);         //创建一个均匀分布的矩阵randu(min, max)
    cout<<m33f1(2,2)<<endl<<m33f2<<endl<<m33f1<<"@"<<m33f2<<endl;  //成员访问
    cout<<3*m33f1<<endl<<m33f1+m33f2<<endl<<m33f1-m33f2<<endl<<m33f1*m33f2<<endl; //矩阵运算
    cout<<"数组乘/点积:"<<endl<<m33f1.mul(m33f2)<<endl<<m33f1.dot(m33f2)<<endl;//数组乘/点积(内积)
    Matx23f m23f = m16f.reshape<2,3>();      // 重塑形状
    cout<<"重塑形状:"<<endl<<m23f<<endl;
    Matx33d m33d2 = (Matx33d)m33f1;          //类型转换
    cout<<m33f2.get_minor<2,2>(0,0)<<endl;   //提取(0,0)处的2*2子矩阵(0:1,0:1)
    cout<<"提取第行列"<<endl<<m33d2.row(1)<<endl<<m33d2.col(1)<<endl; //提取第1行/提取第j列
    cout<<m33d2.diag()<<endl<<m33f2.t()<<endl;//提取矩阵对角线/计算转置
    cout<<"逆矩阵:"<<(2*m33d2).inv()<<endl;//逆矩阵.inv(method)(default method:Cv:DECOMP_LU)
    Matx31f m31f,m31f1(1,2,3);
    Matx32f m32f,m32f1(1,2,3,4,5,6);
    m31f = m33f1.solve(m31f1);               //解线性方程组
    m32f = m33f1.solve<2>(m32f1);            // (default method is DECOMP_LU)
    cout<<"解线性方程组:"<<endl<<m31f<<endl<<m32f<<endl<<endl;
    
    //（5）复数类：
    Complexf Z1;                            //默认构造函数
    Complexd Z2(2,3);                       //复制构造函数(re0,im1)
    Complexd Z3(Z2);                        //复制构造函数
    cout<<"复数类："<<endl<<Z2.re<<"+"<<Z2.im<<"i"<<endl<<Z2.conj()<<endl; //成员访问/复共轭
    
    //（6）智能指针
    /*智能指针(smart pointer)是C++中一个非常有用的类型。这个指针允许我们创建一个对象的引用，
     * 然后把它传递到各处。你可以创建更多的对该对象的引用，然后所有这些引用都会被计数。
     * 当引用超出范围，智能指针的引用计数就会减少。一旦所有的引用(指针的实例)消失，这个对象将自动清理(释放)。而作为程序员的你，不再需要记录这些东西。*/
    
    //（7）Mat类：
    /*
     Mat_<uchar>---------CV_8U
    Mat<char>-----------CV_8S
    Nat_<short>---------CV_16S
    Mat_<ushort>--------CV_16U
    Mat_<int>-----------CV_32S
    Mat_<float>----------CV_32F
    Mat_<double>--------CV_64F
     */
    //创建Mat类
    Mat Ma,Mc(120,160,CV_8UC3,Scalar(255,0,0));  //创建120*130，3通道8位无符号整型矩阵头,type的类型为CV_8UC1,CV_8UC2,CV_8UC3，CV_8UC4，CV_8UC（n）...，最后一个数字是创建n通道的矩阵。Scalar(0,0,255)给每一像素赋相同值(可不用)。灰度图一般是1，彩色的是RGB3个通道，4代表具有透明色的彩色图片。
    imshow("图1",Mc);waitKey(0);
    cout <<"Mat类："<<endl<<"图像宽为" << Mc.cols << ",高为" << Mc.rows<<",通道数为" << Mc.channels() << endl; //Mat(int rows, int cols, int type);rows代表2D数组中的行数，cols代表2D数组中的列数，这两个参数可以合并起来写成Size（size）.
    Ma=imread("../../opencv/data/透明Alpha值图.png");    //赋数据
    Mat Mb=Ma,Md,Mbb;                  //复制矩阵头
    Md.create(3,10,CV_32FC3);          //为Mat创建空间
    float Mbd[8]={1,2,3,4,5,6,7,8};
    Mbb=Mat(2,2,CV_32FC2,Mbd);         //利用数组构造
    cout<<Mbb<<endl;                   //输出整个矩阵(三维显示不对)
    Mbb=Mat(2,4,CV_32FC1,Mbd);
    cout<<Mbb<<endl;
    //特殊矩阵
    Mat Meye=Mat::eye(3,3,CV_8UC1),
        Mones=Mat::ones(3,3,CV_8UC1),
        Mzeros=Mat::zeros(3,3,CV_8UC1),
        Mv=(Mat_<int>(1,3)<<1,2,3);    //创建普通矩阵，并枚举法赋值
    Mat Mdiag=Mat::diag(Mv);           //依据一维Mat创建对角阵
    cout<<Mdiag<<endl;
    //处理矩阵(访问及遍历)
    Mat Me=Mat_<int>(10,10);
    for(int i=0;i<10;i++)
        for(int j=0;j<10;j++)
                Me.at<int>(i,j)=i+j;   // （1）at方法访问并遍历矩阵
    cout<<Me.at<int>(5,5)<<endl;
    cout<<(int)Mc.at<cv::Vec3b>(100,100)[0]<<endl; //访问多维,unsigned char(uchar)-8U-Vec3b,3i-int/3s-short/3w-ushort/3d-double/3f-float/2b...
    for(int i=0;i<Mc.rows;i++)         // （2.1）ptr方法访问位于 i,j (k)处的像素
    {
        uchar* ptr=Mc.ptr<uchar>(i);   // cv::Mat::ptr获得图像的行指针,ptr是第i行的头指针,因为Mat数据都是二维数组，其每行存储为B-G-R-B-G-R-B....故：
        for(int j=0;j<Mc.cols*Mc.channels();j++)
            cout<<(int)ptr[j]<<"\t";
        cout<<endl;
    }
    for (int y = 0; y < Mc.rows; y++)  //（2.2）ptr方法访问位于 i,j,k 处的像素(推荐)
    {
        unsigned char *row_ptr = Mc.ptr<unsigned char>(y);//用cv::Mat::ptr获得图像的行指针
        for (int x = 0; x < Mc.cols; x++) 
        {
            //unsigned char *data_ptr = &row_ptr[x * Mc.channels()]; 
            unsigned char *data_ptr = row_ptr+x * Mc.channels(); // 指向待访问的像素数据
            for (int c = 0; c != Mc.channels(); c++) // 输出该像素的每个通道
            {
                unsigned char data = data_ptr[c];    // data为I(x,y)第c个通道的值
                cout<<(int)data<<"\t";
            }
            cout<<endl;
        }
    }
    
    for(int i=0;i<Mc.rows;i++)//（3）直接读取i,j,k 地址处的像素
    {
        for(int j=0;j<Mc.cols;j++)
        {
            for(int k=0;k<Mc.channels();k++)
            {
                int Mi=(int)(*(Mc.data+Mc.step[0]*i+Mc.step[1]*j+k)); 
                cout<<Mi<<"\t";
            }
        }
        cout<<endl;
    }
    MatIterator_<uchar> it=Meye.begin<uchar>(); // （4）Mat迭代器访问
    MatIterator_<uchar> it_end=Meye.end<uchar>();
    for(int i=0;it!=it_end;it++)
    {
        cout<<(int)(*it)<<"\t";
        if((++i % Meye.cols)==0)
            cout<<endl;
    }
    //切片
    Mat Ma1=Ma.row(100),              //第i行数组
        Ma2=Ma.col(100),              //第j列数组
        Ma3=Ma.rowRange( 10, 20 ),    //第i0行到第i1-1行所构成的数组
        Ma4=Ma.colRange( 10, 20 ),    //第j0列到第j1-1列所构成的数组
        Ma5=Ma.diag( 0 ),             //偏移为d的对角线所组成的数组
        Ma6=Ma(Range(10,10),Range(20,20)),//从点(i0, j0)到点(i1-1, j1-1)所包含数据构成的数组
        Ma7=Ma(Rect(5,5,5,3));    //从点(5,5)到点(5+5-1,5+3-1)所包含数据构成数组
        cout<<Ma1.rows<<","<<Ma1.cols<<","<<Ma1.channels()<<endl;
        cout<<Ma3.rows<<","<<Ma3.cols<<","<<Ma3.channels()<<endl;
        cout<<Ma5.rows<<","<<Ma5.cols<<","<<Ma5.channels()<<endl;
        cout<<Ma7<<endl;
    //矩阵运算(必须保证参与运算的数据类型相同)及函数
    Mat m1=(Mat_<int>(3,3)<<1,2,1,4,-5,6,7,8,9),
        m2=(Mat_<int>(3,3)<<1,2,1,4,-5,6,7,8,9),
        m3=(Mat_<double>(3,3)<<5.0,2,1.0,4,9.0,5.2,1,3,9),
        m4=(Mat_<double>(3,2)<<1.0,2,3.0,4,5,6);
    cout<<"矩阵运算："<<endl<<m1+m2<<endl<<m1-m2<<endl<<m1/m2<<endl<<m3/2<<endl;
    cout<<m3*m4<<endl<<m3.dot(m3)<<endl<<m3.mul(m3)<<endl;  //矩阵乘法/点积/对应位乘
    cout<<m3.inv()<<endl; //求逆矩阵.inv(method-default DECOMP_LU),cv::DECOMP_ _LU,表示使用LU分解，这种方法对任意的非奇异矩阵都有效。第二种是cv::DECOMP_CHOLESKY,表示使用Cholesky分解Cholesky之在矩阵半正定的时候有效，而且在处理大型矩阵的时候比LU分解快很多。最后一种操作是cV::DECOMP_SVD, 使用奇异值分解(SVD)进行求逆。SVD是唯--一种在矩阵奇异或非方阵的情况下都可工作的方法(可以求出矩阵的伪逆)。
    /*m1 = m0.clone();从m0进行完全复制，该复制将复制所有的数据元素
    m0.copyTo(m1);将m0复制给m1，如果有必要，将给m1重分配内存空间(等同于m1=m0.clone())
    m0.copyTo(m1, mask);和m0.copyTo(m1)-样，但是只复制mask所指示的区域
    m0.convertTo(m1, type, scale, offset);转换m0中元素的类型(比如CV_ 32F)并且在尺度变换(默认为1)和增加偏置(默认为0)之后赋值给m1
    m0.assignTo( m1, type );只在内部使用(集成在convertTo中)
    m0.setTo( s, mask );设置m0所有元素为s，如果存在mask,则只对mask指示区域进行操作m0.reshape( chan,rows)改变二维数组的有效形状，chan和rows变量可能为0，表示做更改
    m0.push_back(s );在末尾增加一个m*1大小的数组
    m0.push_back( m1 );向m*n大小的矩阵m0增加k行并且复制到m1中，m1大小必须k*n
    m0.pop_back( n );从mx n大小的矩阵移除n行(默认是1) a
    mO.adjustROI( t, b, 1, r);通过t (最上)，b (最下)，1 (最左)，r (最右调整ROI范围)
    m0.total();计算数组序列的元素的数目(不包括通道)
    m0.type();返回m0元素的类型(比如CV_ _32FC3)
    m0. depth();返回mo通道中的元素类型(比如CV_ 32F)
    m0.size();以cv::Size返回m0的大小
    mO.empty();如果数组没有元素，将返回true (比如mo.total==0或者m0.data==NULL)
    */
    cout<<m1+1<<endl<<abs(m1)<<endl<<m1.t()<<endl;      //广播/绝对值/转置
    //还有更多的函数操作，见《学习OpenCV3》文档注释
    
    //（8）稀疏Mat类：
    int size[] = {10,10};
    SparseMat sm(2, size, CV_32F);
    for( int i=0; i<10; i++ ) // Fill the Matrix
    {
        int idx[2];
        idx[0] = size[0] * rand();
        idx[1] = size[1] * rand();
        sm. ref<float>( idx ) += 1.0f;
    }
    
    //（9）关键点类：KeyPoint
    
    // Test
    cv::Mat mat(5,2,CV_32F);
    for(int i=0; i<5; i++)
    {
        mat.at<float>(i,0)=i*2;
        mat.at<float>(i,1)=i*i;
    }
    cout<<mat<<endl;
    mat=mat.reshape(2);//查看改变通道数的变化
    cout<<mat<<endl;
    
    return 0;
}
