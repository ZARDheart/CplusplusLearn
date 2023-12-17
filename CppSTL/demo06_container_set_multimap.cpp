#include <iostream>
#include <cstring>
#include <vector>
#include <array>
#include <map>
#include <list>
using namespace std;

void printdoublearray(vector<double> v,int n); // 打印vector
struct Info { 
    string name; 
    int num; 
    Info(){}
    Info(string name, int num):name(name),num(num){}
}; 

int main()
{
    //vector
    /* 
     * 可以把它看成一个一维数组，只是这个数组中可以存储任何元素，比如用户创建的类
    empty()://判断当前向量容器是否为空
    size()://返回当前向量容器中的实际元素个数
    []://返回指定下标的元素
    reserve(int n)://为当前向量容器预分配n个元素的存储空间
    capacity()://返回当前向量容器在重新进行内存分配以前所能容纳的元素个数
    resize(int n)://调整当前向量容器的大小,使其能容纳n个元素
    resize(int n,elem); //若容器变长，则以elem填充新位置
    assign(v.begin(),v.end()); //将[v.begin(),v.end()]区间中的元素赋值给本身
    assign(int n,elem); //将n个elem赋值给本身
    push_back(item)://在当前向量容器尾部添加一个元素
    insert(iterator pos,elem)://将元素elem插入到迭代器pos指定的元素之前
    insert(iterator pos,int n,elem)://将元素n个elem插入到迭代器pos指定的元素之前
    front()://获取当前向量容器的第一个元素
    back()://获取当前向量容器的最后一个元素
    swap(v); //容器v和当前容器互换
    erase(const_iterator pos); //删除迭代器指向的元素
    erase(const_iterator begin,const_iterator end); //删除迭代器从begin到end之间的元素clear()://删除当前向量容器中的所有元素
    begin()://该函数的两个版本分别返回iterator或者const_iterator,容器的第一个元素
    end()://两个版本分别返回iterator或者const_iterator,引用容器的最后一个元素后面的一个位置
    rbegin()://两个版本分别返回reverse_iterator或者const_reverse_iterator,引用容器的最后一个元素
    rend()://两个版本分别返回reverse_iterator或者const_reverse_iterator,引用容器的第一个元素前面的第一个位置 */
    double arr2[]={0.1,0.2,0.3,4.5};
    vector<int> vd(3);
    vd[2]=15;
    vector<double> vd1(10,2);                   //全部元素为2的向量
    vector<int> vd2(vd);                        //复制vd
    vector<int> vd3(vd1.begin(),vd1.begin()+3); //切片定义
    vector<double> vd4(arr2,arr2+4);            //使用数组切片
    //vector对象的操作
    cout <<"vector："<<endl<< vd[2] << "  "<< vd1[2]<< "  "<< vd2[2]<< "  "<< vd3[2]<< endl;
    printdoublearray(vd4,vd4.size());           //size元素个数
    vd1.assign(vd4.begin(),vd4.begin()+3);      //将vd4的0-2元素构成的向量赋给vd1
    cout<<vd1[1]<< "  "<< vd1.back()<< "  "<< vd1.front()<<endl; //返回尾首元素
    vd1.push_back(5.0);                         //末尾插入元素5.0
    printdoublearray(vd1,vd1.size());
    vd1.pop_back();                             //删除末尾元素
    printdoublearray(vd1,vd1.size());
    vd4.clear();                                //清空元素
    cout<<vd4.empty()<<endl;                    //判断是否为空，空则返回ture,不空则返回false
    vd1.insert(vd1.begin()+1,2.0);              //在第1个元素（从第0个算起）的位置插入数值2.0，如为1,2,3,4，插入元素后为1,2.0,2,3,4
    vd1.insert(vd1.begin()+1,3,5.0);            //在a的第1个元素（从第0个算起）的位置插入3个数，其值都为5.0
    printdoublearray(vd1,vd1.size());
    vd1.resize(10);                    //将现有元素个数调至10个，多则删，少则补，其值随机
    vd4.swap(vd1);                     //将两向量中的元素进行整体性交换
    printdoublearray(vd1,vd1.size());
    printdoublearray(vd4,vd4.size());
    vector<double> vn=vector<double> (5,0.123);// 创建长度为5的全为0.123的vector
    printdoublearray(vn,vn.size());
    
    //array
    array<double,4> a1={3.14,5.2,6.9594,1.234556789};
    array<double,4> a2;
    a2=a1;                         //array对象可复制，而数组只能复制元素
    cout<<a1[3]<<"  "<<a2[3]<<endl;
    
    //map
    /* 
     * Map是STL的一个关联容器，它提供一对一（其中第一个可以称为关键字，每个关键字只能在map中出现一次，第二个可能称为该关键字的值，key和value可以是任意你需要的类型）的数据处理能力，由于这个特性，它完成有可能在我们处理一对一数据的时候，在编程上提供快速通道。
    map对象是模板类，需要关键字和存储对象两个模板参数：
    std:map<Anytype,Anytype> personnel;
    使用迭代器访问时，iter->first指向元素的键，iter->second指向键对应的值。 使用下标访问map容器与使用下标访问vector的行为截然不同：用下标访问map中不存在的元素将导致在map容器中添加一个新的元素，这个元素的键即为该下标值，键所对应的值为空。
    begin() 返回指向 map 头部的迭代器
    clear(） 删除所有元素
    count() 返回指定元素出现的次数
    empty() 如果 map 为空则返回 true
    end() 返回指向 map 末尾的迭代器
    erase() 删除一个元素
    find() 查找一个元素
    insert() 插入元素
    key_comp() 返回比较元素 key 的函数
    lower_bound() 返回键值>=给定元素的第一个位置
    max_size() 返回可以容纳的最大元素个数
    rbegin() 返回一个指向 map 尾部的逆向迭代器
    rend() 返回一个指向 map 头部的逆向迭代器
    size() 返回 map 中元素的个数
    swap() 交换两个 map
    upper_bound() 返回键值>给定元素的第一个位置
    value_comp() 返回比较元素 value 的函数*/
    map<Info*,int> stus;
    for(int i=1;i<=5;i++)
    {
        Info *stu;
        stu = new Info("stu",5);
        printf("%p\n",stu);
        stu->name="stu"+to_string(i);
        stu->num=220+i;
        int score=90+i;
        stus[stu]=score;
        //stus.insert(pair<Info*,int>(&stu,score));
    }
    map<Info*,int>::iterator it;
    for(it=stus.begin();it!=stus.end();it++)
    {
        Info *stu=it->first;
        int score=it->second;
        cout<<(*stu).name<<" "<<(*stu).num<<" "<<score<<endl;
    }
    
    //list
    /* list() 声明一个空列表；
    list(n) 声明一个有n个元素的列表，每个元素都是由其默认构造函数T()构造出来的
    list(n,val) 声明一个由n个元素的列表，每个元素都是由其复制构造函数T(val)得来的
    list(n,val) 声明一个和上面一样的列表
    list(first,last) 声明一个列表，其元素的初始值来源于由区间所指定的序列中的元素
    begin()和end()：通过调用list容器的成员函数begin()得到一个指向容器起始位置的iterator，可以调用list容器的 end() 函数来得到list末端下一位置
    push_back() 和push_front()：使用list的成员函数push_back和push_front插入一个元素到list中。其中push_back()从list的末端插入，而 push_front()实现的从list的头部插入。
    empty()：利用empty() 判断list是否为空。
    resize()：如果调用resize(n)将list的长度改为只容纳n个元素，超出的元素将被删除，如果需要扩展那么调用默认构造函数T()将元素加到list末端。如果调用resize(n,val)，则扩展元素要调用构造函数T(val)函数进行元素构造，其余部分相同。
    clear()：清空list中的所有元素。
    front()和back()： 通过front()可以获得list容器中的头部元素，通过back()可以获得list容器的最后一个元素。
    pop_back和pop_front()：通过删除最后一个元素，通过pop_front()删除第一个元素；序列必须不为空，如果当list为空的时候调用pop_back()和pop_front()会使程序崩掉。
    assign()：具体和vector中的操作类似
    swap()：交换两个链表(两个重载)，一个是l1.swap(l2); 另外一个是swap(l1,l2)
    reverse()：通过reverse()完成list的逆置。
    merge()：合并两个链表并使之默认升序(也可改)*/
    list<int> list1;
    for (int k=0;k<10;k++){
        list1.push_back(k);
    }
    for (int k=0;k<10;k++){
        list1.insert(list1.end(), k);
    }
    list<int>::iterator list_iter1;
    for (list_iter1 = list1.begin();list_iter1 != list1.end();++list_iter1){
        cout << *list_iter1 << " ";
    }
    cout << endl;
    
    //iterator 
    vector<int> vi;
    for(int i=0;i<=5;i++)
        vi.push_back(i);
    vector <int>:: iterator iter;
    for(int i=0;i<=5;i++)
        cout<<vi[i]<<" ";
    cout<<endl;
    for(iter=vi.begin();iter!=vi.end();iter++)
        cout<<*iter<<" ";
    cout<<endl;

    return 0;
    
}

void printdoublearray(vector<double> v,int n)
{
    if (n>0)
    {
        for(int i=0;i<n;i++)
            cout<<v[i]<<"\t";
        cout<<endl;
    }
    else
    {
        cout<<"Vector is empty!"<<endl;
    }
}

