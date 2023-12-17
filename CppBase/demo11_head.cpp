#include "demo11_head.h"
using namespace std;

double avange(double arr[],int n)
{
    double sum=0;
    for (int i=0;i<n;i++)
        sum+=arr[i];
    
    return sum/n;
}

double sum(double arr[][3],int n)
{
    int m=sizeof(arr[0])/sizeof(arr[0][0]);
    double sum=0;
    for(int i=0;i<n;i++)
        for(int j=0;j<m;j++)
        {
            sum+=arr[i][j];
        }
    return sum;
}

point translation(point p,double trans[3])
{
    p.x=p.x+trans[0];
    p.y=p.y+trans[1];
    p.z=p.z+trans[2];
    
    return p;
}
