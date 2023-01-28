#include "math.h"
#include "ssd1306.h"

#define PI 3.1415926 
#define SIZE 20 //定义显示高度   

float cube[8][3]={{-SIZE,-SIZE,-SIZE},{-SIZE,SIZE,-SIZE},{SIZE,SIZE,-SIZE},{SIZE,-SIZE,-SIZE},{-SIZE,-SIZE,SIZE},{-SIZE,SIZE,SIZE},{SIZE,SIZE,SIZE},{SIZE,-SIZE,SIZE}};//立方体各点坐标
//float cube[8][3]={{-20,-20,-SIZE},{-SIZE,SIZE,-SIZE},{2.23,2.23,-SIZE},{SIZE,-SIZE,-SIZE},{-20,-20,SIZE},{-SIZE,SIZE,SIZE},{2.23,2.23,SIZE},{SIZE,-SIZE,SIZE}};//立方体各点坐标
int lineid[]={1,2,2,3,3,4,4,1,5,6,6,7,7,8,8,5,8,4,7,3,6,2,5,1};//记录点之间连接顺序

float* matconv(float* a,float b[3][3]){//计算矩阵乘法
  float res[3];
  for(int i=0;i<3;i++)
    res[i]=b[i][0]*a[0]+b[i][1]*a[1]+b[i][2]*a[2];
  for(int i=0;i<3;i++)a[i]=res[i];
  return a;
}

void rotate(float* obj,float x,float y,float z){//旋转该向量
  x/=PI;y/=PI;z/=PI;
  float rz[3][3]={{cos(z),-sin(z),0},{sin(z),cos(z),0},{0,0,1}};
  float ry[3][3]={{1,0,0},{0,cos(y),-sin(y)},{0,sin(y),cos(y)}};
  float rx[3][3]={{cos(x),0,sin(x)},{0,1,0},{-sin(x),0,cos(x)}};
  matconv(matconv(matconv(obj,rz),ry),rx);
}
  
void cube_loop() { 
  
  for(int i=0;i<8;i++)rotate(cube[i],0.5f,0.3f,0);//旋转每个点
  for(int i=0;i<24;i+=2){//绘制立方体
    SSD1306_DrawLine(64+cube[lineid[i]-1][0],32+cube[lineid[i]-1][1],64+cube[lineid[i+1]-1][0],32+cube[lineid[i+1]-1][1],SSD1306_COLOR_WHITE);
  }
}  
