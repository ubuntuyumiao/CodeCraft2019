#ifndef __IO__H__
#define __IO__H__

#include<iostream>
#include <fstream>
#include <stdlib.h> // 对应于C++中cstdlib
#include <stdio.h>
#include <time.h> // ctime
#include <stdio.h>
#include<vector>
#include <sys/timeb.h>
#include<algorithm>
#include<cmath>
#include<string.h>
#include "include.h"
#define RED "\033[31m" /*red*/
#define YELLOW "\033[33m" /*Green*/
#define RESET "\033[0m"
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

using namespace std;

typedef struct node
{
    int cross_id; 
    //坐标
    int g;
    //g函数到起点的真实距离
    int h;
    //h函数到终点的估计距离
    int f;
    //f=h+g
    node* father;
    //父节点
    node(int cross_id)   
    //struct的构造函数
    {
        this->cross_id=cross_id;
        this->g=0;
        this->f=0;
        this->father=NULL;
    }
    node(int cross_id,node* father)
    {
        this->cross_id=cross_id;
        this->g=0;
        this->f=0;
        this->father=father;
    }
}node;
 
int weightW=10;//直线代价


class A_star
{
public: 
    node *start;
    node *end; 
    vector<node*> openlist;//open表，存遍历到的节点
    vector<node*> closelist;//close表，存访问过的节点
    A_star(Road* road_,int minroad_id,int maxroad_id,Cross* cross_,int mincross_id,int maxcross_id);
    ~A_star();
 
    void search(node* start,node* end);
    void check(int current_cross_id_,node* father,int g);
    void nextstep(node* current);
    int finding(vector<node*>* nodelist,int x,int y);
    static bool compare(node* n1,node* n2);
    void print(node* current);
    int min_cross_id=-1,max_cross_id=-1;
    Road road[MAX_ROAD];
    Cross cross[MAX_CROSS];
    std::stack<int> route_stack;
    bool find_path=false;
};

A_star::A_star(Road* road_,int minroad_id,int maxroad_id,Cross* cross_,int mincross_id,int maxcross_id)
{
  min_cross_id=mincross_id;
  max_cross_id=maxcross_id;
  for(int i =mincross_id;i<max_cross_id+1;i++)
    memcpy(&cross[i],&cross_[i],sizeof(Cross));
  for(int i =minroad_id;i<maxroad_id+1;i++)
    memcpy(&road[i],&road_[i],sizeof(Road));

}
A_star::~A_star()
{
}
void A_star::search(node* start,node* end)
{
    if(start->cross_id<min_cross_id||start->cross_id>max_cross_id||end->cross_id<min_cross_id||end->cross_id>max_cross_id)
    {
//         std::cout << "io.h ERROR : no this cross id " << std::endl;
        return;//如果路口ID不在范围内直接返回
    }
    node* current;
    this->start=start;
    this->end=end;
    
    openlist.push_back(start);//将起点放到open里
    while(openlist.size()>0)
    {
        current=openlist[0];
        if(current->cross_id==end->cross_id)//如果找到终点
        {
//             cout<<"find the path"<<endl;
	    find_path=true;
            print(current);
            openlist.clear();
            closelist.clear();
            break;
        }
        nextstep(current);//用于遍历函数
        closelist.push_back(current);
        openlist.erase(openlist.begin());//一旦访问完就丢弃到close中
        sort(openlist.begin(),openlist.end(),compare);//根据f值从小到大排序
    }
}
void A_star::nextstep(node* current)
{ 
    check(cross[current->cross_id].down_cross_id,current,weightW);
    //下
    check(cross[current->cross_id].up_cross_id,current,weightW);
    //上
    check(cross[current->cross_id].right_cross_id,current,weightW);
    //右
    check(cross[current->cross_id].left_cross_id,current,weightW);
    //左
}
void A_star::check(int cross_id_neighbor,node* father,int g)
//用来访问节点
{
    int j=0,in_list=0;
    if(cross_id_neighbor<min_cross_id||cross_id_neighbor>max_cross_id||father->cross_id<min_cross_id||father->cross_id>max_cross_id) //如果路口ID不在范围内直接返回
    { 
//       cout << "cross_id_neighbor: "<<cross_id_neighbor << " cross_id_neighbor: "<<father->cross_id <<std::endl;
//        std::cout << "io.h ERROR : no this cross id " << std::endl; 
      return ;
    }
//     if(map1[x][y]==1)//如果是障碍物，也直接返回
//         return ;
  
    for(size_t i=0;i<closelist.size();i++)
    {
        if(closelist.at(i)->cross_id ==cross_id_neighbor)
        {
	    in_list=1;
            break;
        }
    }
   if(in_list==1)
      //如果是close表中的，也直接返回
        { //std::cout << "io.h Output : this cross is in closelist " << std::endl; 
	  return ; }
    in_list=0;
   
       for(size_t i=0;i<openlist.size();i++)
    {
        if(openlist.at(i)->cross_id==cross_id_neighbor)
        {
	    j=i;
	    in_list=1;
            break;
        }
    }
    
    if(in_list==1)
  //如果在open表中的，则需比较f值，如果当前的f值小，就替换成当前的节点
    {
        node* p=openlist[j];
        if(father->f + g < p->f )   //!!!!!!!!!!!!!!!!!!!!!!!!!此处g 要做处理
	  //这个我用了估算，最精确的是把当前节点的f值算出来和在open表中的拿来比较，但是算f值比较麻烦，我这里就用father->f+g来代替当前节点的f
        {
            p->father=father;
            p->g=father->g+g;
            p->f=p->g+p->h;
        }
    }
    //!!!!!!!!!!!!!!!!!!!!!!!!!此处 H 要做处理
    else//如果两个表都不在就计算一下f值，存到open表中
    {
        node* p=new node(cross_id_neighbor,father);
//         p->h=abs(p->x- end->x)*weightW+abs(p->y-end->y)*weightW; 
	p->h=abs(cross[p->cross_id].right_cross_id- cross[end->cross_id].right_cross_id)*weightW; 
        p->g=p->father->g+g;
        p->f=p->g+p->h;
        openlist.push_back(p);
    }
}

bool A_star::compare(node* n1,node* n2)
{
    return n1->f < n2->f;
}
void A_star::print(node* current)//将路径制为7
{
    while(current!=NULL)
    {
        route_stack.push(current->cross_id);  //压入栈中
        current=current->father;
    }
}


#endif