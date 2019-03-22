#ifndef __IO__H__
#define __IO__H__

#include<iostream>
#include <stdlib.h> // 对应于C++中cstdlib
#include <stdio.h>
#include <time.h> // ctime
#include<vector>
#include <sys/timeb.h>
#include<algorithm>
#include<cmath>
#include<string.h>
#include "include.h"


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
 
int weightW=8;//直线代价


class A_star
{
public: 
    node *start;
    node *end; 
    vector<node*> openlist;//open表，存遍历到的节点
    vector<node*> closelist;//close表，存访问过的节点
    A_star(Road* road_array,int minroad_id,int maxroad_id,Cross* cross_array,
	   int mincross_id,int maxcross_id);
    ~A_star();
 
    void search(Car *car_,node* start,node* end,
                 Cross* cross_array_,Road map_[][MAX_CROSS],int (*weight_)[MAX_CROSS]
    ); 
    void check(int current_cross_id_,node* father,int g,Cross* cross_array_);
    void nextstep(Car *car_,node* current,Cross* cross_array_,Road map_[][MAX_CROSS],int (*weight_)[MAX_CROSS]);
    int finding(vector<node*>* nodelist,int x,int y);
    int cal_dis(int cross_from,int cross_to);
    static bool compare(node* n1,node* n2);
    void print(node* current);
    int min_cross_id=-1,max_cross_id=-1;
    int coord_max;
    int coord[MAX_CROSS][MAX_CROSS];
    std::stack<int> route_stack;
    bool find_path=false;
};
int A_star::cal_dis(int cross_from,int cross_to)
{
  int cr1[2]={0,0},cr2[2]={0,0};
  for(int i=1;i<=coord_max;i++)
    for(int j=1;j<=coord_max;j++)
    {
       if(coord[i][j]==cross_from)   {  cr1[0]=i; cr1[1]=j ; }
        else if(coord[i][j]==cross_to)   {   cr2[0]=i; cr2[1]=j ; }

        if((cr1[0]!=0)&&(cr2[0]!=0))  { return  (cr2[0]-cr1[0])*(cr2[0]-cr1[0])+(cr2[1]-cr1[1])*(cr2[1]-cr1[1]);  }
    }
    
    return init_weight;
}
A_star::A_star(Road* road_array,int minroad_id,int maxroad_id,Cross* cross_array,
	       int mincross_id,int maxcross_id)
{
  min_cross_id=mincross_id;
  max_cross_id=maxcross_id;
   
   coord_max=sqrt(maxcross_id-mincross_id+1);
   int id=0;
  for(int i=1;i<=coord_max;i++)
    for(int j=1;j<=coord_max;j++)
    {
      id++;
      coord[i][j]=id;
    }
}
A_star::~A_star()
{
}
void A_star::search(Car *car_,node* start,node* end,
                    Cross* cross_array_,Road map_[][MAX_CROSS],int (*weight_)[MAX_CROSS]
)
{
    if(start->cross_id<min_cross_id
      ||start->cross_id>max_cross_id
      ||end->cross_id<min_cross_id
      ||end->cross_id>max_cross_id)
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
        nextstep(car_,current,cross_array_,map_,weight_
	  );//用于遍历函数
        closelist.push_back(current);
        openlist.erase(openlist.begin());//一旦访问完就丢弃到close中
        sort(openlist.begin(),openlist.end(),compare);//根据f值从小到大排序
    }
}
//核心 怎么动态化网格权重
void A_star::nextstep(Car *car_,node* current,Cross* cross_array_,Road map_[][MAX_CROSS],int (*weight_)[MAX_CROSS])
{ 
  int way_cost;
  //下
if(map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].id!=-1)  
{  
  way_cost = speed_near_w * abs(car_->max_speed-map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].limit_speed) 
                              * abs(car_->max_speed-map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].limit_speed) 
             +  space_plus_speed  *map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].road_length *
                                    map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].lane_num /
                                    map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].limit_speed
             - weight_[current->cross_id][cross_array_[current->cross_id].down_cross_id];
  
  check(cross_array_[current->cross_id].down_cross_id,current,
	  way_cost,cross_array_);
}
//上
if(map_[current->cross_id][cross_array_[current->cross_id].up_cross_id].id!=-1)  
{  
    way_cost = init_weight+ speed_near_w * abs(car_->max_speed-map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].limit_speed)  
                                            * abs(car_->max_speed-map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].limit_speed) 
             +  space_plus_speed  *map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].road_length *
                                    map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].lane_num /
                                    map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].limit_speed;
	    - weight_[current->cross_id][cross_array_[current->cross_id].down_cross_id];
    check(cross_array_[current->cross_id].up_cross_id,current,
	  way_cost,cross_array_);
}
 //右  
if(map_[current->cross_id][cross_array_[current->cross_id].right_cross_id].id!=-1) 
{
    way_cost =init_weight+ speed_near_w * abs(car_->max_speed-map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].limit_speed)  
                                            * abs(car_->max_speed-map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].limit_speed) 
             +  space_plus_speed  *map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].road_length *
                                    map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].lane_num /
                                    map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].limit_speed;
	    - weight_[current->cross_id][cross_array_[current->cross_id].down_cross_id];
    check(cross_array_[current->cross_id].right_cross_id,current,
	 way_cost,cross_array_);
}
//左
if(map_[current->cross_id][cross_array_[current->cross_id].left_cross_id].id!=-1)
{
    way_cost =init_weight+ speed_near_w * abs(car_->max_speed-map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].limit_speed)  
                                           * abs(car_->max_speed-map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].limit_speed) 
             +  space_plus_speed  *map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].road_length *
                                    map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].lane_num /
                                    map_[current->cross_id][cross_array_[current->cross_id].down_cross_id].limit_speed;
	     - weight_[current->cross_id][cross_array_[current->cross_id].down_cross_id];
    check(cross_array_[current->cross_id].left_cross_id,current,
	  way_cost,cross_array_);
}
    
}
// g 为到该点的代价
void A_star::check(int cross_id_neighbor,node* father,int g,Cross* cross_array_)
//用来访问节点
{
  //如果路口ID不在范围内直接返回
    int j=0,in_list=0;
    if(cross_id_neighbor<min_cross_id
      ||cross_id_neighbor>max_cross_id
      ||father->cross_id<min_cross_id
      ||father->cross_id>max_cross_id) 
                return ;
  
    for(size_t i=0;i<closelist.size();i++)
    {
        if(closelist.at(i)->cross_id ==cross_id_neighbor)
        {
	    in_list=1;
            break;
        }
    }
   if(in_list==1) return ; 
    in_list=0;
   
   for(size_t i=0;i<openlist.size();i++)
        if(openlist.at(i)->cross_id==cross_id_neighbor)
        {
	    j=i;   in_list=1;
            break;
        }
    
    if(in_list==1)
  //如果在open表中的，则需比较f值，如果当前的f值小，就替换成当前的节点
    {
        node* p=openlist[j];
        if(father->f + g < p->f )  
	 {
            p->father=father;
            p->g=father->g+g;
            p->f=p->g+p->h;
         }
    }
    else//如果两个表都不在就计算一下f值，存到open表中
    {
        node* p=new node(cross_id_neighbor,father);
// 	p->h=abs(cross_array_[p->cross_id].right_cross_id- cross_array_[end->cross_id].right_cross_id)*weightW; 
	p->h= Astar_h_w*cal_dis(cross_array_[p->cross_id].id,cross_array_[end->cross_id].id);
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