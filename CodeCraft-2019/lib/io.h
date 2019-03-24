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
    A_star();
    ~A_star();
 
    void search(Car *car_,node* start,node* end,std::vector<int>&cross_dict_,
                 Cross* cross_array_,Road map_[][MAX_CROSS],int (*weight_)[MAX_CROSS]
    ); 
    void check(int current_cross_id_,node* father,int g,Cross* cross_array_,std::vector<int>&cross_dict_);
    void nextstep(Car *car_,node* current,Cross* cross_array_,
		  Road map_[][MAX_CROSS],int (*weight_)[MAX_CROSS],std::vector<int>&cross_dict_);
    int finding(vector<node*>* nodelist,int x,int y);
    int cross_tosub(int cross_id_,std::vector<int>&cross_dict_);
    static bool compare(node* n1,node* n2);
    void print(node* current);
    int min_cross_id=-1,max_cross_id=-1;
    int coord_max;
    int coord[MAX_CROSS][MAX_CROSS];
    std::stack<int> route_stack;
    bool find_path=false;
};

A_star::A_star()
{

}
A_star::~A_star()
{
}
int A_star::cross_tosub(int cross_id_,std::vector<int>&cross_dict_)
{
    vector<int>::iterator it = find(cross_dict_.begin(), cross_dict_.end(),cross_id_);
 
    if(it != cross_dict_.end())
    { return &*it-&cross_dict_[0]; }
    else 
        cout<<cross_id_<<" can not find"<<endl;
 
    return &*it-&cross_dict_[0];
}
void A_star::search(Car *car_,node* start,node* end,std::vector<int>&cross_dict_,
                    Cross* cross_array_,Road map_[][MAX_CROSS],int (*weight_)[MAX_CROSS]
)
{
    if(start->cross_id<cross_dict_[0]
      ||start->cross_id>cross_dict_[cross_dict_.size()-1]
      ||end->cross_id<cross_dict_[0]
      ||end->cross_id>cross_dict_[cross_dict_.size()-1])
    {
        std::cout << "io.h ERROR : no this cross id " << std::endl;
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
        nextstep(car_,current,cross_array_,map_,weight_,cross_dict_
	  );//用于遍历函数
        closelist.push_back(current);
        openlist.erase(openlist.begin());//一旦访问完就丢弃到close中
        sort(openlist.begin(),openlist.end(),compare);//根据f值从小到大排序
    }
}
//核心 怎么动态化网格权重
void A_star::nextstep(Car *car_,node* current,Cross* cross_array_,
		      Road map_[][MAX_CROSS],int (*weight_)[MAX_CROSS],std::vector<int>&cross_dict_)
{ 
  int way_cost;
  int cur_cross_sub=this->cross_tosub(current->cross_id,cross_dict_);
  int down_cross_sub=-1,up_cross_sub=-1,left_cross_sub=-1,right_cross_sub=-1;
  if(cross_array_[cur_cross_sub].down_cross_id!=-1)
      down_cross_sub=this->cross_tosub(cross_array_[cur_cross_sub].down_cross_id,cross_dict_);
  if(cross_array_[cur_cross_sub].up_cross_id!=-1)
      up_cross_sub=this->cross_tosub(cross_array_[cur_cross_sub].up_cross_id,cross_dict_);
  if(cross_array_[cur_cross_sub].left_cross_id!=-1)
      left_cross_sub=this->cross_tosub(cross_array_[cur_cross_sub].left_cross_id,cross_dict_);
  if(cross_array_[cur_cross_sub].right_cross_id!=-1)
      right_cross_sub=this->cross_tosub(cross_array_[cur_cross_sub].right_cross_id,cross_dict_);

  //下
if((down_cross_sub!=-1)&&(map_[cur_cross_sub][down_cross_sub].id!=-1))
{  
  
  way_cost = speed_near_w * abs(car_->max_speed-map_[cur_cross_sub][down_cross_sub].limit_speed) 
                              * abs(car_->max_speed-map_[cur_cross_sub][down_cross_sub].limit_speed) 
             +  space_plus_speed  *map_[cur_cross_sub][down_cross_sub].road_length *
                                    map_[cur_cross_sub][down_cross_sub].lane_num /
                                    map_[cur_cross_sub][down_cross_sub].limit_speed
             - weight_[cur_cross_sub][down_cross_sub];
  
  check(cross_array_[cur_cross_sub].down_cross_id,current,
	  way_cost,cross_array_,cross_dict_);
}
//上
if((up_cross_sub!=-1)&&(map_[cur_cross_sub][up_cross_sub].id!=-1)) 
{  

    way_cost = speed_near_w * abs(car_->max_speed-map_[cur_cross_sub][up_cross_sub].limit_speed) 
                              * abs(car_->max_speed-map_[cur_cross_sub][up_cross_sub].limit_speed) 
             +  space_plus_speed  *map_[cur_cross_sub][up_cross_sub].road_length *
                                    map_[cur_cross_sub][up_cross_sub].lane_num /
                                    map_[cur_cross_sub][up_cross_sub].limit_speed
             - weight_[cur_cross_sub][up_cross_sub];
  
  check(cross_array_[cur_cross_sub].up_cross_id,current,
	  way_cost,cross_array_,cross_dict_);
}
 //右  
if((right_cross_sub!=-1)&&(map_[cur_cross_sub][right_cross_sub].id!=-1))
{
   
    way_cost = speed_near_w * abs(car_->max_speed-map_[cur_cross_sub][right_cross_sub].limit_speed) 
                              * abs(car_->max_speed-map_[cur_cross_sub][right_cross_sub].limit_speed) 
             +  space_plus_speed  *map_[cur_cross_sub][right_cross_sub].road_length *
                                    map_[cur_cross_sub][right_cross_sub].lane_num /
                                    map_[cur_cross_sub][right_cross_sub].limit_speed
             - weight_[cur_cross_sub][right_cross_sub];
  
  check(cross_array_[cur_cross_sub].right_cross_id,current,
	  way_cost,cross_array_,cross_dict_);
}
//左
if((left_cross_sub!=-1)&&(map_[cur_cross_sub][left_cross_sub].id!=-1))
{
    way_cost = speed_near_w * abs(car_->max_speed-map_[cur_cross_sub][left_cross_sub].limit_speed) 
                              * abs(car_->max_speed-map_[cur_cross_sub][left_cross_sub].limit_speed) 
             +  space_plus_speed  *map_[cur_cross_sub][left_cross_sub].road_length *
                                    map_[cur_cross_sub][left_cross_sub].lane_num /
                                    map_[cur_cross_sub][left_cross_sub].limit_speed
             - weight_[cur_cross_sub][left_cross_sub];
  
  check(cross_array_[cur_cross_sub].left_cross_id,current,
	  way_cost,cross_array_,cross_dict_);
}
    
}
// g 为到该点的代价
void A_star::check(int cross_id_neighbor,node* father,int g,Cross* cross_array_,std::vector<int>&cross_dict_)
//用来访问节点
{
  //如果路口ID不在范围内直接返回
    int j=0,in_list=0;
    if(cross_id_neighbor<cross_dict_[0]
      ||cross_id_neighbor>cross_dict_[cross_dict_.size()-1]
      ||father->cross_id<cross_dict_[0]
      ||father->cross_id>cross_dict_[cross_dict_.size()-1]) 
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
	int p_sub=cross_tosub(p->cross_id,cross_dict_);
	int end_sub=cross_tosub(end->cross_id,cross_dict_);
	p->h=abs(cross_array_[p_sub].right_cross_id- cross_array_[end_sub].right_cross_id)*weightW; 

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