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
#include <unistd.h>
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
 
int weightW=10;//直线代价


class A_star
{
public: 
    node *start;
    node *end; 
    vector<node*> openlist;//open表，存遍历到的节点
    vector<node*> closelist;//close表，存访问过的节点
    A_star(Car *car_,Road* road_array,int minroad_id,int maxroad_id,Cross* cross_array,
	   int mincross_id,int maxcross_id,int (*weight_)[MAX_CROSS],Road map_[][MAX_CROSS]);
    ~A_star();
 
    void search(node* start,node* end);
    void check(int current_cross_id_,node* father,int g);
    void nextstep(node* current);
    int finding(vector<node*>* nodelist,int x,int y);
    static bool compare(node* n1,node* n2);
    void print(node* current);
    Road change_weight();
    int min_cross_id=-1,max_cross_id=-1;
    Road road[MAX_ROAD];
    Cross cross[MAX_CROSS];
    std::stack<int> route_stack;
    bool find_path=false;
    int weight_net[MAX_CROSS][MAX_CROSS];
};

A_star::A_star(Car *car_,Road* road_array,int minroad_id,int maxroad_id,Cross* cross_array,
	       int mincross_id,int maxcross_id,int (*weight_)[MAX_CROSS],Road map_[][MAX_CROSS])
{
  min_cross_id=mincross_id;
  max_cross_id=maxcross_id;
  memcpy(weight_net,weight_,sizeof(weight_net));
  for(int i =mincross_id;i<max_cross_id+1;i++)
    memcpy(&cross[i],&cross_array[i],sizeof(Cross));
  for(int i =minroad_id;i<maxroad_id+1;i++)
    memcpy(&road[i],&road_array[i],sizeof(Road));
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
Road A_star::change_weight()
{
  
}
//核心 怎么动态化网格权重
void A_star::nextstep(node* current)
{ 
    change_weight();
    check(cross[current->cross_id].down_cross_id,current,weight_net[cross[current->cross_id].down_cross_id][current->cross_id]);
    //下
    check(cross[current->cross_id].up_cross_id,current,weight_net[cross[current->cross_id].up_cross_id][current->cross_id]);
    //上
    check(cross[current->cross_id].right_cross_id,current,weight_net[cross[current->cross_id].right_cross_id][current->cross_id]);
    //右
    check(cross[current->cross_id].left_cross_id,current,weight_net[cross[current->cross_id].left_cross_id][current->cross_id]);
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
bool read_file(char** argv, Cross *cross_array_,Cross *cross_sortedarray_,int* min_cross_id_,int* max_cross_id_,
	        Road *road_array_,Road *road_sortedarray_,int* min_road_id_,int* max_road_id,
	       Car *car_array_,Car *car_sortedarray_,int* min_car_id_,int* max_car_id_,Road map_[][MAX_CROSS])
{
  
  
  
   std::ifstream fin_car(argv[3]);
    std::ifstream fin_road(argv[2]);
    std::ifstream fin_cross(argv[1]);
    
    if(!fin_road.is_open())  {std::cout<<"/**ERROR:No Road input file**/"<<std::endl;return false;} 
    if(!fin_car.is_open())   {std::cout<<"/**ERROR:No Car input file**/"<<std::endl;return false;}
    if(!fin_car.is_open())   {std::cout<<"/**ERROR:No Cross input file**/"<<std::endl;return false;}
 
    int count=1;

    //设置标志位 将文件中的ID号作为数组下标 例如car[5000] 即为编号为5000 的车子信息 道路等同
    bool first_line=true;
    int Subscript=0;
    
    std::cout<<"read start"<<std::endl;
    while(!fin_road.eof())
      {
       char str_tr[50];
       fin_road.getline(str_tr,'\r\n');
       if(str_tr[0]=='#')  continue;
        else if(str_tr[0]=='(')
         {
	   
	    std::sscanf(str_tr,"(%d,%d,%d,%d,%d,%d,%d)",   &(road_array_[Subscript].id),
						    &(road_array_[Subscript].road_length),
						    &(road_array_[Subscript].limit_speed),
						    &(road_array_[Subscript].lane_num),
						    &(road_array_[Subscript].start),
						    &(road_array_[Subscript].end),
						    &(road_array_[Subscript].flag_twoway));
	  if(first_line) 
	  {
	    first_line=false; Subscript=road_array_[Subscript].id;
	    *min_road_id_= Subscript;
	    road_sortedarray_[Subscript] = road_array_[Subscript]=road_array_[0];
	  }
	road_array_[Subscript].completed=false;
	road_sortedarray_[Subscript] = road_array_[Subscript];
	Subscript++;
	count++;
         }
       }
       fin_road.close();
       *max_road_id = Subscript-1;
       count=1;
       Subscript=0;
       first_line=true;
    while(!fin_car.eof())
      {
       char str_tc[50];
       fin_car.getline(str_tc,'\r\n');
       if(str_tc[0]=='#')  continue;
        else if(str_tc[0]=='(')
         {
	   
	   std::sscanf(str_tc,"(%d,%d,%d,%d,%d)",   &(car_array_[Subscript].id),
						    &(car_array_[Subscript].set),
						    &(car_array_[Subscript].goal),
						    &(car_array_[Subscript].max_speed),
						    &(car_array_[Subscript].set_time));
	  if(first_line) 
	  {
	    first_line=false; Subscript=car_array_[Subscript].id;
	    *min_car_id_ = Subscript;
	    car_sortedarray_[Subscript] = car_array_[Subscript] = car_array_[0];
	  } 
	car_sortedarray_[Subscript] = car_array_[Subscript];
	car_array_[Subscript].state = still_stored;
	car_array_[Subscript].wait_anthor =false;
	car_array_[Subscript].now_road= car_array_[Subscript].next_road =-1;
	Subscript++;
	count++;
         }
       }
       fin_car.close();
       *max_car_id_= Subscript-1;
       count=1;
       Subscript=0;
       first_line=true;
    while(!fin_cross.eof())
      {
       char str_ts[50];
       fin_cross.getline(str_ts,'\r\n');
       if(str_ts[0]=='#')  continue;
        else if(str_ts[0]=='(')
         {
	   
	   std::sscanf(str_ts,"(%d,%d,%d,%d,%d)",   &(cross_array_[Subscript].id),
						    &(cross_array_[Subscript].road_id[0]),
						    &(cross_array_[Subscript].road_id[1]),
						    &(cross_array_[Subscript].road_id[2]),
						    &(cross_array_[Subscript].road_id[3]));
	  if(first_line) 
	  {
	    first_line=false; Subscript=cross_array_[Subscript].id;
	    *min_cross_id_ = Subscript;
	    cross_sortedarray_[Subscript] =cross_array_[Subscript] = cross_array_[0];
	  } 
	cross_sortedarray_[Subscript] =cross_array_[Subscript];
	Subscript++;
	count++;
         }
       }
       fin_cross.close();
       *max_cross_id_ = Subscript-1;
       std::cout<<"read finished"<<std::endl;
       
       printf("road_num: %d  car_num: %d  cross_num: %d \n",
	      *max_road_id-*min_road_id_+1,*max_car_id_-*min_car_id_+1,*max_cross_id_-*min_cross_id_+1);
      
       //初始化地图 将所有路口连接置为-1 表示无连接 共有1～(cross_num-1) 个路口
       //数组以0下表开始 直观起见 map[i][j]直接指第i个路口到第j个路口信息
       
       for(int i=*min_cross_id_;i<=*max_cross_id_;i++)
       {
	 //初始化路口的十字连通关系
	 cross_array_[*min_cross_id_+i-1].up_cross_id = 
	    cross_array_[*min_cross_id_+i-1].right_cross_id =
	      cross_array_[*min_cross_id_+i-1].down_cross_id =
		cross_array_[*min_cross_id_+i-1].left_cross_id=-1;
	 for(int j=*min_cross_id_;j<=*max_cross_id_;j++)         
	    map_[i][j].id= -1;
       }
       return true;
}

#endif