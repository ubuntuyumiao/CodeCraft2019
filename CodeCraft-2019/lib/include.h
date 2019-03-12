#ifndef __INCLUDE__H__
#define __INCLUDE__H__

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <cstring>
#include <stack>
#include <vector>
#include<algorithm>
#define MAX_ROAD    7000
#define MAX_CAR     15000
#define MAX_CROSS   100
#define INF 0x3f3f3f3f

#define MAX_LANE 10
#define MAX_LANE_LENGHT 10

typedef enum drive_state
{
  still_stored, 
  wait_schedule,
  completed
};
typedef enum sche_direct
{
  straight, 
  left,
  right
};
typedef struct Car
{
    int id;
    int set;
    int goal;
    int max_speed;
    int set_time;
    int path[MAX_CROSS];
    drive_state state;  //调度状态 ： 等待出发态 等待调度态 终止态
    sche_direct move_ori;
}Car;
typedef struct Road
{
    int id;
    int road_length;
    int limit_speed;
    int lane_num;
    int start;
    int end;
    int flag_twoway;
    int car_on_road;
    //(双向或者单向)道路负载 以 lane_num×road_length数组表示 (start>end)?1:0
    int load[2][MAX_LANE][MAX_LANE];

    //定义： load[0]存放 从较小id路口通往较大路口id方向 load[1]存放 从较大id路口通往较小路口id方向
}Road;

typedef struct Cross
{
    int id;
    int road_id[4];
    int up_cross_id;
    int right_cross_id;
    int down_cross_id;
    int left_cross_id;
    
    bool up_cro_to_me_isempty;
    bool right_cro_to_me_isempty;
    bool down_cro_to_me_isempty;
    bool left_cro_to_me_isempty;
    std::stack<int> magic_garage;
}Cross;


typedef struct MGraph
{
    int edges[MAX_ROAD][MAX_ROAD];//邻接矩阵，记录的是两点之间的距离，也就是权值 
    int n,e;//顶点数和边数
}MG;




//打印时间。入参为打印信息头
void print_time(const char * const head);
bool Astar_search(Car *car_,Road* road,int min_road_id,int max_road_id,Cross* cross,int min_cross_id,int max_cross_id);
int campare_settime(const void * a, const void * b);
void quickSortOfCpp(Car* car_list,int car_begin,int car_end);
bool cmp(int a,int b);
#endif

