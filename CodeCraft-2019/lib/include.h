#ifndef __INCLUDE__H__
#define __INCLUDE__H__

#include <iostream>
#include <stdio.h>
#include <queue>
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
#define MAX_LANE_LENGHT 30

#define RED "\033[31m" /*red*/
#define YELLOW "\033[33m" /*Green*/
#define RESET "\033[0m"
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

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
    int now_road;
    int next_road;
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
    
    //定义： magic_garage[0]存放 start-->end方向车辆 magic_garage[1]存放 end->start方向车辆
    std::stack<int> magic_garage[2];
    
    //(双向或者单向)道路负载 以 lane_num×road_length数组表示 start-->end:0    end->start:1
    //定义： load[0]存放 start-->end方向道路 load[1]存放 end->start方向道路
    int load[2][MAX_LANE][MAX_LANE_LENGHT];
}Road;

typedef struct Cross
{
    int id;
    int road_id[4];
    
    int prior_up;
    int prior_right;
    int prior_down;
    int prior_left;   //该段数据供所有通过这路口的车辆查阅 对比优先级
    
    
    int up_cross_id;
    int right_cross_id;
    int down_cross_id;
    int left_cross_id;
    
    bool up_cro_to_me_isempty;
    bool right_cro_to_me_isempty;
    bool down_cro_to_me_isempty;
    bool left_cro_to_me_isempty;
    
}Cross;
typedef struct road_empty
{
    bool is_empty;// 是否空标志
    int  lane;    // 非空的话优先级最高的有容量的车道
    int  offset;    // 非空的话优先级最高的有容量的车道 的倒数第几个车位
}road_empty;

typedef struct MGraph
{
    int edges[MAX_ROAD][MAX_ROAD];//邻接矩阵，记录的是两点之间的距离，也就是权值 
    int n,e;//顶点数和边数
}MG;




//打印时间。入参为打印信息头
bool All_car_iscompleted(Car* car_array,int min_car_id_,int max_car_id_);
road_empty check_road_empty(Cross *cur_cross_,Road *cur_road_);
void print_time(const char * const head);
bool Astar_search(Car *car_,Road* road_array_,int min_road_id,int max_road_id,Cross* cross_array_,int min_cross_id,int max_cross_id);
int campare_settime(const void * a, const void * b);
void quickSortOfCpp(Car* car_list,int car_begin,int car_end);
bool cmp(int a,int b);
#endif

