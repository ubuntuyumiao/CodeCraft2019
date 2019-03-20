#ifndef __INCLUDE__H__
#define __INCLUDE__H__



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
  still_stored, //还在车库
  wait_schedule, //某时刻在路上等待调度
  completed,     //某时刻已经调度完成
  reached        //到达终点
};
typedef enum sche_direct
{
  go_straight, 
  turn_left,
  turn_right
};
typedef struct how_tonext
{
  int next_road;
  sche_direct direct;
}how_tonext;
typedef struct Magic_garage
{
    int road_id;
    std::vector<int> garage[2];
}Magic_garage;
typedef struct Car
{
    int id;
    int set;
    int goal;
    int max_speed;
    int set_time;
    int cross_path[MAX_CROSS];
    int now_road;
    int next_road;
    bool wait_anthor;
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
    bool completed;
    
    //定义： load[0]存放 start-->end方向道路 load[1]存放 end->start方向道路
    int load[2][MAX_LANE][MAX_LANE_LENGHT];
}Road;

typedef struct Cross
{
    int id;
    int road_id[4];
    
    int prior_uproad;
    int prior_rightroad;
    int prior_downroad;
    int prior_leftroad;   //该段数据供所有通过这路口的车辆查阅 对比优先级
    
    
    int up_cross_id;
    int right_cross_id;
    int down_cross_id;
    int left_cross_id;

}Cross;
typedef struct sch_pos
{
    int  road_completed; 
    int  lane;    
    int  offset;       //车辆所在车位下标  
    int  next_road;
    bool  block;
}sch_pos; 

typedef struct road_space
{
    bool is_empty;  // 是否空标志
    int  lane;      // 非空的话优先级最高的有容量的车道
    int  offset;    // 非空的话优先级最高的有容量的车道 的倒数第几个车位
}road_space;

typedef struct drive_toroad
{
    bool no_drive;  
    int  lane;      
    int  offset;    
}drive_toroad;

typedef struct Global
{
    bool all_car_iscompleted;// 是否空标志
    bool car_inroad_iscompleted;    // 非空的话优先级最高的有容量的车道
}Global;






typedef struct pior_cross
{
  int road_id;
  int dir;
}pior_cross;
int  campare_dir(const void * a, const void * b);
int  campare_id(const void * a, const void * b);
void quickSort(Car* car_list,int car_begin,int car_end);
//路口优先级对比
int compare_prior_sch(Car* car_array_,Cross* cross_,Road* road_array,Road* road_,Cross *cross_array);


//路口调度 获得对应道路等待态优先级最高的车（不含已调度过 wait_another的车）
sch_pos sch_most_prior(Car *car_array_,Road* road_,Cross* cross_,int offset,
		       int cur_road[],Cross *cross_array_,Road* road_array_,Road map_[][MAX_CROSS],
		       std::vector<int> &wait_list_,std::vector<int> &block_list_,int T_);
//在调度车库时的检查  注意： 要与调度路口车辆区分
//功能：检查该车是否为该道路优先级最高的车
bool check_most_prior(int car_id,Road* road_,Cross* cross_);
//通过当前所在道路输出下一条道路 以及转向关系
how_tonext next_road(Car* car_,Road* cur_road,Cross* cross_array_,Road map_[][MAX_CROSS]);
// 将所有终止态的车改为等待态
void chang_completed_towait(int min_car_id_,int max_car_id_,Car *car_array_,std::vector<int>&wait_list_);
//检查该道路的车是否都已是终止态，且返回非终止态车数量
int* has_car_wait_inroad(Cross* cross_,Road* road_,Car *car_array_);
// 如果该道路在位置上最靠前 且为终止态 ，且下一个时刻即将过路口，将其信息发送到其下一个路口公告字段
void update_to_cross(Car* car_,Road* road_,Cross* cross_,Cross *cross_array_);
//检查此车是否在等待列表中出现
bool check_in_list(int car_id_,std::vector<int> &wait_list_);
//查找并删除
bool check_and_delete(int car_id,std::vector<int> &wait_list_);
//全局车辆的两种判断 1、所有车辆是否全是终止态 2、所有已上路车辆全是终止态 
Global All_car_iscompleted(Car *car_array,int min_car_id_,int max_car_id_);
//检查是否有车未到终点
bool All_car_isreached(Car* car_array,int min_car_id_,int max_car_id_);
//检查某道路是否为空 不为空 那最高优先级的有余量的车道是哪条 最优先车位的下标？  驶向路口
// road_empty check_road_empty(Cross *cur_cross_,Road *cur_road_);
// 为进入该道路的车辆提供数据   注意： 此次检查发生在调度道路的车库 所以 车道为驶离路口
road_space check_road_space(Cross *cur_cross_,Road *cur_road_);
//输入参数 当前路口  目标道路
drive_toroad check_road_drive_space(Cross *cur_cross_,Road *road_,int max_offset);
// 注意：发车与调度也不一样
void update_to_cross_drive(Car* car_,Road* road_,Cross* cross_);


void init_waitanthor(Car* car_array,int min_car_id_,int max_car_id_);
void print_time(const char * const head);
//Astart寻路
bool Astar_search(Car *car_,Road* road_array_,int min_road_id,int max_road_id,Cross* cross_array_,int min_cross_id,int max_cross_id,int (*weight_)[MAX_CROSS]);
int campare_settime(const void * a, const void * b);
//发车排序
void quickSortOfCpp(Car* car_list,int car_begin,int car_end);
bool cmp(int a,int b);
int not_equal(int a,int b);
int min(int a, int b);

void out(std::string s);

void debug_dir_leavecross(Road *road_array_,int min_cross_id,int max_cross_id,Cross *cross_array_);


void debug_dir_tocross(Road *road_array_,int min_cross_id,int max_cross_id,Cross *cross_array_);


bool sch_allcross_drive(Car* car_array,int min_car_id,int max_car_id,
			 Cross* cross_array,int min_cross_id,int max_cross_id,
			 Road* road_array,
			 Magic_garage* garage,
			 Road map_[][MAX_CROSS],
			 int T,
			 std::vector<int> &wait_list_,
			 std::vector<int> &bloack_list_
			);



void sch_allcross_garage(Car* car_array,
			 Cross* cross_array_,int min_cross_id,int max_cross_id,
			 Road* road_array,
			 Magic_garage* garage,
			 Road map_[][MAX_CROSS],
			 int T
			);
#endif

