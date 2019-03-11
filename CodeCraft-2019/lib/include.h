#ifndef __INCLUDE__H__
#define __INCLUDE__H__

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <cstring>
#include <stack>
#include <vector>

#define MAX_ROAD    10000
#define MAX_CAR     20000
#define MAX_CROSS   5000
#define INF 0x3f3f3f3f

typedef struct Road
{
    int id;
    int road_length;
    int limit_speed;
    int lane_num;
    int start;
    int end;
    int flag_twoway;
}Road;

typedef struct Car
{
    int id;
    int set;
    int goal;
    int max_speed;
    int set_time;
}Car;


typedef struct Cross
{
    int id;
    int road_id[4];
    int up_cross_id;
    int right_cross_id;
    int down_cross_id;
    int left_cross_id;
}Cross;



typedef struct MGraph
{
    int edges[MAX_ROAD][MAX_ROAD];//邻接矩阵，记录的是两点之间的距离，也就是权值 
    int n,e;//顶点数和边数
}MG;




//打印时间。入参为打印信息头
void print_time(const char * const head);
bool Astar_search(int from_,int to_,Road* road,int min_road_id,int max_road_id,Cross* cross,int min_cross_id,int max_cross_id);
#endif

