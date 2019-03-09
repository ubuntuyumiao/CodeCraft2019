#ifndef __LIB_IO_H__
#define __LIB_IO_H__

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
}Cross;



struct MGraph
{
    int edges[MAX_ROAD][MAX_ROAD];//邻接矩阵，记录的是两点之间的距离，也就是权值 
    int n,e;//顶点数和边数
}G;


#endif

