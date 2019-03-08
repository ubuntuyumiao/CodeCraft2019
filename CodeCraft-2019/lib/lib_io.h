#ifndef __LIB_IO_H__
#define __LIB_IO_H__

#define MAX_ROAD    5000
#define MAX_CAR     5000
#define MAX_CROSS   5000


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




#endif

