#include <iostream>
#include <fstream>
#include <stdio.h>
#include <lib_io.h>
#include "lib_time.h"
Road road[MAX_ROAD];
Car  car[MAX_CAR];
Cross cross[MAX_CROSS];
int main(int argc,char** argv)
{
  print_time("Begin");
    std::ifstream fin_car(argv[1]);
    std::ifstream fin_road(argv[2]);
    std::ifstream fin_cross(argv[3]);
    
    if(!fin_road.is_open())  {std::cout<<"/**ERROR:No Road input file**/"<<std::endl;return 0;} 
    if(!fin_car.is_open())   {std::cout<<"/**ERROR:No Car input file**/"<<std::endl;return 0;}
    if(!fin_car.is_open())   {std::cout<<"/**ERROR:No Cross input file**/"<<std::endl;return 0;}
    std::string s;
    int road_num=0,car_num=0,cross_num=0;
    std::cout<<"read start"<<std::endl;
    while(!fin_road.eof())
      {
       char str_tr[50];
       fin_road.getline(str_tr,'\r\n');
       if(str_tr[0]=='#')  continue;
        else if(str_tr[0]=='(')
         {
	   
	   std::sscanf(str_tr,"(%d,%d,%d,%d,%d,%d,%d)",   &(road[road_num].id),
						    &(road[road_num].road_length),
						    &(road[road_num].limit_speed),
						    &(road[road_num].lane_num),
						    &(road[road_num].start),
						    &(road[road_num].end),
						    &(road[road_num].flag_twoway));
	road_num++;
         }
       }
       fin_road.close();
    while(!fin_car.eof())
      {
       char str_tc[50];
       fin_car.getline(str_tc,'\r\n');
       if(str_tc[0]=='#')  continue;
        else if(str_tc[0]=='(')
         {
	   
	   std::sscanf(str_tc,"(%d,%d,%d,%d,%d)",   &(car[car_num].id),
						    &(car[car_num].set),
						    &(car[car_num].goal),
						    &(car[car_num].max_speed),
						    &(car[car_num].set_time));
	car_num++;
         }
       }
       fin_car.close();
    while(!fin_cross.eof())
      {
       char str_ts[50];
       fin_cross.getline(str_ts,'\r\n');
       if(str_ts[0]=='#')  continue;
        else if(str_ts[0]=='(')
         {
	   
	   std::sscanf(str_ts,"(%d,%d,%d,%d,%d)",   &(cross[cross_num].id),
						    &(cross[cross_num].road_id[0]),
						    &(cross[cross_num].road_id[1]),
						    &(cross[cross_num].road_id[2]),
						    &(cross[cross_num].road_id[3]));
	cross_num++;
         }
       }
       fin_cross.close();
       std::cout<<"read finished"<<std::endl;
       int l=0;
       int m=0;
       int n=0;
       printf("Number %d  car_info: %d  %d  %d  %d  %d \n",l,car[l].id,car[l].set,car[l].goal,car[l].max_speed,car[l].set_time);
       printf("Number %d  cross: %d  %d  %d  %d  %d \n",m,cross[m].id,cross[m].road_id[0],cross[m].road_id[1],cross[m].road_id[2],cross[m].road_id[3]);
       printf("Number %d  road: %d  %d  %d  %d  %d  %d  %d\n",n,(road[n].id),
						    (road[n].road_length),
						    (road[n].limit_speed),
						    (road[n].lane_num),
						    (road[n].start),
						    (road[n].end),
						    (road[n].flag_twoway));
       
       printf("road_num: %d  car_num: %d  cross_num: %d \n",road_num,car_num,cross_num);

  print_time("End");
  return 0;
}