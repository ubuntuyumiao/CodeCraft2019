#include <iostream>
#include <vector>
#include "include.h"
using namespace std;
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define WHITE   "\033[37m"      /* White */
#define CYAN         "\033[0;36m"
#define LIGHT_CYAN   "\033[1;36;43m"
#define PURPLE       "\033[0;35m"
#define LIGHT_PURPLE "\033[1;35m"
#define LIGHT_BLUE   "\033[1;34m"
#define DARY_GRAY    "\033[1;30m"

#define DEBUG
// #define DEEP_DEBUG
void init_Para(struct System_Para &para_) 
{
        
	para_.speed_near_w =4.5;
	para_.T1_roadlenghtspace_w= -0.008;                                              
	     
	
	para_.road_percent =0.90;          
	para_.DECAY =0.0008;                 
	para_.min_road_per =0.75;
       
	para_.max_car_road=  1200;            
	para_.T_SOFT = 50;                 
	para_.T_SOFT_RATE = 0.000004;  
/********动态调度影响参数**********/
        para_.normalize_length_w =0.0; 
	para_.length_di_speed= 60.0;
	para_.best_space_w= 12.8;
	para_.car_onroad_w=0.65;
	para_.garage_size_w =0.08;
	para_.car_willonroad=0.2 ;

}
void out(std::string s)
{
  std::cout<< "Debug: "<< s <<std::endl;
}
//将车辆的出发时间进行排序
int  campare_settime(const void * a, const void * b)
{
 return (*(Car *)a).set_time > (*(Car *)b).set_time ? 1 : -1; 
}
int  campare_carid(const void * a, const void * b)
{
 return (*(Car *)a).id > (*(Car *)b).id ? 1 : -1; 
}
int  campare_speed(const void * a, const void * b)
{
 return (*(Car *)a).max_speed < (*(Car *)b).max_speed ? 1 : -1; 
}
int  campare_route(const void * a, const void * b)
{
 return (*(Car *)a).route > (*(Car *)b).route ? 1 : -1; 
}
bool cmp(int a,int b){
    return a > b;
}

//输入
int not_equal(int a,int b)
{
  if(a==b) return 0;
    else return 1;
}
int min(int a, int b)
{
  if(a>b) return b;
     return a;
}
//将其从等代表中删除
bool check_and_delete(int car_id,std::vector<int> &wait_list_)
{
    if(wait_list_.empty()) return false;
  std::vector<int>::iterator it;
  it =find(wait_list_.begin(),wait_list_.end(),car_id);
  if (it!=wait_list_.end())  wait_list_.erase(it); 
     else return false;
  return true;
}
//检查此车是否在检锁列表中出现
bool check_in_list(int car_id_,std::vector<int> &wait_list_)
{
  if(wait_list_.empty()) { return false; }
  std::vector<int>::iterator it;
  it =find(wait_list_.begin(),wait_list_.end(),car_id_);
  if (it!=wait_list_.end()) return true;
    else  return false;
}
int  campare_dir(const void * a, const void * b)
{
 return (*(pior_cross *)a).dir > (*(pior_cross *)b).dir ? 1 : -1; 
}
int  campare_id(const void * a, const void * b)
{
 return (*(pior_cross *)a).road_id > (*(pior_cross *)b).road_id ? 1 : -1; 
}
void quickSort(Car* car_list,int car_begin,int car_end)
{
    qsort(car_list+car_begin, car_end-car_begin+1, sizeof(car_list[0]), campare_settime);
}
