#include <iostream>
#include <vector>
#include "include.h"
using namespace std;
int cross_tosub(int cross_id_,std::vector<int>&cross_dict_)
{
    vector<int>::iterator it = find(cross_dict_.begin(), cross_dict_.end(),cross_id_);
 
    if(it != cross_dict_.end())
        return &*it-&cross_dict_[0];
    else
        cout<<"can not find cross"<<" "<<cross_id_<<endl;
 
    return &*it-&cross_dict_[0];
}
int car_tosub(int car_id_,std::vector<int>&car_dict_)
{
    vector<int>::iterator it = find(car_dict_.begin(), car_dict_.end(),car_id_);
 
    if(it != car_dict_.end())
        return &*it-&car_dict_[0];
    else
        cout<<"can not find car"<<endl;
 
    return &*it-&car_dict_[0];
}
int road_tosub(int road_id_,std::vector<int>&road_dict_)
{
    vector<int>::iterator it = find(road_dict_.begin(), road_dict_.end(),road_id_);
 
    if(it != road_dict_.end())
        return &*it-&road_dict_[0];
    else
        cout<<"can not find road"<<endl;
 
    return &*it-&road_dict_[0]; 
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