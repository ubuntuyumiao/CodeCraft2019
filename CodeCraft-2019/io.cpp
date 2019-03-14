#include "io.h"

#define _DEBUG

#define INLINE  static __inline
#ifdef _DEBUG
#define PRINT   printf
#else
#define PRINT(...)
#endif

bool All_car_iscompleted(Car* car_array,int min_car_id_,int max_car_id_)
{
   for(int i=min_car_id_;i<=max_car_id_;i++)
    {
      if((car_array[i].state!=completed)&&(car_array[i].now_road!=-1)) return false;   //已上路的车是否都已达到终止态
    }
return true;
}

/******************************检查某道路是否为空 不为空 那最高优先级的有余量的车道是哪条 最优先车位的下标？******************************/

road_empty check_road_empty(Cross *cur_cross_,Road *cur_road_)
{
       road_empty road_situation;
       road_situation.is_empty=true;
       road_situation.lane=-1;
       for(int j=cur_road_->road_length-1;j>=0;j--)
       {
	 for(int l=0;l<cur_road_->lane_num;l++)
	 if(cur_road_->load[(cur_cross_->id==cur_road_->end)?0:1][l][j]!=0)  
	 {
	   if(road_situation.is_empty) 
	   {
	     road_situation.is_empty=false;
	     road_situation.lane=l;
	     road_situation.offset=j;
	     break;
	   }
	  }
       }
       
return road_situation; 
}
/******************************调度k号车辆后 更新道路******************************/
void car_change_raod(Car *car_,Road *road_)
{
  
}
/******************************车辆调度规则执行******************************/


bool Astar_search(Car *car_,Road* road_array_,int min_road_id,int max_road_id,Cross* cross_array_,int min_cross_id,int max_cross_id)
{
    
    int  from=  car_->set  ;
    int	 to=  car_->goal     ;
    int i=0;
    A_star *a=new A_star(road_array_, min_road_id, max_road_id,cross_array_,min_cross_id, max_cross_id);
    
    
    node *start=new node(from);
    node *end=new node(to);
    
    a->search(start,end);
    if(a->find_path==true){
    while(!a->route_stack.empty()){
        car_->path[i]= a->route_stack.top() ;
//	if(a->route_stack.top()!=to) std::cout<< a->route_stack.top()"--->";
	//打印栈顶元素，实现了顶点的逆序打印
	a->route_stack.pop();      
	i++;
	//出栈
    }
    }
//     std::cout << std::endl<<std::endl;
    return a->find_path;

}

void print_time(const char *head)
{
#ifdef _DEBUG
    struct timeb rawtime;
    struct tm * timeinfo;
    ftime(&rawtime);
    timeinfo = localtime(&rawtime.time);

    static int ms = rawtime.millitm;
    static unsigned long s = rawtime.time;
    int out_ms = rawtime.millitm - ms;
    unsigned long out_s = rawtime.time - s;
    ms = rawtime.millitm;
    s = rawtime.time;

    if (out_ms < 0)
    {
        out_ms += 1000;
        out_s -= 1;
    }
    printf("%s date/time is: %s \tused time is %lu s %d ms.\n", head, asctime(timeinfo), out_s, out_ms);
    
#endif
}
//将车辆的出发时间进行排序
int  campare_settime(const void * a, const void * b)
{
 return (*(Car *)a).set_time > (*(Car *)b).set_time ? 1 : -1; 
}

void quickSortOfCpp(Car* car_list,int car_begin,int car_end)
{
    qsort(car_list+car_begin, car_end-car_begin+1, sizeof(car_list[0]), campare_settime);
}
bool cmp(int a,int b){
    return a > b;
}
int min_ready_road_id(Road *road_,Cross *cross_)
{
   int up,right,down,left;
   if(cross_->up_cross_id ==-1)    up=-1;
   if(cross_->right_cross_id ==-1) right=-1;
   if(cross_->down_cross_id ==-1)  down=-1;
   if(cross_->left_cross_id ==-1)  left=-1;
}
