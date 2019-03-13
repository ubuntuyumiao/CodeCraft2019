#include "io.h"

#define _DEBUG

#define INLINE  static __inline
#ifdef _DEBUG
#define PRINT   printf
#else
#define PRINT(...)
#endif




/******************************车辆调度规则执行******************************/
bool car_schedule()
{
//     for(int current_cross=min_cross_id;current_cross<=max_cross_id;current_cross++)
//     {
//         
//   
// 
// 
// 
// 
// 
// 
// 
// 
//     }

}


/******************************车辆调度规则执行******************************/


bool Astar_search(Car *car_,Road* road,int min_road_id,int max_road_id,Cross* cross,int min_cross_id,int max_cross_id)
{
    
    int  from=  car_->set  ;
    int	 to=  car_->goal     ;
    int i=0;
    int path[2000];
    memset(path,-1,sizeof(path));
    A_star *a=new A_star(road, min_road_id, max_road_id,cross,min_cross_id, max_cross_id);
    node *start=new node(from);
    node *end=new node(to);
    
    a->search(start,end);
    if(a->find_path==true){
    while(!a->route_stack.empty()){
        car_->path[i]= a->route_stack.top() ;
	if(a->route_stack.top()!=to) std::cout<< a->route_stack.top() <<"--->";
	 else std::cout<< a->route_stack.top();
	//打印栈顶元素，实现了顶点的逆序打印
	a->route_stack.pop();      
	i++;
	//出栈
    }
    }
    std::cout << std::endl<<std::endl;
    return a->find_path;
    
   
    
//     for(int i=0;i<2000;i++)
//     {
//       Answer[0].
//       
//     }
     
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
