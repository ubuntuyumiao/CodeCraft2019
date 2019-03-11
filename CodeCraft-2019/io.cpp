#include "io.h"

#define _DEBUG

#define INLINE  static __inline
#ifdef _DEBUG
#define PRINT   printf
#else
#define PRINT(...)
#endif

bool Astar_search(int from_,int to_,Road* road,int min_road_id,int max_road_id,Cross* cross,int min_cross_id,int max_cross_id)
{
    
    int  from=  from_  ;
    int	 to=  to_     ;
    A_star *a=new A_star(road, min_road_id, max_road_id,cross,min_cross_id, max_cross_id);
    
    
    node *start=new node(from);
    node *end=new node(to);
    
    a->search(start,end);
    if(a->find_path==true){
    while(!a->route_stack.empty()){
	std::cout << a->route_stack.top() ;
	if(a->route_stack.top()!=to_) std::cout<< "--->";
	//打印栈顶元素，实现了顶点的逆序打印
	a->route_stack.pop();      
	//出栈
    }
    }
    std::cout << std::endl<<std::endl;
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


