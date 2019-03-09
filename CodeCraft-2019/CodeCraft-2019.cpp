#include <iostream>
#include <stdio.h>
#include <fstream>
#include <cstring>
#include <stack>
#include "lib_time.h"
#include "lib_io.h"
Road road[MAX_ROAD];
Car  car[MAX_CAR];
Cross cross[MAX_CROSS];
Road map[MAX_CROSS][MAX_CROSS];
// 道路（边）总数 路口（节点）数 车辆数
int road_num=0,cross_num=0,car_num=0;
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
    int count=1;
    int min_car_id,max_car_id;
    int min_road_id,max_road_id;
    int min_cross_id,max_cross_id;
    //设置标志位 将文件中的ID号作为数组下标 例如car[5000] 即为编号为5000 的车子信息 道路等同
    bool first_line=true;
    int Subscript=0;
    
    std::cout<<"read start"<<std::endl;
    while(!fin_road.eof())
      {
       char str_tr[50];
       fin_road.getline(str_tr,'\r\n');
       if(str_tr[0]=='#')  continue;
        else if(str_tr[0]=='(')
         {
	   
	    std::sscanf(str_tr,"(%d,%d,%d,%d,%d,%d,%d)",   &(road[Subscript].id),
						    &(road[Subscript].road_length),
						    &(road[Subscript].limit_speed),
						    &(road[Subscript].lane_num),
						    &(road[Subscript].start),
						    &(road[Subscript].end),
						    &(road[Subscript].flag_twoway));
	  if(first_line) 
	  {
	    first_line=false; Subscript=road[Subscript].id;
	    min_road_id= Subscript;
	    road[Subscript] = road[0];
	  } 
	Subscript++;
	count++;
         }
       }
       fin_road.close();
       max_road_id = Subscript-1;
       road_num=count-1;
       count=1;
       Subscript=0;
       first_line=true;
    while(!fin_car.eof())
      {
       char str_tc[50];
       fin_car.getline(str_tc,'\r\n');
       if(str_tc[0]=='#')  continue;
        else if(str_tc[0]=='(')
         {
	   
	   std::sscanf(str_tc,"(%d,%d,%d,%d,%d)",   &(car[Subscript].id),
						    &(car[Subscript].set),
						    &(car[Subscript].goal),
						    &(car[Subscript].max_speed),
						    &(car[Subscript].set_time));
	  if(first_line) 
	  {
	    first_line=false; Subscript=car[Subscript].id;
	    min_car_id = Subscript;
	    car[Subscript] = car[0];
	  } 
	Subscript++;
	count++;
         }
       }
       fin_car.close();
       max_car_id= Subscript-1;
       car_num=count-1;
       count=1;
       Subscript=0;
       first_line=true;
    while(!fin_cross.eof())
      {
       char str_ts[50];
       fin_cross.getline(str_ts,'\r\n');
       if(str_ts[0]=='#')  continue;
        else if(str_ts[0]=='(')
         {
	   
	   std::sscanf(str_ts,"(%d,%d,%d,%d,%d)",   &(cross[Subscript].id),
						    &(cross[Subscript].road_id[0]),
						    &(cross[Subscript].road_id[1]),
						    &(cross[Subscript].road_id[2]),
						    &(cross[Subscript].road_id[3]));
	  if(first_line) 
	  {
	    first_line=false; Subscript=cross[Subscript].id;
	    min_cross_id = Subscript;
	    cross[Subscript] = cross[0];
	  } 
	Subscript++;
	count++;
         }
       }
       cross_num=count-1;
       fin_cross.close();
       max_cross_id = Subscript-1;
       std::cout<<"read finished"<<std::endl;
       int l=min_car_id;
       int m=min_cross_id;
       int n=min_road_id;
       printf("MinNumber %d  car_info: %d  %d  %d  %d  %d \n",l,car[l].id,car[l].set,car[l].goal,car[l].max_speed,car[l].set_time);
       printf("MinNumber %d  cross: %d  %d  %d  %d  %d \n",m,cross[m].id,cross[m].road_id[0],cross[m].road_id[1],cross[m].road_id[2],cross[m].road_id[3]);
       printf("MinNumber %d  road: %d  %d  %d  %d  %d  %d  %d\n",n,(road[n].id),
						    (road[n].road_length),
						    (road[n].limit_speed),
						    (road[n].lane_num),
						    (road[n].start),
						    (road[n].end),
						    (road[n].flag_twoway));
       
       printf("road_num: %d  car_num: %d  cross_num: %d \n",max_road_id-min_road_id+1,max_car_id-min_car_id+1,max_cross_id-min_cross_id+1);
      
       //初始化地图 将所有路口连接置为-1 表示无连接 共有1～(cross_num-1) 个路口
       //数组以0下表开始 直观起见 map[i][j]直接指第i个路口到第j个路口信息
       
       for(int i=1;i<cross_num+1;i++)       
	 for(int j=1;j<cross_num+1;j++)         
	    map[i][j].id= -1;

/*********************************将所有路口与道路信息以邻接矩阵表示*********************************/
/*	             1      2      3       .....  (cross_num)
 *                ------------------------------------------------> matrix_cols
 *                '   
 *              1 ' -1  |  R2  |   R3
 *                '
 *                ' R2' |  -1  |   R4
 *                '
 *              2 ' R3' |  R4' |   -1
 *                '
 *                '
 *                '
 *              3 '
 *                '
 *              . '
 *              . '
 *              . '
 *              . '
 *                '
 *    (cross_num) '
 *                '
 *                '
 *  matrix_rows   '
 *  
 */       
       for(int i=min_cross_id;i<max_cross_id+1;i++)
       {
	 for(int j=i+1;j<max_cross_id+1;j++)
	 {
	   if((cross[i].road_id[0]!=-1)&&(cross[i].road_id[0]==cross[j].road_id[2])) 
	      {
		map[i][j]=road[cross[i].road_id[0]];
		if(road[cross[i].road_id[0]].flag_twoway==1)    
		  //如果是双向道路 （j,i）元素值与(i,j)处相等 下同
		  map[j][i]=road[cross[i].road_id[0]];
	      }
	   if((cross[i].road_id[1]!=-1)&&(cross[i].road_id[1]==cross[j].road_id[3]))
	      { 
		map[i][j]=road[cross[i].road_id[1]];
		if(road[cross[i].road_id[1]].flag_twoway==1)
		  map[j][i]=road[cross[i].road_id[1]];
	      }
	   if((cross[i].road_id[2]!=-1)&&(cross[i].road_id[2]==cross[j].road_id[0])) 
	      { 
		map[i][j]=road[cross[i].road_id[2]];
		if(road[cross[i].road_id[2]].flag_twoway==1)
		  map[j][i]=road[cross[i].road_id[2]];
	      }
	   if((cross[i].road_id[3]!=-1)&&(cross[i].road_id[3]==cross[j].road_id[1])) 
	      { 
		map[i][j]=road[cross[i].road_id[3]];
		if(road[cross[i].road_id[3]].flag_twoway==1)
		  map[j][i]=road[cross[i].road_id[3]];
	      }
	 }
       }
       
       printf("Map info: %d 号路口与 %d 号路口连接的道路为： %d\n",27,28,map[28][27].id);
/*********************************将所有路口与道路信息以邻接矩阵表示*********************************/      
       
       
/*********************************最短路径Dijkstra*********************************/      
          int from,to,goal;int which_car;
	  for( which_car=min_car_id;which_car<=max_car_id;which_car++)
	  {
	  from=car[which_car].set,to=car[which_car].goal,goal=to;  
//初始化距离矩阵
          
          int dist[MAX_ROAD], path[MAX_ROAD];
          memset(G.edges, INF, sizeof(G.edges));
          G.e = road_num; G.n=cross_num;
	  for(int x=min_cross_id;x<max_cross_id+1;x++)
	    for(int y=min_cross_id;y<max_cross_id+1;y++)
	      if(map[x][y].id!=-1) G.edges[x][y]=map[x][y].road_length;
//最短路径法  
	  int set[MAX_ROAD],min,u;
	  
	  //对各个数组进行初始化
	  
          for(int i=min_cross_id;i<max_cross_id+1;i++)
	    {
		dist[i]= G.edges[from][i];
		set[i]=0;
		if(G.edges[from][i]<INF) path[i]=from;
		  else path[i]=-1;
	    }
          set[from]=1;
	  path[from]=-1;
	  
	  //初始化结束，最短路径算法开始执行
	  
	  for(int i=min_cross_id;i<max_cross_id;i++)
	  {
	    min=INF;
	    //从剩余顶点中选出一个顶点，该顶点满足：通往这个顶点路径在通往所有剩余顶点的路径中是长度最短的
	    for(int j=min_cross_id;j<=max_cross_id;j++)
	    {
	      if(set[j]==0 && dist[j]<min)
	      {
		u=j;
		min=dist[j];
	      }
	    }
	    set[u]=1;   //某点 置DONE标志 并入最短路径中
	    
	    //以刚并入的顶点作为中间点，对所有通往剩余顶点的路径进行检测
	    for(int j =min_cross_id;j<=max_cross_id;j++)
	    {
	      if(set[j]==0 && dist[u]+G.edges[u][j]<dist[j])
	      {
		dist[j]=dist[u]+G.edges[u][j];
		path[j]=u;
	      }
	    }
 
	  }
        
        //输出最短路径
           
            std::stack<int> route_stack;
	  //这个循环以由叶子结点到根结点的顺序将其入栈
	  while(path[to] != -1){
	      route_stack.push(to);     
	      //入栈
	      to = path[to];
	  } 
	  route_stack.push(to);
	  int offset=0;
	  printf("\n/***%d 号车*********** %d 号路口至 %d 号路口的路径输出****************/\n\n",which_car,from,goal);
	  
	  while(!route_stack.empty()){
// 	    got_path[offset] = route_stack.top();
	      std::cout << route_stack.top() ;
	      if(route_stack.top()!=goal) std::cout<< "--->";
	      //打印栈顶元素，实现了顶点的逆序打印
	      route_stack.pop();      
	      //出栈
	      offset++;
	  }
	  std::cout << std::endl;
	  printf("\n/****************from至to的路径输出****************/\n\n");
       
       
	  }
//        std::cout << "iteration:"<<which_car<<std::endl;
       
       
       
  print_time("End");
  return 0;
}