#include "include.h"

Road road[MAX_ROAD],road_sorted[MAX_ROAD];
Car  car[MAX_CAR],car_sorted[MAX_CAR];
Cross cross[MAX_CROSS],cross_sorted[MAX_CROSS];
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
	    road_sorted[Subscript] = road[Subscript]=road[0];
	  }
	road[Subscript].car_on_road=0;
	road_sorted[Subscript] = road[Subscript];
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
	    car_sorted[Subscript] = car[Subscript] = car[0];
	  } 
	car_sorted[Subscript] = car[Subscript];
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
	    cross_sorted[Subscript] =cross[Subscript] = cross[0];
	  } 
	cross_sorted[Subscript] =cross[Subscript];
	cross_sorted[Subscript].up_cro_to_me_isempty=
				  cross_sorted[Subscript].right_cro_to_me_isempty=
				  cross_sorted[Subscript].down_cro_to_me_isempty=
				  cross_sorted[Subscript].left_cro_to_me_isempty=true;
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
       {
	 //初始化路口的十字连通关系
	 cross[min_cross_id+i-1].up_cross_id = cross[min_cross_id+i-1].right_cross_id =cross[min_cross_id+i-1].down_cross_id =cross[min_cross_id+i-1].left_cross_id=-1;
	 for(int j=1;j<cross_num+1;j++)         
	    map[i][j].id= -1;
       }
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
		cross[i].up_cross_id=j;
		if(road[cross[i].road_id[0]].flag_twoway==1) {   
		  //如果是双向道路 （j,i）元素值与(i,j)处相等 下同
		  map[j][i]=road[cross[i].road_id[0]];
		  cross[j].down_cross_id=i;
		}
	      }
	   if((cross[i].road_id[1]!=-1)&&(cross[i].road_id[1]==cross[j].road_id[3]))
	      { 
		map[i][j]=road[cross[i].road_id[1]];
		cross[i].right_cross_id=j;
		if(road[cross[i].road_id[1]].flag_twoway==1){
		  map[j][i]=road[cross[i].road_id[1]];
		  cross[j].left_cross_id=i;
		}
	      }
	   if((cross[i].road_id[2]!=-1)&&(cross[i].road_id[2]==cross[j].road_id[0])) 
	      { 
		map[i][j]=road[cross[i].road_id[2]];
		cross[i].down_cross_id=j;
		if(road[cross[i].road_id[2]].flag_twoway==1){
		  map[j][i]=road[cross[i].road_id[2]];
		  cross[j].up_cross_id=i;
		}
	      }
	   if((cross[i].road_id[3]!=-1)&&(cross[i].road_id[3]==cross[j].road_id[1])) 
	      { 
		map[i][j]=road[cross[i].road_id[3]];
		if(road[cross[i].road_id[3]].flag_twoway==1){
		  cross[i].left_cross_id=j;
		  map[j][i]=road[cross[i].road_id[3]];
		  cross[j].right_cross_id=i;
		}
	      }
	 }
       }
/*****************测试数据结构正确性 查看路口的十字连通情况 -1为不连通*****************/
//           for(int across=min_cross_id;across<=max_cross_id;across++)
// 	  {
//           printf("%d 号路口上面是 %d 号路口\n",across,cross[across].up_cross_id);
// 	  printf("%d 号路口右面是 %d 号路口\n",across,cross[across].right_cross_id);
// 	  printf("%d 号路口下面是 %d 号路口\n",across,cross[across].down_cross_id);
// 	  printf("%d 号路口左面是 %d 号路口\n",across,cross[across].left_cross_id);
// 	  std::cout << std::endl;
// 	  }
/*****************测试数据结构正确性 查看路口的十字连通情况 -1为不连通*****************/

/*********************************将所有路口与道路信息以邻接矩阵表示*********************************/      
       
   

/*********************************神奇车库 将在该路口出发的车优先级由低到高压入栈 最先出栈则优先级最高*********************************/
          quickSortOfCpp(car_sorted,min_car_id,max_car_id);
          for(int i=min_cross_id;i<=max_cross_id;i++)
	  {
	    for(int j=max_car_id;j>=min_car_id;j--)
	    {
	      //该车找到出发所在路口
	      if(car_sorted[j].set==i)  cross[i].magic_garage.push(car_sorted[j].id);    
	    }
	    
	  }
// 	    printf("\n\n 在 %d 号路口出发的车有： \n",min_cross_id);
// 	    while(!cross[min_cross_id].magic_garage.empty()){
// 	    std::cout << cross[min_cross_id].magic_garage.top()<<"  settime:  "<<car[cross[min_cross_id].magic_garage.top()].set_time ;
// 	    std::cout <<std::endl;
// 	    //打印栈顶元素，实现了顶点的逆序打印
// 	    cross[min_cross_id].magic_garage.pop();      
// 	    //出栈
// 	    }
    
/*********************************神奇车库*********************************/

/*********************************A-Star算法*********************************/ 
      int has_find=0;

     for(int i=min_car_id;i<max_car_id+1;i++)
     {
       if( Astar_search(&car[i], road,min_road_id,max_road_id,cross,min_cross_id,max_cross_id))
           has_find++;
     }
     for(int j=0;j<MAX_CROSS;j++)
     {
       if(car[max_car_id].path[j]==0) break;
       std::cout << car[max_car_id].path[j] << "-->";
     }
     printf("\nget %d solution of %d car\n\n",has_find,car_num);
     
/*********************************A-Star算法*********************************/

/******************************车辆调度规则执行******************************/

   for(int sche_cross=min_cross_id;sche_cross<=max_cross_id;sche_cross++)
   {
        int cur_cross_road[4];
	std::stack<int> cross_shoulebe;
        std::memcpy(cur_cross_road,cross[sche_cross].road_id,sizeof(cross[sche_cross].road_id));
	int array_offset=3;
	for(int i=0;i<4;i++)
	  if(cur_cross_road[i]!=-1)
	   if(road[cur_cross_road[i]].flag_twoway!=1)                      //该道路为单向道，且该路口为入口
	    if(sche_cross!=road[cur_cross_road[i]].end)  cur_cross_road[i]=-1;
	std::sort(cur_cross_road,cur_cross_road+4);    
	while(cur_cross_road[array_offset]>0){cross_shoulebe.push(cur_cross_road[array_offset]); array_offset--;if(array_offset<0) break;}   //去掉不存在的道路 或者不调度 不进入该交叉口的道路	
	 
      //需要调度的路口id升序存于cur_cross_road中    起始下标为 array_offset


	printf("%d 号路口的调度顺序为： ",sche_cross);     //测试输出
	while(!cross_shoulebe.empty())
	{ std::cout << cross_shoulebe.top()<< "  ";cross_shoulebe.pop();}
	std::cout << std::endl;
   }

   
/******************************车辆调度规则执行******************************/

       
  print_time("End");
  return 0;
}