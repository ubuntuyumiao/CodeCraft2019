// #include <iostream>
// #include <cstring>
// #include <stack>
// using namespace std;
// #define MAX 100
// #define INF 0x3f3f3f3f
// int dist[MAX], path[MAX];
// struct MGraph
// {
//     int edges[MAX][MAX];//邻接矩阵，记录的是两点之间的距离，也就是权值 
//     int n,e;//顶点数和边数
// }G;
// void init() {
//   //默认为INF
//     memset(G.edges, INF, sizeof(G.edges));
// }
// void insert(int u, int v, int w) {
//     G.edges[u][v] = w;//
// }
// int goal_path[100];
// void printfPath(int path[], int a,int got_path[]){
//     stack<int> s;
// 
//     //这个循环以由叶子结点到根结点的顺序将其入栈
//     while(path[a] != -1){
//         s.push(a);     //入栈
//         a = path[a];
//     } 
//     s.push(a);
//     int offset=0;
//     while(!s.empty()){
//       got_path[offset] = s.top();
//         cout << s.top() << " ";//打印栈顶元素，实现了顶点的逆序打印
//         s.pop();      //出栈
// 	offset++;
//     }
//     cout << endl;
// } 
// //顶点默认从0到n
// 
// void Dijkstra(MGraph g, int v, int dist[], int path[]){ 
//     int set[MAX], min, i, j, u;
//     //对各个数组进行初始化
//     for(i = 0; i < g.n; i++){
//         dist[i] = g.edges[v][i];
//         set[i] = 0;
//         if(g.edges[v][i] < INF){
//             path[i] = v;
//         }else{
//             path[i] = -1;
//         }
//     } 
//     set[v] = 1; 
//     path[v] = -1;
//     //初始化结束，关键操作开始
//     for(i = 0; i < g.n - 1; i++)
//     {
//         min = INF;//找到的点   目前最小 
//         //这个循环每次从剩余顶点中选出一个顶点，通往这个顶点的路径在通往所有剩余顶点的路径中是长度最短的
//         for(j = 0; j < g.n; j++){
//             if(set[j] == 0 && dist[j] < min){
//                 u = j;
//                 min = dist[j];
//             }
//         } 
//         set[u] = 1;   //某点 DONE标志
// 	//将选出的顶点并入最短路径中
//         //这个循环以刚并入的顶点作为中间点，对所有通往剩余顶点的路径进行检测
//         for(j = 0; j < g.n; j++) {
//             //这个if判断顶点u的加入是否会出现通往顶点j的更短的路径，如果出现，则改变原来路径及其长度，否则什么都不做
//             if(set[j] == 0 && dist[u] + g.edges[u][j] < dist[j]){
//                 dist[j] = dist[u] + g.edges[u][j];
// 		//更新路径长度 
//                 path[j] = u;
// 		//更新路径顶点 
//             } 
//         } 
//     } 
// }
// int main() {
//     init();
//     int n, m;
//     //n个点，m条边
//     int a, x, y, w;
//     m=12,n=7;
//     G.e = m;
//     G.n = n;
//     G.edges[0][1]=4;
//     G.edges[1][0]=6;
//     
//     G.edges[0][2]=6;
//     G.edges[2][0]=7;
//     
//     G.edges[0][3]=1;
//     G.edges[3][0]=4;
//     
//     G.edges[1][4]=6;
//     G.edges[4][1]=3;
//     
//     G.edges[1][2]=4;
//     G.edges[2][1]=1;
//      
//     G.edges[2][4]=2;
//     G.edges[4][2]=5;
//     
//     G.edges[2][5]=5;
//     G.edges[5][2]=1;
//     
//     G.edges[3][2]=6;
//     G.edges[2][3]=6;
//     
//     G.edges[3][5]=1;
//     G.edges[5][3]=4;
//     
//     G.edges[4][6]=8;
//     G.edges[6][4]=5;
//     
//     G.edges[5][4]=1;
//     G.edges[4][5]=1;
//     
//     G.edges[5][6]=6;
//     G.edges[6][5]=3;
// //     for(int i = 0; i < m; i++){
// //         cin >> x >> y >> w;
// //         insert(x, y, w);
// //     }
//     for(int car=0;car<5;car++)
//     {
//       memset(goal_path, -1, sizeof(goal_path));
//       Dijkstra(G, 0, dist, path);
//       printfPath(path, 5,goal_path);
//       for(int i = 0; i < sizeof(goal_path)/sizeof(int); i++) 
//       {
// 	if(goal_path[i+1]==-1) break;
// 	   else 
// 	    {
// 	      //已走过的道路进行权重更改
// 	      G.edges[goal_path[i]][goal_path[i+1]]+=2 ;   
// 	    }
//       }
//       cout << endl;
//     }
//     return 0;
// }