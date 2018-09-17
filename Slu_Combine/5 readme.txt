1 FileHandler.cpp中的initOverview 修改setSelection_=0/8, 默认选择不用手动
2 showSol()函数可现实solution(sCluCurrent_或sNodCurrent_)或instance
3 成功在cluvns中调用lkh求解tsp算法 -> extern C 
	Solution.cpp文件中增加
		extern "C" {
			int runlkh(int *x);
		};
	LKHmain.c文件修改main函数为runlkh函数
4 重点修改NodeSolution* ClusterSolution::convert(void)函数
	1 注释了类内随机选择节点选项 if ((double)rand() / RAND_MAX < 0) 
	2 改进：找出该聚类c的起始和终点,并按LHK方法固定起点终点,解决该TSP问题 （需要核验整合函数正确性）
		不断生成新的lkh.tsp;调用LKH不断读取lkh.tour文件;
		修改addstop从Tempvnodes增加 nodT->addStop
5 修改未改进的迭代终止条件
	sNodCurrent_->getTotalDist() < bestNodTour
6 注释do while(goToNodeVNS)过程 不采用nodevns过程
7 修改lhk的WriteTour.c,判断是否为读取lkh.tour文件;如是,按指定格式输出.
	int equal = strcmp("lkh.tour", FileName); //相同为0 
8 生成的lkh.tsp是安装matrix距离,非坐标
9 增加聚类内部vns后的diversification,及之后的node的vns
10 增加random选择一个cluster内部的起终点

	TODO:
 	1 random 内部sequence; 增加聚类的顺序改变; 去除nodevns的算子的可行性(结果似乎可行)
 	2 增加random只选择起点，终点依据LHK算法决定；
 	3 改变BF获取初始BPP问题解的方式