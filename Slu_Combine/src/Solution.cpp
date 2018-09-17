#include "Solution.h"
#include "Heuristic.h"
#include "Printer.h"
#include <iostream>
#include <fstream> //输出到txt使用

using namespace std;

extern "C" {
	//int runlkh(int argc, char *argv[]);
	//int runlkh(int, int*);
	int runlkh(int *x);
};

Solution::Solution(){}

Solution::Solution(CluVRPinst* cluVRPinst)
{
	this->cluVRPinst_ = cluVRPinst;
	this->totalDist_ = 0;
}

Solution::~Solution(){}

double Solution::getTotalDist(void) const
{
	return totalDist_;
}

void Solution::setTotalDist(double value)
{
	totalDist_ = value;
}

/********************************************************************************/
/********************************************************************************/

ClusterSolution::ClusterSolution(CluVRPinst* cluVRPinst) : Solution(cluVRPinst){}

ClusterSolution::~ClusterSolution()
{
	for (unsigned int i = 0; i < vTrips_.size(); i++) delete vTrips_.at(i);
	vTrips_.clear();
}

void ClusterSolution::addTrip(CluTrip* t)
{
	vTrips_.push_back(t);
	totalDist_ += t->getDistance();
}

int ClusterSolution::getRandomFeasibleVehicle(Cluster*& c, bool& success)
{
	std::vector<unsigned int short> feasVeh;

	for (unsigned int i = 0; i < vTrips_.size(); i++)
	{
		if (vTrips_.at(i)->getSpareCapacity() >= c->getDemand())
		{
			feasVeh.push_back(i);
		}
	}

	if (feasVeh.size() != 0)
	{	
		success = true;
		return feasVeh.at(rand() % feasVeh.size());
	}
	else
	{
		success = false;
		return -1;
	}
}

int ClusterSolution::getClosestFeasibleVehicle(Cluster*& c, bool& success)
{
	double minDist = BIG_M;
	int veh = -1;

	success = false;

	for (unsigned int i = 0; i < vTrips_.size(); i++)
	{
		if (vTrips_.at(i)->getSpareCapacity() >= c->getDemand())
		{
			double dist = cluVRPinst_->getDistClusters(vTrips_.at(i)->getLastPosition(), c);	//获取c与第i个trip对应的最后一个类的距离
			if (dist < minDist)
			{
				minDist = dist;
				veh = i;
				success = true;
			}
		}
	}

	return veh;
}

Pos ClusterSolution::findPosition(Cluster* c)
{
	for (int v = 0; v < cluVRPinst_->getnVehicles(); v++)
	{
		for (int i = 1; i < this->vTrips_.at(v)->getSize() - 1; i++)
		{
			if (this->vTrips_.at(v)->getCluster(i)->getId() == c->getId())
			{
				Pos p(v, i);
				return p;
			}
		}
	}

	return Pos(-1, -1);
}

double ClusterSolution::calcTotalDist(void)
{
	totalDist_ = 0;

	for (int i = 0; i < cluVRPinst_->getnVehicles(); i++)
	{
		totalDist_ += vTrips_.at(i)->calcDistance();
	}

	return totalDist_;
}


//聚类solution转换为节点solution,返回nodSol(由多条nodT构成)
NodeSolution* ClusterSolution::convert(void)
{
	CluTrip* cluT = nullptr;
	NodTrip* nodT = nullptr;
	NodeSolution* nodSol = new NodeSolution(cluVRPinst_);

	//循环;针对每个聚类对应的veh/trip,获取对应的nodT,再加入该trip到nodSlo中
	//循环1:trip无顺序;   每个trip计算nodT距离,并将该trip增加到解solution		nodT->calcDistance();	nodSol->addTrip(nodT);
	for (int v = 0; v < cluVRPinst_->getnVehicles(); v++)
	{
		//copy current Clustered Trip
		cluT = vTrips_.at(v);

		//Create new Node Trip
		nodT = new NodTrip(cluVRPinst_);
		nodT->setTotalDemand(vTrips_.at(v)->getTotalDemand());	

		//for every cluster in the Clustered Trip, take all nodes and push them in the Node Trip
		//循环2;Cluster无顺序;   针对veh/trip中的每个Cluster,逐步计算获取完整的nodT.
		for (int i = 0; i < cluT->getSize(); i++)
		{
			Cluster* c = cluT->getCluster(i);

			//there is only one node in the current cluster -> just add the node
			if (c->getnNodes() == 1)
			{
				nodT->addStop(c->getvNodesPtr().front());
				continue;
			}
			else
			{
				//找出当前聚类c的所有节点nodes
				std::vector<Node*> vNodes = c->getvNodesPtr();

				//参数;判断如何选择聚类c内节点顺序
				if ((double)rand() / RAND_MAX < 0) //Params::RANDOM_CONVERSION)
				{
					/* STRATEGY 1 => add nodes in random order for this cluster */
					//方法1;随机
					random_shuffle(vNodes.begin(), vNodes.end());

					for (int j = 0; j < c->getnNodes(); j++)
					{
						nodT->addStop(vNodes.at(j));
					}
				}
				else // TODO - 找出该聚类c的起始和终点,并按nearest方法解决该TSP问题
				{
					// 定义TempvNodes : 不删除的vNodes;
					std::vector<Node*> TempvNodes = vNodes;

					//if满足: 随机选择起点 终点 不满足:按聚类距离选择 起点 终点
					int lastinTempvNodes, firstinTempvNodes;
					if ((double)rand() / RAND_MAX < 0.4) //Params::RANDOM_CONVERSION)
					{
						cout << (double)rand() << " " << RAND_MAX << " ";
						int nbNod = c->getnNodes();
						firstinTempvNodes = rand() % nbNod;
						lastinTempvNodes = rand() % nbNod;
						while (lastinTempvNodes== firstinTempvNodes)
						{
							lastinTempvNodes = rand() % nbNod;
							if (lastinTempvNodes != firstinTempvNodes)
								break;
						}
						if (lastinTempvNodes>= nbNod || firstinTempvNodes >= nbNod)
						{
							cout << nbNod << " ERRROR " << firstinTempvNodes << " " << lastinTempvNodes << " ";
						}						
						cout << nbNod << "  " << firstinTempvNodes << " " << lastinTempvNodes << " ";
					}
					else
					{
						/* STRATEGY 2 => add nodes according to nearest neighbour approach */
						//方法2:按距离选择
						//2.1 找出距离当前nodT位置最佳的点作为first点,并addStop进入nodT
						//define and add first node in the cluster (closest to current position)

						double minDist = BIG_M;
						int first;
						for (unsigned int j = 0; j < vNodes.size(); j++)
						{
							if (cluVRPinst_->getDistNodes(nodT->getLastPosition(), vNodes.at(j)) < minDist)
							{
								first = j;
								minDist = cluVRPinst_->getDistNodes(nodT->getLastPosition(), vNodes.at(j));
							}
						}

						//nodT->addStop(vNodes.at(first));
						//vNodes.erase(vNodes.begin() + first);

						// ********************* LKH *************************
						//get next cluster to define the appropriate last node in the current cluster (closest to a node in the next cluster)
						//2.2-TempvNodes 找出距离下一cluster最近的当前cluster中的最佳点作为当前聚类的last点,但并不addStop进入nodT(类似预处理聚类间距离)
						std::vector<Node*> vNodesNextTemp = cluT->getCluster(i + 1)->getvNodesPtr();
						minDist = BIG_M;
						firstinTempvNodes = first;
						for (unsigned int j = 0; j < TempvNodes.size(); j++)
						{
							for (unsigned int k = 0; k < vNodesNextTemp.size(); k++)
							{
								//cout << TempvNodes[j]->id << " " << TempvNodes[firstinTempvNodes]->id << " " << cluVRPinst_->getDistNodes(TempvNodes.at(j), vNodesNextTemp.at(k)) << " ";
								if (cluVRPinst_->getDistNodes(TempvNodes.at(j), vNodesNextTemp.at(k)) < minDist & TempvNodes[j]->id != TempvNodes[firstinTempvNodes]->id)
								{
									lastinTempvNodes = j;
									minDist = cluVRPinst_->getDistNodes(TempvNodes.at(j), vNodesNextTemp.at(k));
								}
							}
						}
					}
					if (TempvNodes.size()>=4)
					{
						cout << " ";
					}
					// ********* 以上是确定进出点流程 firstinTempvNodes lastinTempvNodes ***********
					
					//DONE 此處調用LKH算法 将确定起终点的获取TSP距离矩阵lkh.tsp,增加DMMMY点(距离起点,终点距离0;其它距离300000)
					//方法2：输出距离，起始终点到txt文档；LKH计算结果输出txt文档；读入txt文档
					if (vNodes.size() >= 0)
					{
						//cout << cluVRPinst_;						
						ofstream outFile;//创建了一个ofstream 对象
						outFile.open("lkh.tsp");
						outFile << "NAME : lkh" << endl;
						outFile << "COMMENT : lkh - temp city problem(Xu Rui)" << endl;
						outFile << "TYPE : TSP" << endl;
						outFile << "DIMENSION : " << TempvNodes.size()+1 << endl;
						outFile << "EDGE_WEIGHT_TYPE : EXPLICIT" << endl;
						outFile << "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << endl;
						outFile << "EDGE_WEIGHT_SECTION" << endl;
						// 增加DUMMY NODE 位置在最后一位
						for (unsigned int j = 0; j < TempvNodes.size()+1; j++)
						{
							for (unsigned int k = 0; k < TempvNodes.size()+1; k++)
							{
								if (j< TempvNodes.size())
								{
									if (k == TempvNodes.size())
									{
										if (j == firstinTempvNodes | j == lastinTempvNodes)
										{
											outFile << 0 << " ";
										}
										else
										{
											outFile << 300000 << " ";
										}
									}
									else
									{
										if (j != k)
											outFile << round(1000 * cluVRPinst_->getDistNodes(TempvNodes.at(j), TempvNodes.at(k))) << " ";
										if (j == k)
											outFile << 0 << " ";
									}
								}
								else
								{
									if ( k == firstinTempvNodes | k == lastinTempvNodes)
									{
										outFile << 0 << " ";
									}
									else
									{
										outFile << 300000 << " ";
									}
								}
							}
							outFile << endl;
						}	
						outFile << "EOF" << endl;
						outFile.close();

						//LKH1 - 允许LKH算法(从lkh.tsp读取数据,访问顺序结果输出到lkh.tour文件)
						int tourCost=0;
						runlkh(&tourCost);

						//LKH2 - 读取lkh.tour文件,将访问顺序读取到nodeSeq数组
						ifstream InFile;
						InFile.open("lkh.tour");
						string line;
						stringstream ss;
						int anum = -1;
						int idx = 1;
						vector<int> nodeSeq; //LHK求解的city顺序,放在此vector中
						vector<int> nodeSeqSort;
						if (!InFile.is_open()){
							exit(EXIT_FAILURE);
						}
						else
						{
							while (getline(InFile, line)) // line中不包括每行的换行符  
							{
								ss.clear(); ss << line; ss >> anum;
								if (idx!=1)  //排除第一个Bestcost值
									nodeSeq.push_back(anum-1); //将n个city的顺序给出,从0开始
								idx++; 
							}
						}
						InFile.close();


						//LKH3 - 修定nodeSeq顺序,最终确保从first开始不断进入到nodT中:nodT->addStop(TempvNodes.at(nodeSeq[i]));
						//如果vnodes是按first顺序进入 -> 第一个点的id==snodeSeq的第一个点的id OK
						if (TempvNodes[firstinTempvNodes]->id == TempvNodes[nodeSeq[0]]->id)
						{
							cout << " vnodes进入的第一个点的id是: " << TempvNodes[firstinTempvNodes]->id << endl;
							for (unsigned int i = 0; i < TempvNodes.size(); i++)
							{
								if (nodeSeq[i] < TempvNodes.size()) //如果找到Dummy点,其值是新增,大于TempvNodes尺寸,不往nodT中增加
								{
									nodT->addStop(TempvNodes.at(nodeSeq[i]));
									if (i == TempvNodes.size() - 1) {
										if (TempvNodes[lastinTempvNodes]->id != TempvNodes[nodeSeq[i]]->id) {
											cout << " EEError " << TempvNodes[lastinTempvNodes]->id << "   " << TempvNodes[nodeSeq[i]]->id;
										}											
									}										
								}
								else
								{
									cout << " vnodes增加的DMMMY点id是: " << TempvNodes[nodeSeq[i]]->id << endl;
								}
								cout << "   " << nodeSeq[i];
							}
						}
						else
						{
							signed int firstinvNodes = -1;
							//找出第一个节点对应的i值
							for (unsigned int i = 0; i < TempvNodes.size()+1; i++)
							{
								if (firstinTempvNodes == nodeSeq[i])
									firstinvNodes = i;
							}
							if (firstinvNodes<0)
								cout << " ERROR 未赋值  " << firstinvNodes;
							//cout << "   " << firstinvNodes;
							//如果第一个节点对应的下一个节点为dummy节点 // dummy点在frist点的后面(若first是最后一个,不可能有此情况)

							if (firstinvNodes < TempvNodes.size() && nodeSeq[firstinvNodes+1]>= TempvNodes.size())
							{
								for (signed int i = firstinvNodes; i >= 0; i--)
								{
									nodeSeqSort.push_back(nodeSeq.at(i));
									//nodeSeqSort.push_back(i);
								}
								for (unsigned int i = TempvNodes.size(); i > firstinvNodes+1; i--)
								{
									nodeSeqSort.push_back(nodeSeq.at(i));
									//nodeSeqSort.push_back(i);
								}
							}
							else if(firstinvNodes > 0 && nodeSeq[firstinvNodes -1] >= TempvNodes.size()) // dummy点在frist点的前面(若first是第一个,不可能有此情况)
							{
								for (unsigned int i = firstinvNodes; i < TempvNodes.size()+1; i++)
								{
									nodeSeqSort.push_back(nodeSeq.at(i));
								}
								for (unsigned int i = 0; i < firstinvNodes-1; i++)
								{
									nodeSeqSort.push_back(nodeSeq.at(i));
								}
							}
							else
							{
								cout << " eeee ";
							}
							
							if (TempvNodes[firstinTempvNodes]->id == TempvNodes[nodeSeqSort[0]]->id && TempvNodes[lastinTempvNodes]->id == TempvNodes[nodeSeqSort[TempvNodes.size()-1]]->id)
							{
								for (unsigned int i = 0; i < TempvNodes.size(); i++)
								{
									nodT->addStop(TempvNodes.at(nodeSeqSort[i]));
								}
								//cout << " vnodes进入的第点的id是: " << TempvNodes.at(nodeSeqSort[i])->id << endl;
								//对其排序 
								//cout << "   " << nodeSeq[0];
								//cout << "   " << TempvNodes[firstinTempvNodes]->id << "   " << TempvNodes[nodeSeq[0]]->id;
							}
							else
							{
								cout << "发生顺序错误EEEEEEEEEEEEE   " << endl;
							}
						}
					}

					// ********************* LKH *************************

					//get next cluster to define the appropriate last node in the current cluster (closest to a node in the next cluster)
					//2.2 找出距离下一cluster最近的当前cluster中的最佳点作为当前聚类的last点,但并不addStop进入nodT(类似预处理聚类间距离)
					//std::vector<Node*> vNodesNext = cluT->getCluster(i + 1)->getvNodesPtr();
					//minDist = BIG_M;
					//int last;
					//for (unsigned int j = 0; j < vNodes.size(); j++)
					//{
					//	for (unsigned int k = 0; k < vNodesNext.size(); k++)
					//	{
					//		//cout << vNodes[j]->id << " " << cluVRPinst_->getDistNodes(TempvNodes.at(j), vNodesNextTemp.at(k)) << " ";
					//		if (cluVRPinst_->getDistNodes(vNodes.at(j), vNodesNext.at(k)) < minDist )
					//		{
					//			last = j;
					//			minDist = cluVRPinst_->getDistNodes(vNodes.at(j), vNodesNext.at(k));
					//		}
					//	}
					//}
					//Node* lastNode = vNodes.at(last);		//最后节点暂放到lastNode
					//vNodes.erase(vNodes.begin() + last);
					//cout << last << " " << vNodes[last]->id << " " << minDist << endl; // vNodes.at(last)

					// get Node* n here is vNodes.at(j) at：第几个的具体索引号
					
					//cout << firstinTempvNodes << " " << TempvNodes[firstinTempvNodes]->id << endl; // vNodes.at(first)
					//cout << lastinTempvNodes << " " << TempvNodes[lastinTempvNodes]->id << endl; // vNodes.at(last)

					//cout << first << " " << TempvNodes[first]->id << " " << minDist << endl; // vNodes.at(first)
					

					//cout << GlobalBestCost;

					

					//TODO 此處調用LKH算法
					//方法1：基于坐标，起始终点到txt文档；LKH计算结果输出txt文档；读入txt文档
					//if (vNodes.size() >= 0)
					//{
					//	//cout << cluVRPinst_;						
					//	ofstream outFile;//创建了一个ofstream 对象
					//	outFile.open("lkh.tsp");
					//	outFile << "NAME : lkh" << endl;
					//	outFile << "COMMENT : lkh - temp city problem(Xu Rui)" << endl;
					//	outFile << "TYPE : TSP" << endl;
					//	outFile << "DIMENSION : " << TempvNodes.size() << endl;
					//	outFile << "EDGE_WEIGHT_TYPE : EUC_2D" << endl;
					//	outFile << "NODE_COORD_SECTION" << endl;
					//	for (unsigned int j = 0; j < TempvNodes.size(); j++)
					//	{
					//		outFile << j+1 << " "<< TempvNodes[j]->x << " " << TempvNodes[j]->y;
					//		// << TempvNodes.at(j); TempvNodes[j]->id
					//		outFile << endl;
					//	}
					//	outFile << "EOF" << endl;
					//	outFile.close();

					//	int re;		re = runlkh();	std::cout << "results is = "  << endl;
					//}

					//std::vector<std::vector<double>> m = cluVRPinst_->getDistNodesMatrix();
					//vector<vector<double> >::iterator it1;
					//vector<double>::iterator it2;
					//for (it1 = m.begin(); it1 != m.end(); ++it1)
					//{
					//	for (it2 = it1->begin(); it2 != it1->end(); ++it2)
					//	{
					//		outFile << *it2 << " ";
					//	}
					//}
					//add optimal sequence to the trip
					//2.3 排除first和last后,找出余下点的next点(从first出发最近的),并addStop进入nodT;
					//while (vNodes.size() > 0)
					//{
					//	minDist = BIG_M;
					//	int next;
					//	//循环找出距离当前node最近的节点next
					//	for (unsigned int j = 0; j < vNodes.size(); j++)
					//	{
					//		if (cluVRPinst_->getDistNodes(nodT->getLastPosition(), vNodes.at(j)) < minDist)
					//		{
					//			next = j;
					//			minDist = cluVRPinst_->getDistNodes(nodT->getLastPosition(), vNodes.at(j));
					//		}
					//	}
					//	
					//	nodT->addStop(vNodes.at(next));
					//	vNodes.erase(vNodes.begin() + next);
					//}

					//2.4 最后addStop最后的last点,完成该nodT
					//nodT->addStop(lastNode);
				}
			}
		}

		//计算;nodT对应的trip距离(不同与距离的distance)
		nodT->calcDistance();
		nodSol->addTrip(nodT);
	}

	return nodSol;
}

/********************************************************************************/
/********************************************************************************/

NodeSolution::NodeSolution(){}

NodeSolution::NodeSolution(CluVRPinst* cluVRPinst):Solution(cluVRPinst)
{
	this->totalDist_ = 0;
}

NodeSolution::NodeSolution(CluVRPinst* cluVRPinst, int d) : Solution(cluVRPinst)
{
	this->totalDist_ = d;
}

NodeSolution::NodeSolution(NodeSolution* copy):Solution(copy->cluVRPinst_)
{
	for (int i = 0; i < copy->getnTrips(); i++)
	{
		NodTrip* t = new NodTrip(copy->getTrip(i));
		this->addTrip(t);
	}

	this->totalDist_ = copy->totalDist_;
}

NodeSolution::~NodeSolution()
{
	for (unsigned int i = 0; i < vTrips_.size(); i++) delete vTrips_.at(i);
	vTrips_.clear();
}

void NodeSolution::addTrip(NodTrip* t)
{	
	this->vTrips_.push_back(t);
	this->totalDist_ += t->getDistance();
}

double NodeSolution::calcTotalDist(void)
{
	totalDist_ = 0;

	for (int i = 0; i < this->getnTrips(); i++)
	{
		totalDist_ += vTrips_.at(i)->getDistance();
	}

	return totalDist_;
}

bool NodeSolution::evaluate(NodeSolution*& best)
{
	if (this->totalDist_ < best->totalDist_)
	{
		delete best;		
		best = this;
		return true;
	}
	else
	{
		return false;
	}
}

ClusterSolution* NodeSolution::convert(void)
{
	ClusterSolution* s = new ClusterSolution(cluVRPinst_);

	for (unsigned int i = 0; i < vTrips_.size(); i++)
	{
		NodTrip* nT = vTrips_.at(i);
		CluTrip* cT = new CluTrip(cluVRPinst_);
		
		int c = nT->getvNodes().front()->cluster;		//c is first cluster
		cT->addStop(cluVRPinst_->getCluster(c));
		
		//loop through all nodes, if cluster changes -> add new cluster
		for (int j = 1; j < nT->getSize(); j++)
		{
			if (nT->getNode(j)->cluster == c) continue;

			c = nT->getNode(j)->cluster;
			cT->addStop(cluVRPinst_->getCluster(c));
		}

		s->addTrip(cT);
	}

	return s;
}