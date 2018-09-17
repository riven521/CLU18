#include "Solution.h"
#include "Heuristic.h"
#include <iostream>

using namespace std;

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
				if ((double)rand() / RAND_MAX < Params::RANDOM_CONVERSION)
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

					nodT->addStop(vNodes.at(first));
					vNodes.erase(vNodes.begin() + first);

					//get next cluster to define the appropriate last node in the current cluster (closest to a node in the next cluster)
					//2.2 找出距离下一cluster最近的当前cluster中的最佳点作为当前聚类的last点,但并不addStop进入nodT(类似预处理聚类间距离)
					std::vector<Node*> vNodesNext = cluT->getCluster(i + 1)->getvNodesPtr();
					minDist = BIG_M;
					int last;
					for (unsigned int j = 0; j < vNodes.size(); j++)
					{
						for (unsigned int k = 0; k < vNodesNext.size(); k++)
						{
							if (cluVRPinst_->getDistNodes(vNodes.at(j), vNodesNext.at(k)) < minDist)
							{
								last = j;
								minDist = cluVRPinst_->getDistNodes(vNodes.at(j), vNodesNext.at(k));
							}
						}
					}

					Node* lastNode = vNodes.at(last);		//最后节点暂放到lastNode
					vNodes.erase(vNodes.begin() + last);

					//TODO 此處調用LKH算法
					//方法1：输出坐标，起始终点到txt文档；LKH计算结果输出txt文档；读入txt文档


					//add optimal sequence to the trip
					//2.3 排除first和last后,找出余下点的next点(从first出发最近的),并addStop进入nodT;
					while (vNodes.size() > 0)
					{
						minDist = BIG_M;
						int next;
						//循环找出距离当前node最近的节点next
						for (unsigned int j = 0; j < vNodes.size(); j++)
						{
							if (cluVRPinst_->getDistNodes(nodT->getLastPosition(), vNodes.at(j)) < minDist)
							{
								next = j;
								minDist = cluVRPinst_->getDistNodes(nodT->getLastPosition(), vNodes.at(j));
							}
						}

						nodT->addStop(vNodes.at(next));
						vNodes.erase(vNodes.begin() + next);
					}

					//2.4 最后addStop最后的last点,完成该nodT
					nodT->addStop(lastNode);
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