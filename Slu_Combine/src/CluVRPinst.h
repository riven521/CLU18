//重要:输入数据CluVRPinst的形式(如修改算例需调整该文件)
#pragma once
#include <vector>

#include "Cluster.h"
//#include <iostream>
using namespace std;

class CluVRPinst
{
private:
	int nVehicles_;			//车数量
	int vehicleCapacity_;	//车容量

	std::vector<std::vector<double>> distNodes_;	//节点间距离 由setDistNodes
	std::vector<std::vector<double>> distClusters_;	//聚类间距离 由setDistClusters

	std::vector<Cluster> vClusters_;	//聚类向量vector
	int nClusters_;						//聚类数

public:
	CluVRPinst(std::vector<Cluster>, int);
	~CluVRPinst();

	// 自增：方便输出结果 : 参考Overloading the << Operator for Your Own Classes from https://msdn.microsoft.com/en-us/library/1z2f6c2k.aspx
	friend ostream& operator<<(ostream& os, const CluVRPinst* cluVRPinst);

	//getters
	inline double getDistNodes(Node* a, Node* b) const
	{
		return this->distNodes_.at(a->id - 1).at(b->id - 1);
	}
	inline double getDistClusters(Cluster* a, Cluster* b) const
	{
		return this->distClusters_.at(a->getId()).at(b->getId());
	}
	inline int getnVehicles(void) const
	{
		return nVehicles_;
	}
	inline int getnClusters(void) const
	{
		return nClusters_;
	}
	inline int getVehicleCapacity(void) const
	{
		return vehicleCapacity_;
	}
	std::vector<Cluster*> getClientClusters(void); // 获取排除Depot外的聚类
	Cluster* getRandomDepot(void);
	inline Cluster* getCluster(int id)
	{
		return &this->vClusters_.at(id);
	}
	inline std::vector<std::vector<double>> getDistNodesMatrix(void) const
	{
		return distNodes_;
	}

	//setters
	inline void setDistNodes(std::vector<std::vector<double>> m)
	{
		distNodes_ = m;
	}
	inline void setDistClusters(std::vector<std::vector<double>> m)
	{
		distClusters_ = m;
	}

	inline void increasenVeh(void)
	{
		nVehicles_++;
	}
	 bool isFeasible(void);
	 //void calculateIntraClusterTSP(void);
 };
