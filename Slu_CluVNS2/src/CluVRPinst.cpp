#include "CluVRPinst.h"
#include <iostream>



//**********************************************************************************************//

CluVRPinst::CluVRPinst(std::vector<Cluster> vClusters, int nVehicles)
{
	this->nVehicles_ = nVehicles;							//输入的车辆数为可用车辆数
	this->vehicleCapacity_ = Globals::vehicleCapacity;		//车辆容量是相同的

	this->vClusters_ = vClusters;
	this->nClusters_ = vClusters.size();
}

CluVRPinst::~CluVRPinst(){}

// 获取排除Depot外的聚类
std::vector<Cluster*> CluVRPinst::getClientClusters(void)
{
	std::vector<Cluster*> v;
	for (int i = 0; i < nClusters_; i++)
	{
		if(!vClusters_.at(i).isDepot) v.push_back(&vClusters_.at(i));
	}
	return v;
}
// 如有多个depot,随机选择一个
Cluster* CluVRPinst::getRandomDepot(void)
{
	std::vector<Cluster*> v;

	for (int i = 0; i < nClusters_; i++)
	{
		if (vClusters_.at(i).isDepot) v.push_back(&vClusters_.at(i));
	}

	return v.at(rand() % v.size());//随机选择一个
}

// 简单判定车辆数量是否足以满足总需求
bool CluVRPinst::isFeasible(void)
{
	int totalDemand = 0;
	
	for (int i = 0; i < nClusters_; i++)
	{
		totalDemand += vClusters_.at(i).getDemand();
	}

	return totalDemand <= (nVehicles_ * vehicleCapacity_);
}


// 新增符号<<的重载
template <typename TElem>
ostream& operator<<(ostream& os, const vector<TElem>& vec) {
	typedef vector<TElem>::const_iterator iter_t;
	const iter_t iter_begin = vec.begin();
	const iter_t iter_end = vec.end();
	os << "[";
	for (iter_t iter = iter_begin; iter != iter_end; ++iter) {
		cout << ((iter != iter_begin) ? "," : "") << *iter;
	}
	os << "]" << endl;
	return os;
}

ostream& operator<<(ostream& os, const CluVRPinst* cluVRPinst)
{
	cout << "本次算例如下：" << endl;
	cout << "车辆数：" << cluVRPinst->nClusters_ << " ";
	cout << "车辆容量:" << cluVRPinst->vehicleCapacity_ << " ";
	//cout << "车辆足够?:" << cluVRPinst->isFeasible << " ";
	cout << "聚类数:" << cluVRPinst->nClusters_ << endl << endl;
	

	cout << "节点间距离为:" << endl;
	cout << cluVRPinst->distNodes_ << endl;

	cout << "聚类间Handoff距离为:" << endl;
	cout << cluVRPinst->distClusters_ << endl;

	cout << "实现了:迭代输出每个聚类:" << endl;
	//cout << (cluVRPinst->vClusters_) << endl;
	vector<Cluster>::const_iterator it = cluVRPinst->vClusters_.begin();
	for (; it != cluVRPinst->vClusters_.end(); ++it)
	{
		//cout << "聚类ID：" << it->show() << endl;
		cout << "聚类ID：" << it->getId() << " ";
		cout << "聚类内节点数:" << it->getnNodes() << " ";
		cout << "聚类内Demand:" << it->getDemand() << " ";
		cout << "聚类是否depot:" << it->isDepot <<  " ";
		cout << "聚类内节点信息未输出" << endl;		
	}
	return os;
}

