//勿动:较为简单,最终返回解的形式,包含两个解solution;clustersolution+nodeSolution
#pragma once
#include "Solution.h"
//该类比较简单,一个总体solution包含ClusterSolution和NodeSolution两个单独的solution构成
struct CluVRPsol
{
	ClusterSolution* sCluster_;
	NodeSolution* sNode_;

	CluVRPsol(ClusterSolution*, NodeSolution*);
	~CluVRPsol();
};

