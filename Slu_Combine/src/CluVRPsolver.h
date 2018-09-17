//勿动:主程序,流程图
#pragma once
#include "CluVRPinst.h"
#include "CluVRPsol.h"
#include "Globals.h"
#include "Solution.h"
#include "Heuristic.h"
#include "VNS.h"
#include "Timer.h"

class CluVRPsolver
{
private:
	Params* params_;

	//solutions
	ClusterSolution* sCluCurrent_;
	ClusterSolution* sCluBest_;
	NodeSolution* sNodCurrent_;
	NodeSolution* sNodBest_;

	//methods
	BinPacking* BPheuristic_;
	CluVNS* cluVNS_;
	NodVNS* nodVNS_;
	Diversification* diversOperator_;

public:
	CluVRPsolver();
	~CluVRPsolver();

	void showSol(ClusterSolution*);
	void showSol(NodeSolution*);
	CluVRPsol* solve(CluVRPinst*, Timer*);
};