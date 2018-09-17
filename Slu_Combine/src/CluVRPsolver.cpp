#include "CluVRPsolver.h"

#include <time.h>

CluVRPsolver::CluVRPsolver()
{
	params_ = new Params();	//默认全局参数赋值

	sCluCurrent_ = nullptr;
	sCluBest_ = nullptr;
	sNodCurrent_ = nullptr;
	sNodBest_ = nullptr;
}

CluVRPsolver::~CluVRPsolver()
{
	delete params_;
}

void CluVRPsolver::showSol(ClusterSolution* sCluCurrent)
{
	//555 自增: 解的展示sCluCurret 属于ClusterSolution
	cout << endl;
	cout << "本次算例的聚类解为：" << endl;
	cout << "聚类解总线路数= " << sCluCurrent_->getnTrips() << "    ";
	cout << "聚类解总距离= " << sCluCurrent_->getTotalDist() << "    " << endl;
	for (int i = 0; i < sCluCurrent_->getnTrips(); i++)
	{
		CluTrip* thisCluTrip = sCluCurrent_->getTrip(i);
		cout << "路线" << i + 1 << "总需求=" << thisCluTrip->getTotalDemand() << "  ";
		cout << "剩余容量= " << thisCluTrip->getSpareCapacity() << "  ";
		cout << "总距离= " << thisCluTrip->getDistance() << "  ";
		cout << "最后位置= " << thisCluTrip->getLastPosition()->getId() << "  " << endl;
		//cout << "路线" << i << "总size  =" << thisCluTrip->getSize() << endl;

		// 从每个trip的vClusters中循环访问聚类的Id,也可增加其它内容
		cout << "访问聚类顺序  ";
		for (int j = 0; j < thisCluTrip->getSize(); j++)
		{
			cout << thisCluTrip->getCluster(j)->getId() << "  ";
		}
		cout << endl;
	}
}

void CluVRPsolver::showSol(NodeSolution* sNodCurrent_)
{
	//555 自增: 解的展示sNodCurrent_ 属于NodeSolution
	cout << endl;
	cout << "本次算例的聚类转换为节点解为：" << endl;
	cout << "节点解总线路数= " << sNodCurrent_->getnTrips() << "    ";
	cout << "节点解总距离= " << sNodCurrent_->getTotalDist() << "    " << endl;
	for (int i = 0; i < sNodCurrent_->getnTrips(); i++)
	{
		NodTrip* thisNodTrip = sNodCurrent_->getTrip(i);
		cout << "路线" << i + 1 << "总需求=" << thisNodTrip->getTotalDemand() << "  ";
		cout << "剩余容量= " << thisNodTrip->getSpareCapacity() << "  ";
		cout << "总距离= " << thisNodTrip->getDistance() << "  ";
		cout << "最后位置= " << thisNodTrip->getLastPosition()->id << "  " << endl;
		//cout << "路线" << i << "总size  =" << thisNodTrip->getSize() << endl;

		// 从每个trip的vNodes中循环访问节点的id,也可增加其它内容
		cout << "访问节点顺序  ";
		for (int j = 0; j < thisNodTrip->getSize(); j++)
		{
			cout << thisNodTrip->getNode(j)->id << "  ";
		}
		cout << endl;
	}
}


CluVRPsol* CluVRPsolver::solve(CluVRPinst* cluVRPinst, Timer* timer)
{
	//特殊情况:如果车辆总容量<聚类总需求,即给定车辆不够,则instance是infeasible
	if (!cluVRPinst->isFeasible()) {
		std::cout << "instance infeasible" << std::endl; return nullptr;
	}

	//初始化:赋值空指针给cluVRPsol
	CluVRPsol* cluVRPsol = nullptr;

	//create methods
	BPheuristic_ = new BinPacking(cluVRPinst);

	//keep track of the best solutions
	//定义本类的私有,两个Best是我们最需要的
	sCluBest_ = nullptr;
	sNodBest_ = new NodeSolution(cluVRPinst, BIG_M);//获取最优NodeSolution:包含算例+totalDist_=BIG_M极大值; 
	
	sCluCurrent_ = new ClusterSolution(cluVRPinst);	//当前的ClusterSolution,应该暂时为空
	

	//try to construct feasible solution at cluster level
	//3.2;构建过程+试错过程,增加车辆数,直到BPheuristic_.run进行计算后返回Ture
	while (!BPheuristic_->run(sCluCurrent_)) 
	{
		delete sCluCurrent_; sCluCurrent_ = nullptr;
		cluVRPinst->increasenVeh();
		sCluCurrent_ = new ClusterSolution(cluVRPinst);
	}

	showSol(sCluCurrent_); //自增: solution结果展示
	// ************** 以上 完成聚类初始解的构建 **************** //

	//create methods
	cluVNS_ = new CluVNS(cluVRPinst);		//初始化cluVNS->nbhSequence为数组,0-6
	nodVNS_ = new NodVNS(cluVRPinst, false);
	diversOperator_ = new Diversification(cluVRPinst);
	
	//start local search
	bool goToNodeVNS = false;
	bool stoppingCriterion = false;
	int nIterationsWithoutImprovement = 0;
	double bestNodTour = BIG_M;

	// ************** 以上 初始化 **************** //

	do{
		//VNS1:将sCluCurrent_转换为VNS的sCluCurrent_,并重新计算目标距离值
		//		showSol(sCluCurrent_); //自增: solution结果展示
		cluVNS_->run(sCluCurrent_);//仅仅是获得一个新解		
		//		showSol(sCluCurrent_); //自增: solution结果展示

		//Convert;由sCluCurrent_获取对应的NodeSolution;sNodCurrent_
 		sNodCurrent_ = sCluCurrent_->convert(); //DONE: 将nod内由VNS转换为LHK算法获取最佳访问顺序
		 //自增: solution结果展示
		showSol(sCluCurrent_);
		showSol(sNodCurrent_); 

		// ************** 以上 VNS对初始CluCurrent求解 **************** //		
				
		if (sNodCurrent_->getTotalDist() < bestNodTour) {
			bestNodTour = sNodCurrent_->getTotalDist();
			sCluBest_ = sCluCurrent_; //没有node2cluster了,此处就是bestcluster
			sNodBest_ = sNodCurrent_; // LKH计算就是最好的,不需再evaluate对比了
			nIterationsWithoutImprovement = 0;
			//timer->setTimeBest(timer->getIntervalTime());
		}
		else
		{
			nIterationsWithoutImprovement++;
			if (nIterationsWithoutImprovement >= params_->N_NO_IMPROVEMENT) //check stopping criterion 5
			{
				stoppingCriterion = true;
				break;
			}
		}

		// ************** 以下 DIVERSFICATION对CluCurrent增加多样性 **************** //		

		goToNodeVNS = diversOperator_->run(sCluCurrent_);	//DIVERSIFICATION_1=1参数判断goToNodeVNS的类型,全部为false:go to cluVNS
		//showSol(sCluCurrent_);
		if (1) { //似乎为1更好->即考虑diversificatin也有可能获取更优的node解
			sNodCurrent_ = sCluCurrent_->convert();	//
			//showSol(sNodCurrent_);
			if (sNodCurrent_->getTotalDist() < bestNodTour) {
				bestNodTour = sNodCurrent_->getTotalDist();
				sCluBest_ = sCluCurrent_; //没有node2cluster了,此处就是bestcluster
				sNodBest_ = sNodCurrent_; // LKH计算就是最好的,不需再evaluate对比了
				nIterationsWithoutImprovement = 0;
				showSol(sCluBest_);
				showSol(sNodBest_);
				//timer->setTimeBest(timer->getIntervalTime());
			}
		}

		//do
		//{			
		//	//VNS2:将sNodCurrent_转换为VNS的sNodCurrent_
		//	//nodVNS_->run(sNodCurrent_);	//如采用LK算法，将不需要该函数

		//	//evaluate node solution 评估当前sNodCurrent_是否小于sNodBest_,如是则同时更新sCluBest_+nIterationsWithoutImprovement
		//	if (sNodCurrent_->evaluate(sNodBest_))	//555 获取best:sNodBest_和sCluBest_
		//	{
		//		timer->setTimeBest(timer->getIntervalTime());
		//		//keep also the cluster variant of the best node solution
		//		if (sCluBest_ != nullptr) delete sCluBest_;
		//		sCluBest_ = sNodBest_->convert();	// TODO 如采用nodVNS,则还不要不断的转换,如转换后结果不好,还要进行各种改进,还 不如永LHK算法的好 TODO
				//似乎没有必要了,虽然node层面有cluster的vns作用,但cluster层面的也都包含了;可以remove
		//		showSol(sCluBest_); //自增: solution结果展示
		//		showSol(sNodBest_); //自增: solution结果展示
		//		//reset counter
		//		nIterationsWithoutImprovement = 0;

		//		nodVNS_->disableAdditionalNBHs();
		//	}
		//	else  //无改进则增加nIterationsWithoutImprovement;再重新VNS1循环
		//	{
		//		delete sNodCurrent_; sNodCurrent_ = nullptr;

		//		nIterationsWithoutImprovement++;
		//		
		//		if (nIterationsWithoutImprovement % 100 == 0)	//every 100th iteration
		//		//if (nIterationsWithoutImprovement + 1 == params_->N_NO_IMPROVEMENT)	//the last iteration
		//		{
		//			//use all the power we have to improve the current best solution
		//			sNodCurrent_ = new NodeSolution(sNodBest_);
		//			nodVNS_->enableAdditionalNBHs();	//try complex neighbourhoods
		//			goToNodeVNS = true;
		//			continue;
		//		}				
		//		else if (nIterationsWithoutImprovement >= params_->N_NO_IMPROVEMENT) //check stopping criterion
		//		{
		//			stoppingCriterion = true;
		//			break;
		//		}
		//		else
		//			nodVNS_->disableAdditionalNBHs();
		//	}
		//	
		//	//diversification before new round of improvement
		//	//Diversification阶段:无论是否有改进,都执行如下操作.
		//	goToNodeVNS = diversOperator_->run(sCluCurrent_);	//DIVERSIFICATION_1=1参数判断goToNodeVNS的类型,全部为false:go to cluVNS,原作者全部返回的是false
		//	if(goToNodeVNS) sNodCurrent_ = sCluCurrent_->convert();	//如果为true,则将Clu转换为Nodes,循环VNS2;为false:则循环VNS1;

		//} while (goToNodeVNS);

	//} while (timer->getIntervalTime() < CALC_TIME);
	} while (!stoppingCriterion);

	//free memory
	delete BPheuristic_;
	delete cluVNS_;
	delete nodVNS_;
	delete diversOperator_;
	delete sCluCurrent_;

	//create solution object
	//获取最终的解:cluVRPsol:包含两个子解
	cluVRPsol = new CluVRPsol(sCluBest_, sNodBest_);
	return cluVRPsol;	
}