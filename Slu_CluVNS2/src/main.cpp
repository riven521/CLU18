#include "FileHandler.h"
#include "CluVRPsolver.h"
#include "CluVRPinst.h"
#include "CluVRPsol.h"
#include "Reader.h"
#include "Cluster.h"
#include "Timer.h"
#include "Printer.h"

using namespace std;

int main(void)
{



	//srand((unsigned)time(NULL));	//确保每次随机数的随机性

	CluVRPsolver* cluVRPsolver = new CluVRPsolver();
	CluVRPinst* cluVRPinst = nullptr; //算例变量赋空指针
	CluVRPsol* cluVRPsol = nullptr;
	Timer* timer = new Timer();
	Printer* print = new Printer();
	
	//select the overview file and put all necessary files in place
	FileHandler* fileHandler = new FileHandler();
	//open reader for the instance set selected
	Reader* reader = new Reader(fileHandler->getSetSelection());
	
	do {
		cluVRPinst = reader->read(fileHandler->getFilePathInstance()); //读取数据到cluVRPinst变量;Note 其中vClusters与distNodes等相对重要
		
		cout << cluVRPinst;

		for (int i = 0; i < Params::N_RUNS; i++)  //循环对某一个算例运行次数
		{
			timer->startClock();
			cluVRPsol = cluVRPsolver->solve(cluVRPinst, timer);  //求解该算例,参数1是算例;参数2为时间timer //Note cluVRPsol:其中sCluster;sNode包含solution和vtrips,最详细的解介绍
			timer->stopClock();
			
			if (cluVRPsol == nullptr)
				continue;
	
			//输出必要信息到文件 Note 必须建立solutions文件夹
			Printer::addToSolutionOverview(cluVRPsol, timer, fileHandler);
			Printer::nodeSolution(cluVRPsol, fileHandler);	
			Printer::cluster(cluVRPsol, timer);
			delete cluVRPsol;
		}

		delete cluVRPinst;

	} while (true); //for every instance

	delete fileHandler;
	delete reader;
	delete cluVRPsolver;
	delete timer;
	delete print;
	return 0;
}