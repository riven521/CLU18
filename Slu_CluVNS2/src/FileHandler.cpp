#include "FileHandler.h"
//#include "Globals.h"

using namespace std;

std::string FileHandler::currentInstanceName_;

void FileHandler::createFilePathNextInstance(void)
{
	char temp[30];
	fscanf_s(fpo_, "%s", &temp, sizeof(temp));
	std::ostringstream temp2;
	temp2 << temp;
	currentInstanceName_ = temp2.str();

	// 如果overview文件遇到STOP符号
	if (currentInstanceName_ == "STOP")
	{
		system("pause");	//增加“按任意键继续”
		exit(0);
	}

	std::ostringstream buff;

	if (setSelection_ == 0)
	{
		buff << "\\instances\\CluVRP2\\" << currentInstanceName_;
	}
	else if (setSelection_ == 1)
	{
		buff << "\\instances\\instLargeCluVRP\\" << currentInstanceName_;
	}
	else if (setSelection_ == 2)
	{
		buff << "\\instances\\tuning\\" << currentInstanceName_;
	}
	else if (setSelection_ == 3)
	{
		buff << "\\instances\\GoldenWasilKellyAndChao-0.1\\" << currentInstanceName_;
	}
	else if (setSelection_ == 4)
	{
		buff << "\\instances\\GoldenWasilKellyAndChao-0.25\\" << currentInstanceName_;
	}
	else if (setSelection_ == 5)
	{
		buff << "\\instances\\GoldenWasilKellyAndChao-0.5\\" << currentInstanceName_;
	}
	else if (setSelection_ == 6)
	{
		buff << "\\instances\\GoldenWasilKellyAndChao-0.75\\" << currentInstanceName_;
	}
	else if (setSelection_ == 7)
	{
		buff << "\\instances\\GoldenWasilKellyAndChao-1.0\\" << currentInstanceName_;
	}
	else if (setSelection_ == 8)
	{
		buff << "\\instances\\SimpleInstance\\" << currentInstanceName_;
	}
	char cwd[300];
	filePathInstance_ = _getcwd(cwd, 300) + buff.str(); //_getcwd:获取当前working目录路径 from direct.h
}

void FileHandler::createFilePathSolutionOverview(void)
{
	std::ostringstream buff;
	if (setSelection_ == 0)
	{
		buff << "\\solutions\\sol-CluVRP2-Full" << CALC_TIME <<".txt";
	}
	else if (setSelection_ == 1)
	{
		buff << "\\solutions\\sol-instLargeCluVRP-Full" << CALC_TIME << ".txt";
	}
	else if (setSelection_ == 2)
	{
		buff << "\\solutions\\sol-tuning-Full" << CALC_TIME << ".txt";
	}
	else if (setSelection_ == 3)
	{
		buff << "\\solutions\\sol-golden-0.1-Full" << CALC_TIME << ".txt";
	}
	else if (setSelection_ == 4)
	{
		buff << "\\solutions\\sol-golden-0.25-Full" << CALC_TIME << ".txt";
	}
	else if (setSelection_ == 5)
	{
		buff << "\\solutions\\sol-golden-0.5-Full" << CALC_TIME << ".txt";
	}
	else if (setSelection_ == 6)
	{
		buff << "\\solutions\\sol-golden-0.75-Full" << CALC_TIME << ".txt";
	}
	else if (setSelection_ == 7)
	{
		buff << "\\solutions\\sol-golden-1.0-Full" << CALC_TIME << ".txt";
	}
	else if (setSelection_ == 8)
	{
		buff << "\\solutions\\sol-SimpleInstance-Full" << CALC_TIME << ".txt";
	}

	char cwd[300];
	filePathSolutionOverview_ = _getcwd(cwd, 300) + buff.str();

	txtSolOverview_.open(filePathSolutionOverview_);
}

void FileHandler::createTrack(void)
{
	std::ostringstream buff;
	if (setSelection_ == 0)
	{
		buff << "\\solutions\\track-CluVRP2.txt";
	}
	else if (setSelection_ == 1)
	{
		buff << "\\solutions\\track-instLargeCluVRP.txt";
	}

	char cwd[300];
	filePathTracker_ = _getcwd(cwd, 300) + buff.str();

	txtTrack_.open(filePathTracker_);
}

void FileHandler::initOverview(void)
{
	string instSet;
	bool again;

	do{
		again = false;

		if (__argc == 1)
		{
			cout << "	***	Welcome to the colTSPTW - solver.	***" << endl << endl << "Select the set of test instances you want to run :" << endl << \
				"0 = CluVRP2 (small)" << endl << \
				"1 = instLargeCluVRP (large)" << endl << \
				"2 = tuning" << endl << \
				"3 = GoldenWasilKellyAndChao-0.1" << endl << \
				"4 = GoldenWasilKellyAndChao-0.25" << endl << \
				"5 = GoldenWasilKellyAndChao-0.5" << endl << \
				"6 = GoldenWasilKellyAndChao-0.75" << endl << \
				"7 = GoldenWasilKellyAndChao-1.0" << endl << \
				"8 = SimpleInst" << endl << endl << \
				"Press 9 to quit" << endl << endl;

			setSelection_ = 0;	//不用输入,自动选择8
			//scanf_s("%d", &setSelection_);
		}
		else
		{
			if (!sscanf_s(__argv[1], "%i", &setSelection_)) { cout << "ERROR while reading argv"; };
		}

		if (setSelection_ == 0)
		{
			instSet = "CluVRP2";
		}
		else if (setSelection_ == 1)
		{
			instSet = "instLargeCluVRP";
		}
		else if (setSelection_ == 2)
		{
			instSet = "tuning";
			Params::TUNING = true;
		}
		else if (setSelection_ == 3)
		{
			instSet = "GoldenWasilKellyAndChao-0.1";
		}
		else if (setSelection_ == 4)
		{
			instSet = "GoldenWasilKellyAndChao-0.25";
		}
		else if (setSelection_ == 5)
		{
			instSet = "GoldenWasilKellyAndChao-0.5";
		}
		else if (setSelection_ == 6)
		{
			instSet = "GoldenWasilKellyAndChao-0.75";
		}
		else if (setSelection_ == 7)
		{
			instSet = "GoldenWasilKellyAndChao-1.0";
		}
		else if (setSelection_ == 8)
		{
			instSet = "SimpleInstance";
		}
		else if (setSelection_ == 9)
		{
			exit(0);
		}
		else
		{
			cout << "ERROR no valid instance selected.  Choose again.";
			again = true;
		}
	} while (again);

	std::ostringstream buff;
	buff << "\\instances\\" << instSet << "\\overview.txt";
	char cwd[300];
	filePathOverview_ = _getcwd(cwd, 300) + buff.str(); //_getcwd:获取当前working目录路径 from direct.h

	if (fopen_s(&fpo_, filePathOverview_.c_str(), "r")) { cout << "ERROR overview file could not be found"; }
}

void FileHandler::initOutputFiles(void)
{
	createFilePathSolutionOverview(); //输出结果到solutions文件夹
	// createTrack(); //输出track结果 但作者似乎还未完善
}

FileHandler::FileHandler()
{
	this->initOverview();		//输入文件读取从instance中
	this->initOutputFiles();	//输出文件到solution中
}

FileHandler::~FileHandler()
{
	fclose(fpo_);				//close overview file
	txtSolOverview_.close();	//close solution overview file
	txtTrack_.close();			//close track file

	delete fpo_;
}	

std::string FileHandler::getFilePathInstance(void)
{
	createFilePathNextInstance();
	return filePathInstance_;
}

std::string FileHandler::getCurrentInstanceName(void)
{
	return currentInstanceName_;
}

int FileHandler::getSetSelection(void) const
{
	return setSelection_;
}