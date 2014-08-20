// WalkRay.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "renderEngine.h"
using namespace LFJ;

int _tmain(int argc, _TCHAR* argv[])
{
	RenderEngine engine;
	engine.window();
	engine.waitCmd();
	return 0;
}

