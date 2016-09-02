#include "Vision_Gait0.h"
#include <string.h>
#include <math.h>
#include <iostream>

#ifndef PI
#define PI 3.141592653589793
#endif

using namespace std;

int RobotVisionWalk(Robots::RobotBase &robot, const VISION_WALK_PARAM &param)
{
    Robots::WalkParam wk_param;
    
    wk_param.alpha = param.alpha;
    wk_param.d = param.stepDis;
    wk_param.beta = param.beta;
    wk_param.h = 0.05;
    wk_param.n = param.stepNumber;
    wk_param.count = param.count;
    wk_param.totalCount = param.totalCount;
    
    return Robots::walkGait(robot, wk_param);
}

