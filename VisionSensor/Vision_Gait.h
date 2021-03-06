#ifndef VISION_GAIT0_H
#define VISION_GAIT0_H

#include <Basic_Gait.h>

#define N_CALIBRATION 70
using namespace aris::dynamic;

enum robotMove
{
    nomove = 0,
    turn = 1,
    flatmove = 2,
    bodymove = 3,
    stepup = 4,
    stepdown = 5,
    stopmove = 6,
};

struct VISION_WALK_PARAM
{
    int count = 0;
    robotMove movetype = nomove;
    int totalCount = 2000;
    double turndata = 0;
    double movedata[3] = {0, 0, 0};
    double bodymovedata[3] = {0, 0, 0};
    double stepupdata[6] = {0, 0, 0, 0, 0, 0};
    double stepdowndata[6] = {0, 0, 0, 0, 0, 0};
};

struct VISION_TREEPASS_PARAM
{
    int count = 0;
    int totalCount = 2000;
    int orentation = 0;
    double alpha=0;
    double stepDis=0;
    int stepNumber=6;
    double beta=0;
};

struct VISION_CALIBRATION_PARAM final:public aris::server::GaitParamBase
{
    int gaitLength=2000;// from zero to the targeting posture
    int localCount=0;
    int postureNum=N_CALIBRATION;
    int postureCount=0;
    double postureLib[6*N_CALIBRATION];
};

int RobotVisionWalk(Robots::RobotTypeIII &robot, const VISION_WALK_PARAM &param);

int RobotVisionWalkForTreePass(Robots::RobotTypeIII &robot, const VISION_TREEPASS_PARAM &param);

int RobotBody(Robots::RobotTypeIII &robot, int count, float bodymovedata[3]);

int RobotStepUp(Robots::RobotTypeIII &robot, int count, float stepheight);

int RobotStepDown(Robots::RobotTypeIII &robot, int count, float stepdetph);

#endif // VISION_GAIT0_H
