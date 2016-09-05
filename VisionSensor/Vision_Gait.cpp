#include "Vision_Gait.h"

#include <string.h>
#include <math.h>
#include <iostream>

using namespace std;

void parseMoveWithRotate(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg)
{
    MoveRotateParam param;

    for(auto &i:params)
    {
        if(i.first=="u")
        {
            param.targetBodyPE213[0]=stod(i.second);
        }
        else if(i.first=="v")
        {
            param.targetBodyPE213[1]=stod(i.second);
        }
        else if(i.first=="w")
        {
            param.targetBodyPE213[2]=stod(i.second);
        }
        else if(i.first=="yaw")
        {
            param.targetBodyPE213[3]=stod(i.second)*PI/180;
        }
        else if(i.first=="pitch")
        {
            param.targetBodyPE213[4]=stod(i.second)*PI/180;
        }
        else if(i.first=="roll")
        {
            param.targetBodyPE213[5]=stod(i.second)*PI/180;
        }
        else if(i.first=="totalCount")
        {
            param.totalCount=stoi(i.second);
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
        }
    }

    msg.copyStruct(param);

    std::cout<<"finished parse"<<std::endl;
}

int moveWithRotate(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotTypeIII &>(model);
    auto &param = static_cast<const MoveRotateParam &>(param_in);

    static double beginBodyPE213[6];
    static double pEE[18];
    if(param.count==0)
    {
        robot.GetPeb(beginBodyPE213,"213");
        robot.GetPee(pEE);
    }

    double realBodyPE213[6];
    for(int i=0;i<6;i++)
    {
        double s = -(param.targetBodyPE213[i] / 2)*cos(PI * (param.count + 1) / param.totalCount ) + param.targetBodyPE213[i] / 2;
        realBodyPE213[i]=beginBodyPE213[i]+s; //target of current ms
    }

    double pBody[6];

    robot.SetPeb(realBodyPE213,"213");
    robot.SetWa(0);
    robot.SetPee(pEE);

    return param.totalCount - param.count - 1;
}

int RobotVisionWalkForTreePass(Robots::RobotTypeIII &robot, const VISION_TREEPASS_PARAM &param)
{
    Robots::Gait::WalkParam wk_param;

    wk_param.alpha = param.alpha;
    wk_param.d = -param.stepDis;
    wk_param.beta = param.beta;
    wk_param.h = 0.05;
    wk_param.n = param.stepNumber;
    wk_param.count = param.count;
    wk_param.totalCount = param.totalCount;

    return Robots::Gait::walkGait(robot, wk_param);
}

int RobotVisionWalk(Robots::RobotTypeIII &robot, const VISION_WALK_PARAM &param)
{
    Robots::Gait::WalkParam wk_param;

    switch(param.movetype)
    {
    case turn:
    {
        wk_param.alpha = 0;
        wk_param.beta = param.turndata * M_PI / 180;
        wk_param.d = 0;
        wk_param.h = 0.05;
    }
        break;
    case flatmove:
    {
        if(param.movedata[0] != 0)
        {
            if(param.movedata[0] > 0)
            {
                wk_param.alpha = -M_PI/2;
                wk_param.d = param.movedata[0];
            }
            else
            {
                wk_param.alpha = M_PI/2;
                wk_param.d = -param.movedata[0];
            }
            wk_param.beta = 0;
            wk_param.h = 0.05;
        }
        else
        {
            wk_param.alpha = M_PI;
            wk_param.beta = 0;
            wk_param.d = param.movedata[2];
            wk_param.h = 0.05;
        }
    }
        break;
    default:
        break;
    }

    wk_param.n = 1;
    wk_param.count = param.count;
    wk_param.totalCount = param.totalCount;

    return Robots::Gait::walkGait(robot, wk_param);
}

int RobotBody(Robots::RobotTypeIII &robot, int count, float bodymovedata[3])
{
    int totalCount = 2000;

    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
    static double beginPeb[6];

    if (count % totalCount == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
        robot.GetPeb(beginPeb, beginMak);
    }

    double Peb[6], Pee[18];
    std::copy(beginPeb, beginPeb + 6, Peb);
    std::copy(beginPee, beginPee + 18, Pee);

    double s = -(PI / 2)*cos(PI * (count + 1) / totalCount) + PI / 2;

    Peb[0] += bodymovedata[0] * (1 - cos(s))/2;
    Peb[1] += bodymovedata[1] * (1 - cos(s))/2;
    Peb[2] += bodymovedata[2] * (1 - cos(s))/2;

    robot.SetPeb(Peb, beginMak);
    robot.SetWa(0);
    robot.SetPee(Pee, beginMak);

    return totalCount - count - 1;
}

//int RobotStepUp(Robots::RobotBase &robot, int count, float stepheight)
//{
//    int period = 6;
//    int periodCount = 4500;
//    int pperiodCount = 4500 / 3;
//    int totalCount = 4500 * period;

//    double stepUpH = stepheight + 0.05;
//    double stepUpD = 0.325;

//    static aris::dynamic::FloatMarker beginMak{ robot.ground() };

//    static double beginPee[18];
//    static double inifootY;
//    static int inturn = 0;

//    if(count == 0)
//    {
//        beginMak.setPrtPm(*robot.body().pm());
//        beginMak.update();
//        robot.GetPee(beginPee, beginMak);
//        inifootY = beginPee[1];
//    }

//    double footOnGround = inifootY;
//    double footOnStep = inifootY + stepheight;

//    double footholdY[6][4]=
//    {
//        {footOnGround, footOnStep, footOnStep, footOnStep},
//        {footOnGround, footOnGround, footOnStep, footOnStep},
//        {footOnGround, footOnGround, footOnGround, footOnStep},
//        {footOnGround, footOnStep, footOnStep, footOnStep},
//        {footOnGround, footOnGround, footOnStep, footOnStep},
//        {footOnGround, footOnGround, footOnGround, footOnStep},
//    };

//    int stepNum = 0;

//    if(count < periodCount)
//    {
//        int localCount = count % periodCount;

//        double pBodyPE[6] = {0, 0, 0, 0, 0, 0};
//        double pEE[18];
//        memcpy(pEE, beginPee, sizeof(beginPee));

//        if(localCount < pperiodCount)
//        {
//            double s = -(PI / 2)*cos(PI * (localCount + 1) / pperiodCount) + PI / 2;

//            pEE[1 + 3 * inturn] = (stepUpH + footholdY[0 + 1 * inturn][stepNum]) * (1 - cos(s))/2;
//            pEE[7 + 3 * inturn] += (stepUpH + footholdY[2 + 1 * inturn][stepNum]) * (1 - cos(s))/2;
//            pEE[13 +  3 * inturn] += (stepUpH + footholdY[4 + 1 * inturn][stepNum]) * (1 - cos(s))/2;
//        }
//        else if(localCount >= pperiodCount && localCount < pperiodCount * 2)
//        {
//            double s = -(PI / 2)*cos(PI * (localCount + 1 - pperiodCount) / pperiodCount) + PI / 2;

//            pEE[1 + 3 * inturn] += (stepUpH + footholdY[0 + 1 * inturn][stepNum]);
//            pEE[7 + 3 * inturn] += (stepUpH + footholdY[2 + 1 * inturn][stepNum]);
//            pEE[13 +  3 * inturn] += (stepUpH + footholdY[4 + 1 * inturn][stepNum]);

//            pEE[2 + 3 * inturn] += stepUpD * (1 - cos(s))/2;
//            pEE[8 + 3 * inturn] += stepUpD * (1 - cos(s))/2;
//            pEE[14 + 3 * inturn] += stepUpD * (1 - cos(s))/2;

//            pBodyPE[2] += stepUpD/2 * (1 - cos(s))/2;
//        }
//        else
//        {
//            double s = -(PI / 2)*cos(PI * (localCount + 1 - 2 * pperiodCount) / pperiodCount) + PI / 2;

//            pEE[1 + 3 * inturn] += (stepUpH + footholdY[0 + 1 * inturn][stepNum])
//                    - (stepUpH - stepheight) * (1 - cos(s)) / 2;
//            pEE[7 + 3 * inturn] += (stepUpH + footholdY[2 + 1 * inturn][stepNum])
//                    - (stepUpH - stepheight) * (1 - cos(s)) / 2;
//            pEE[13 +  3 * inturn] += (stepUpH + footholdY[4 + 1 * inturn][stepNum])
//                    - (stepUpH - stepheight) * (1 - cos(s)) / 2;

//            pEE[2 + 3 * inturn] += stepUpD;
//            pEE[8 + 3 * inturn] += stepUpD;
//            pEE[14 + 3 * inturn] += stepUpD;

//            pBodyPE[2] += stepUpD/2;
//        }
//        if(count % periodCount == periodCount - 1)
//        {
//            if(inturn == 0)
//            {
//                inturn = 1;
//            }
//            else
//            {
//                inturn = 0;
//                stepNum++;
//            }
//        }
//        robot.SetPeb(pBodyPE, beginMak);
//        robot.SetPee(pEE, beginMak);
//    }
//    else if(count >= periodCount && count < periodCount * 5)
//    {
//        if(count % periodCount == 0)
//        {
//            beginMak.setPrtPm(*robot.body().pm());
//            beginMak.update();
//            robot.GetPee(beginPee, beginMak);
//        }

//        double pBodyPE[6] = {0, 0, 0, 0, 0, 0};
//        double pEE[18];
//        memcpy(pEE, beginPee, sizeof(beginPee));

//        int localCount = count % periodCount;

//        if(localCount < pperiodCount)
//        {
//            double s = -(PI / 2)*cos(PI * (localCount + 1) / pperiodCount) + PI / 2;

//            pEE[1 + 3 * inturn] += (stepUpH + footholdY[0 + 1 * inturn][stepNum]) * (1 - cos(s))/2;
//            pEE[7 + 3 * inturn] += (stepUpH + footholdY[2 + 1 * inturn][stepNum]) * (1 - cos(s))/2;
//            pEE[13 +  3 * inturn] += (stepUpH + footholdY[4 + 1 * inturn][stepNum]) * (1 - cos(s))/2;
//        }
//        else if(localCount >= pperiodCount && localCount < pperiodCount * 2)
//        {
//            double s = -(PI / 2)*cos(PI * (localCount + 1 - pperiodCount) / pperiodCount) + PI / 2;

//            pEE[1 + 3 * inturn] += (stepUpH + footholdY[0 + 1 * inturn][stepNum]);
//            pEE[7 + 3 * inturn] += (stepUpH + footholdY[2 + 1 * inturn][stepNum]);
//            pEE[13 +  3 * inturn] += (stepUpH + footholdY[4 + 1 * inturn][stepNum]);

//            pEE[2 + 3 * inturn] += 2 * stepUpD * (1 - cos(s))/2;
//            pEE[8 + 3 * inturn] += 2 * stepUpD * (1 - cos(s))/2;
//            pEE[14 + 3 * inturn] += 2 * stepUpD * (1 - cos(s))/2;

//            pBodyPE[2] += stepUpD * (1 - cos(s))/2;
//        }
//        else
//        {
//            double s = -(PI / 2)*cos(PI * (localCount + 1 - 2 * pperiodCount) / pperiodCount) + PI / 2;

//            pEE[1 + 3 * inturn] += (stepUpH + footholdY[0 + 1 * inturn][stepNum])
//                    - (stepUpH - footholdY[0 + 1 * inturn][stepNum + 1]) * (1 - cos(s)) / 2;
//            pEE[7 + 3 * inturn] += (stepUpH + footholdY[2 + 1 * inturn][stepNum])
//                    - (stepUpH - footholdY[2 + 1 * inturn][stepNum + 1]) * (1 - cos(s)) / 2;
//            pEE[13 +  3 * inturn] += (stepUpH + footholdY[4 + 1 * inturn][stepNum])
//                    - (stepUpH - footholdY[4 + 1 * inturn][stepNum + 1]) * (1 - cos(s)) / 2;

//            pEE[2 + 3 * inturn] += 2 * stepUpD;
//            pEE[8 + 3 * inturn] += 2 * stepUpD;
//            pEE[14 + 3 * inturn] += 2 * stepUpD;

//            pBodyPE[2] += stepUpD;
//        }

//        if(count % periodCount == periodCount - 1)
//        {

//            if(inturn == 0)
//            {
//                inturn = 1;
//            }
//            else
//            {
//                inturn = 0;
//                stepNum++;
//            }
//        }
//        robot.SetPeb(pBodyPE, beginMak);
//        robot.SetPee(pEE, beginMak);
//    }
//    else
//    {
//        if(count % periodCount == 0)
//        {
//            beginMak.setPrtPm(*robot.body().pm());
//            beginMak.update();
//            robot.GetPee(beginPee, beginMak);
//        }

//        double pBodyPE[6] = {0, 0, 0, 0, 0, 0};
//        double pEE[18];
//        memcpy(pEE, beginPee, sizeof(beginPee));

//        int localCount = count % periodCount;

//        if(localCount < pperiodCount)
//        {
//            double s = -(PI / 2)*cos(PI * (localCount + 1) / pperiodCount) + PI / 2;

//            pEE[1 + 3 * inturn] += (stepUpH + footholdY[0 + 1 * inturn][stepNum]) * (1 - cos(s))/2;
//            pEE[7 + 3 * inturn] += (stepUpH + footholdY[2 + 1 * inturn][stepNum]) * (1 - cos(s))/2;
//            pEE[13 +  3 * inturn] += (stepUpH + footholdY[4 + 1 * inturn][stepNum]) * (1 - cos(s))/2;
//        }
//        else if(localCount >= pperiodCount && localCount < pperiodCount * 2)
//        {
//            double s = -(PI / 2)*cos(PI * (localCount + 1 - pperiodCount) / pperiodCount) + PI / 2;

//            pEE[1 + 3 * inturn] += (stepUpH + footholdY[0 + 1 * inturn][stepNum]);
//            pEE[7 + 3 * inturn] += (stepUpH + footholdY[2 + 1 * inturn][stepNum]);
//            pEE[13 +  3 * inturn] += (stepUpH + footholdY[4 + 1 * inturn][stepNum]);

//            pEE[2 + 3 * inturn] +=  stepUpD * (1 - cos(s))/2;
//            pEE[8 + 3 * inturn] +=  stepUpD * (1 - cos(s))/2;
//            pEE[14 + 3 * inturn] += stepUpD * (1 - cos(s))/2;

//            pBodyPE[2] += stepUpD / 2 * (1 - cos(s))/2;
//        }
//        else
//        {
//            double s = -(PI / 2)*cos(PI * (localCount + 1 - 2 * pperiodCount) / pperiodCount) + PI / 2;

//            pEE[1 + 3 * inturn] += (stepUpH + footholdY[0 + 1 * inturn][stepNum])
//                    - (stepUpH - footholdY[0 + 1 * inturn][stepNum + 1]) * (1 - cos(s)) / 2;
//            pEE[7 + 3 * inturn] += (stepUpH + footholdY[2 + 1 * inturn][stepNum])
//                    - (stepUpH - footholdY[2 + 1 * inturn][stepNum + 1]) * (1 - cos(s)) / 2;
//            pEE[13 +  3 * inturn] += (stepUpH + footholdY[4 + 1 * inturn][stepNum])
//                    - (stepUpH - footholdY[4 + 1 * inturn][stepNum + 1]) * (1 - cos(s)) / 2;

//            pEE[2 + 3 * inturn] += 2 * stepUpD;
//            pEE[8 + 3 * inturn] += 2 * stepUpD;
//            pEE[14 + 3 * inturn] += 2 * stepUpD;

//            pBodyPE[2] += stepUpD / 2;
//        }

//        if(count % periodCount == periodCount - 1)
//        {

//            if(inturn == 0)
//            {
//                inturn = 1;
//            }
//            else
//            {
//                inturn = 0;
//                stepNum++;
//            }
//        }
//        robot.SetPeb(pBodyPE, beginMak);
//        robot.SetPee(pEE, beginMak);
//    }
//    return totalCount - count - 1;
//}

//int RobotStepDown(Robots::RobotBase &robot, int count)
//{
//    int totalCount = 6000 * 3;
//    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
//    static double beginPee[18];

//    if (count % totalCount == 0)
//    {
//        beginMak.setPrtPm(*robot.body().pm());
//        beginMak.update();
//        robot.GetPee(beginPee, beginMak);
//    }

//    double pBodyPE[6] = {0, 0, 0, 0, 0, 0};

//    double pEE[18] =
//    { -0.3, -0.85, -0.65,
//      -0.45, -0.85, 0,
//      -0.3, -0.85, 0.65,
//      0.3, -0.85, -0.65,
//      0.45, -0.85, 0,
//      0.3, -0.85, 0.65 };

//    static double StepDownCurrentPos[6] = {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85};

//    double StepDownStartPos[6] = {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85};

//    for(int i = 0; i < 6; i++)
//    {
//        pEE[i*3 + 1] = StepDownCurrentPos[i];
//    }

//    double stepDownH = 0.05;
//    double stepDownD = 0.325;

//    double StepDownNextPos[6] = {0, 0, 0, 0, 0, 0};

//    memcpy(StepDownNextPos,pParam.stepdowndata,6*sizeof(double));

//    int periodcounter = totalCount / 6;

//    if(count < periodcounter)
//    {
//        double s = -(PI / 2)*cos(PI * (count + 1) / periodcounter) + PI / 2;

//        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85)) * (1 - cos(s))/2;
//        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85)) * (1 - cos(s))/2;
//        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85)) * (1 - cos(s))/2;
//    }
//    else if(count >= periodcounter && count < 2 * periodcounter)
//    {
//        double s = -(PI / 2)*cos(PI * (count + 1 - periodcounter) / periodcounter) + PI / 2;

//        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85));
//        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85));
//        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85));

//        pEE[2] += stepDownD * (1 -cos(s))/2;
//        pEE[8] += stepDownD * (1 -cos(s))/2;
//        pEE[14] += stepDownD * (1 -cos(s))/2;

//        pBodyPE[2] += stepDownD/2 * (1 - cos(s))/2;
//    }
//    else if(count >= 2 * periodcounter && count < 3 * periodcounter)
//    {
//        double s = -(PI / 2)*cos(PI * (count + 1 - 2*periodcounter) / periodcounter) + PI / 2;

//        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
//                - (stepDownH + (-StepDownNextPos[0] - 0.85)) * (1 - cos(s)) / 2;
//        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
//                - (stepDownH + (-StepDownNextPos[2] - 0.85)) * (1 - cos(s)) / 2;
//        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
//                - (stepDownH + (-StepDownNextPos[4] - 0.85)) * (1 - cos(s)) / 2;

//        pEE[2] += stepDownD;
//        pEE[8] += stepDownD;
//        pEE[14] += stepDownD;

//        pBodyPE[2] += stepDownD/2;
//    }
//    else if(count >= 3 * periodcounter && count < 4 * periodcounter)
//    {
//        double s = -(PI / 2)*cos(PI * (count + 1 - 3*periodcounter) / periodcounter) + PI / 2;

//        pEE[4] += (stepDownH + (-StepDownCurrentPos[1] - 0.85)) * (1 - cos(s))/2;
//        pEE[10] += (stepDownH + (-StepDownCurrentPos[3] - 0.85)) * (1 - cos(s))/2;
//        pEE[16] += (stepDownH + (-StepDownCurrentPos[5] - 0.85)) * (1 - cos(s))/2;

//        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
//                - (stepDownH + (-StepDownNextPos[0] - 0.85));
//        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
//                - (stepDownH + (-StepDownNextPos[2] - 0.85));
//        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
//                - (stepDownH + (-StepDownNextPos[4] - 0.85));

//        pEE[2] += stepDownD;
//        pEE[8] += stepDownD;
//        pEE[14] += stepDownD;

//        pBodyPE[2] += stepDownD/2;
//    }
//    else if(count >= 4 * periodcounter && count < 5 * periodcounter)
//    {
//        double s = -(PI / 2)*cos(PI * (count + 1 - 4*periodcounter) / periodcounter) + PI / 2;

//        pEE[4] += (stepDownH + (-StepDownCurrentPos[1] - 0.85));
//        pEE[10] += (stepDownH + (-StepDownCurrentPos[3] - 0.85));
//        pEE[16] += (stepDownH + (-StepDownCurrentPos[5] - 0.85));

//        pEE[5] += stepDownD * (1 -cos(s))/2;
//        pEE[11] += stepDownD * (1 -cos(s))/2;
//        pEE[17] += stepDownD * (1 -cos(s))/2;

//        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
//                - (stepDownH + (-StepDownNextPos[0] - 0.85));
//        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
//                - (stepDownH + (-StepDownNextPos[2] - 0.85));
//        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
//                - (stepDownH + (-StepDownNextPos[4] - 0.85));

//        pEE[2] += stepDownD;
//        pEE[8] += stepDownD;
//        pEE[14] += stepDownD;

//        pBodyPE[2] += stepDownD/2 + stepDownD/2 * (1 - cos(s))/2;
//    }
//    else
//    {
//        double s = -(PI / 2)*cos(PI * (count + 1 - 5*periodcounter) / periodcounter) + PI / 2;

//        pEE[4] += (stepDownH + (-StepDownCurrentPos[1] - 0.85))
//                - (stepDownH + (-StepDownNextPos[1] - 0.85)) * (1 - cos(s)) / 2;
//        pEE[10] += (stepDownH + (-StepDownCurrentPos[3] - 0.85))
//                - (stepDownH + (-StepDownNextPos[3] - 0.85)) * (1 - cos(s)) / 2;
//        pEE[16] += (stepDownH + (-StepDownCurrentPos[5] - 0.85))
//                - (stepDownH + (-StepDownNextPos[5] - 0.85)) * (1 - cos(s)) / 2;

//        pEE[5] += stepDownD;
//        pEE[11] += stepDownD;
//        pEE[17] += stepDownD;

//        pEE[1] += (stepDownH + (-StepDownCurrentPos[0] - 0.85))
//                - (stepDownH + (-StepDownNextPos[0] - 0.85));
//        pEE[7] += (stepDownH + (-StepDownCurrentPos[2] - 0.85))
//                - (stepDownH + (-StepDownNextPos[2] - 0.85));
//        pEE[13] += (stepDownH + (-StepDownCurrentPos[4] - 0.85))
//                - (stepDownH + (-StepDownNextPos[4] - 0.85));

//        pEE[2] += stepDownD;
//        pEE[8] += stepDownD;
//        pEE[14] += stepDownD;

//        pBodyPE[2] += stepDownD/2 + stepDownD/2;
//    }

//    if (totalCount - count - 1 == 0)
//    {
//        if(pParam.stepdowndata[6] == 4)
//        {
//            memcpy(StepDownCurrentPos, StepDownStartPos, 6*sizeof(double));
//        }
//        else
//        {
//            memcpy(StepDownCurrentPos, StepDownNextPos, 6*sizeof(double));
//        }
//    }

//    robot.SetPeb(pBodyPE, beginMak);
//    robot.SetPee(pEE, beginMak);
//}
