#include "UpOneSep.h"

void FootRecTraj(double inPEE[3], double outPEE[3], double liftHeight, double fallHeight, double stepLength, int count, int totalCount)
{
    memcpy(outPEE, inPEE, 3 * sizeof(double));

    if(count < totalCount / 3)
    {
        //-1 ~ 1
        double s = - cos(M_PI * (count + 1) / (totalCount / 3));
        outPEE[1] = inPEE[1] + liftHeight * (1 + s) / 2;
    }
    else if(count >= totalCount / 3 && count < 2 * (totalCount / 3))
    {
        double s = - cos(M_PI * (count + 1 - totalCount / 3) / (totalCount / 3));
        outPEE[1] = inPEE[1] + liftHeight;
        outPEE[2] = inPEE[2] + stepLength * (1 + s) / 2;
    }
    else
    {
        double s = - cos(M_PI * (count + 1 - 2 * totalCount / 3) / (totalCount / 3));
        outPEE[1] = inPEE[1] + liftHeight - fallHeight * (1 + s) / 2;
        outPEE[2] = inPEE[2] + stepLength;
    }
}

void BodyTraj(double inBodyPEE[6], double outBodyPEE[6], double moveDis, double count, double totalCount)
{
    memcpy(outBodyPEE, inBodyPEE, 6 * sizeof(double));

    if(count < totalCount / 3)
    {
        double s = - cos(M_PI * (count + 1) / (totalCount / 3));
        // not move body
        ;
    }
    else if(count >= totalCount / 3 && count < 2 * (totalCount / 3))
    {
        double s = - cos(M_PI * (count + 1 - totalCount / 3) / (totalCount / 3));
        outBodyPEE[2] = inBodyPEE[2] + moveDis * (1 + s) / 2;
    }
    else
    {
        double s = - cos(M_PI * (count + 1 - 2 * totalCount / 3) / (totalCount / 3));
        outBodyPEE[2] = inBodyPEE[2] + moveDis;
    }
}

void ParseUpOnestep(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg)
{
    UpOneStepParam param;

    for(auto &i:params)
    {
        if(i.first == "sh")
        {
            param.stepHeight = stod(i.second);
        }
        else if(i.first == "bh")
        {
            param.bodyHeight = stod(i.second);
        }
        else if(i.first == "nh")
        {
            param.normalHeight = stod(i.second);
        }
        else
        {
            std::cout<<"Parse Failed! "<< std::endl;
        }
    }

    msg.copyStruct(param);

    std::cout<<"Finished Parse! "<< std::endl;
}

int UpOneStepGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotTypeIII &>(model);
    auto &param = static_cast<const UpOneStepParam &>(param_in);
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };

    static double beginBodyPEE[6];
    static double beginPEE[18];

    int upBodyCount = 2000;
    int moveBodyCount = 6000;

    double bodyPee[6] = {0};
    double feetPee[18]  = {0};

    double safeHeight = param.normalHeight;
    double nHeight = param.normalHeight;
    double sHeight = param.stepHeight + safeHeight;
    double bHeight = param.bodyHeight;
    double moveDis = 0.325;

    if(param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
    }

    // Step 0: Move Body Up 0.2
    if(param.count < upBodyCount)
    {
        if(param.count == 0)
        {
            // only one angle
            robot.GetPeb(beginBodyPEE, beginMak, "213");
            robot.GetPee(beginPEE, beginMak);
        }

        memcpy(feetPee, beginPEE, sizeof(beginPEE));
        memcpy(bodyPee, beginBodyPEE, sizeof(beginBodyPEE));

        // -1 ~ 1
        double s = - cos(M_PI * (param.count + 1) / upBodyCount);
        bodyPee[1] = beginBodyPEE[1] + bHeight * (1 + s) / 2 ;
    }
    // Step 1: leg 1 3 5; leg 3 up
    else if(param.count >= upBodyCount && param.count < upBodyCount + moveBodyCount)
    {
        int localCount = param.count - upBodyCount;

        if(param.count == upBodyCount)
        {
            beginMak.setPrtPm(*robot.body().pm());
            beginMak.update();
            robot.GetPeb(beginBodyPEE, beginMak, "213");
            robot.GetPee(beginPEE, beginMak);
            rt_printf("Body Z: %lf\n Foot 1: %lf. Foot 3: %lf.Foot 6: %lf.Foot 2: %lf.Foot 5: %lf.Foot 1: %lf.Foot 4: %lf.\n", beginBodyPEE[2], beginPEE[7], beginPEE[16], beginPEE[4], beginPEE[13], beginPEE[1], beginPEE[10]);
        }

        memcpy(feetPee, beginPEE, sizeof(beginPEE));
        memcpy(bodyPee, beginBodyPEE, sizeof(beginBodyPEE));

        FootRecTraj(&beginPEE[0], &feetPee[0], nHeight, nHeight, moveDis, localCount ,moveBodyCount);
        FootRecTraj(&beginPEE[6], &feetPee[6], sHeight, safeHeight, moveDis, localCount ,moveBodyCount);
        FootRecTraj(&beginPEE[12], &feetPee[12], nHeight, nHeight, moveDis, localCount ,moveBodyCount);

        BodyTraj(beginBodyPEE, bodyPee, moveDis / 2, localCount, moveBodyCount);
    }
    //Step 2: leg 2 4 6; leg 6 up
    else if(param.count >= upBodyCount + moveBodyCount && param.count < upBodyCount + 2 * moveBodyCount)
    {
        int localCount = param.count - upBodyCount - moveBodyCount;

        if(param.count == upBodyCount + moveBodyCount)
        {
            robot.GetPeb(beginBodyPEE, beginMak, "213");
            robot.GetPee(beginPEE, beginMak);
            rt_printf("Body Z: %lf\n Foot 1: %lf. Foot 3: %lf.Foot 6: %lf.Foot 2: %lf.Foot 5: %lf.Foot 1: %lf.Foot 4: %lf.\n", beginBodyPEE[2], beginPEE[7], beginPEE[16], beginPEE[4], beginPEE[13], beginPEE[1], beginPEE[10]);
        }

        memcpy(feetPee, beginPEE, sizeof(beginPEE));
        memcpy(bodyPee, beginBodyPEE, sizeof(beginBodyPEE));

        FootRecTraj(&beginPEE[3], &feetPee[3], nHeight, nHeight, moveDis * 2, localCount ,moveBodyCount);
        FootRecTraj(&beginPEE[9], &feetPee[9], nHeight, nHeight, moveDis * 2, localCount ,moveBodyCount);
        FootRecTraj(&beginPEE[15], &feetPee[15], sHeight, safeHeight, moveDis * 2, localCount ,moveBodyCount);

        BodyTraj(beginBodyPEE, bodyPee, moveDis, localCount, moveBodyCount);
    }
    //Step 3: leg 1 3 5; leg 3 5 up
    else if(param.count >= upBodyCount + 2 * moveBodyCount && param.count < upBodyCount + 3 * moveBodyCount)
    {
        int localCount = param.count - upBodyCount - 2 * moveBodyCount;

        if(param.count == upBodyCount + 2 * moveBodyCount)
        {
            robot.GetPeb(beginBodyPEE, beginMak, "213");
            robot.GetPee(beginPEE, beginMak);
            rt_printf("Body Z: %lf\n Foot 1: %lf. Foot 3: %lf.Foot 6: %lf.Foot 2: %lf.Foot 5: %lf.Foot 1: %lf.Foot 4: %lf.\n", beginBodyPEE[2], beginPEE[7], beginPEE[16], beginPEE[4], beginPEE[13], beginPEE[1], beginPEE[10]);
        }

        memcpy(feetPee, beginPEE, sizeof(beginPEE));
        memcpy(bodyPee, beginBodyPEE, sizeof(beginBodyPEE));

        FootRecTraj(&beginPEE[0], &feetPee[0], nHeight, nHeight, moveDis * 2, localCount ,moveBodyCount);
        FootRecTraj(&beginPEE[6], &feetPee[6], safeHeight, safeHeight, moveDis * 2, localCount ,moveBodyCount);
        FootRecTraj(&beginPEE[12], &feetPee[12], sHeight, safeHeight, moveDis * 2, localCount ,moveBodyCount);

        BodyTraj(beginBodyPEE, bodyPee, moveDis, localCount, moveBodyCount);
    }
    //Step 4: leg 2 4 6; leg 6 2 up
    else if(param.count >= upBodyCount + 3 * moveBodyCount && param.count < upBodyCount + 4 * moveBodyCount)
    {
        int localCount = param.count - upBodyCount - 3 * moveBodyCount;

        if(param.count == upBodyCount + 3 * moveBodyCount)
        {
            robot.GetPeb(beginBodyPEE, beginMak, "213");
            robot.GetPee(beginPEE, beginMak);
            rt_printf("Body Z: %lf\n Foot 1: %lf. Foot 3: %lf.Foot 6: %lf.Foot 2: %lf.Foot 5: %lf.Foot 1: %lf.Foot 4: %lf.\n", beginBodyPEE[2], beginPEE[7], beginPEE[16], beginPEE[4], beginPEE[13], beginPEE[1], beginPEE[10]);
        }

        memcpy(feetPee, beginPEE, sizeof(beginPEE));
        memcpy(bodyPee, beginBodyPEE, sizeof(beginBodyPEE));

        FootRecTraj(&beginPEE[3], &feetPee[3], sHeight, safeHeight, moveDis * 2, localCount ,moveBodyCount);
        FootRecTraj(&beginPEE[9], &feetPee[9], nHeight, nHeight, moveDis * 2, localCount ,moveBodyCount);
        FootRecTraj(&beginPEE[15], &feetPee[15], safeHeight, safeHeight, moveDis * 2, localCount ,moveBodyCount);

        BodyTraj(beginBodyPEE, bodyPee, moveDis, localCount, moveBodyCount);
    }
    //Step 5: leg 1 3 5; leg 3 5 1 up
    else if(param.count >= upBodyCount + 4 * moveBodyCount && param.count < upBodyCount + 5 * moveBodyCount)
    {
        int localCount = param.count - upBodyCount - 4 * moveBodyCount;

        if(param.count == upBodyCount + 4 * moveBodyCount)
        {
            robot.GetPeb(beginBodyPEE, beginMak, "213");
            robot.GetPee(beginPEE, beginMak);
            rt_printf("Body Z: %lf\n Foot 1: %lf. Foot 3: %lf.Foot 6: %lf.Foot 2: %lf.Foot 5: %lf.Foot 1: %lf.Foot 4: %lf.\n", beginBodyPEE[2], beginPEE[7], beginPEE[16], beginPEE[4], beginPEE[13], beginPEE[1], beginPEE[10]);
        }

        memcpy(feetPee, beginPEE, sizeof(beginPEE));
        memcpy(bodyPee, beginBodyPEE, sizeof(beginBodyPEE));

        FootRecTraj(&beginPEE[0], &feetPee[0], sHeight, safeHeight, moveDis * 2, localCount, moveBodyCount);
        FootRecTraj(&beginPEE[6], &feetPee[6], safeHeight, safeHeight, moveDis * 2, localCount, moveBodyCount);
        FootRecTraj(&beginPEE[12], &feetPee[12], safeHeight, safeHeight, moveDis * 2, localCount, moveBodyCount);

        BodyTraj(beginBodyPEE, bodyPee, moveDis, localCount, moveBodyCount);
    }
    //Step 6: leg 2 4 6; leg 6 2 4 up
    else if(param.count >= upBodyCount + 5 * moveBodyCount && param.count < upBodyCount + 6 * moveBodyCount)
    {
        int localCount = param.count - upBodyCount - 5 * moveBodyCount;

        if(param.count == upBodyCount + 5 * moveBodyCount)
        {
            robot.GetPeb(beginBodyPEE, beginMak, "213");
            robot.GetPee(beginPEE, beginMak);
            rt_printf("Body Z: %lf\n Foot 1: %lf. Foot 3: %lf.Foot 6: %lf.Foot 2: %lf.Foot 5: %lf.Foot 1: %lf.Foot 4: %lf.\n", beginBodyPEE[2], beginPEE[7], beginPEE[16], beginPEE[4], beginPEE[13], beginPEE[1], beginPEE[10]);
        }

        memcpy(feetPee, beginPEE, sizeof(beginPEE));
        memcpy(bodyPee, beginBodyPEE, sizeof(beginBodyPEE));

        FootRecTraj(&beginPEE[3], &feetPee[3], safeHeight, safeHeight, moveDis, localCount, moveBodyCount);
        FootRecTraj(&beginPEE[9], &feetPee[9], sHeight, safeHeight, moveDis, localCount, moveBodyCount);
        FootRecTraj(&beginPEE[15], &feetPee[15], safeHeight, safeHeight, moveDis, localCount, moveBodyCount);

        BodyTraj(beginBodyPEE, bodyPee, moveDis / 2, localCount, moveBodyCount);
    }
    robot.SetPeb(bodyPee, "213");
    robot.SetWa(0);
    robot.SetPee(feetPee);

    return param.count - upBodyCount - 6 * moveBodyCount - 1;
}
