#include "Calibration.h"

namespace Calibration
{

double feetPosi[18] =
{ -0.3,  -0.935, -0.65,
  -0.45, -0.935,  0,
  -0.3,  -0.935,  0.65,
  0.3,  -0.935, -0.65,
  0.45, -0.935,  0,
  0.3,   -0.935,  0.65 };

enum CalibrationState
{
    None=0,
    Go=1,
    Processing=2,
    Back=3,
};

void TransM(double matrixIn[6], double matrixOut[6])
{
    double	alpha = matrixIn[0];
    double	beta = matrixIn[1];
    double	gama = matrixIn[2];

    Eigen::Matrix3f R_X;
    R_X << 1, 0, 0, 0, cos(gama), -sin(gama), 0, sin(gama), cos(gama);

    Eigen::Matrix3f R_Z;
    R_Z << cos(beta), -sin(beta), 0, sin(beta), cos(beta), 0, 0, 0, 1;

    Eigen::Matrix3f R_Y;
    R_Y << cos(alpha), 0, sin(alpha), 0, 1, 0, -sin(alpha), 0, cos(alpha);

    Eigen::Matrix3f Pose;
    Pose = R_Y * R_Z * R_X;

    double pMatrix[4][4] = { 0 };

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            pMatrix[i][j] = Pose(i, j);
        }
    }
    pMatrix[0][3] = matrixIn[3];
    pMatrix[1][3] = matrixIn[4];
    pMatrix[2][3] = matrixIn[5];
    pMatrix[3][3] = 1;

    aris::dynamic::s_pm2pe(*pMatrix, matrixOut, "313");
}

CalibrationWrapper calibrationWrapper;

aris::control::Pipe<int> CalibrationWrapper::calibrationPipe(true);
std::thread CalibrationWrapper::calibrationThread;

atomic_bool CalibrationWrapper::isTerrainCaliRecorded(false);
atomic_bool CalibrationWrapper::isSending(false);
atomic_bool CalibrationWrapper::isStop(false);
atomic_int CalibrationWrapper::calibrationState(CalibrationState::None);

void CalibrationWrapper::CalibrationStart()
{
    calibrationThread = std::thread([]()
    {
        while(true)
        {
            int postureCount;
            calibrationPipe.recvInNrt(postureCount);

            kinect2.SavePcd();
            cout<<" map recorded"<<endl;
            isTerrainCaliRecorded=true;
        }
    });
}

auto CalibrationWrapper::visionCalibrateParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    VISION_CALIBRATION_PARAM param;

    double a;

    std::ifstream file;
    std::string FileName = "/home/hex/Desktop/codes/chaixun/IXCalibration/Pose.txt";
    cout<<"file name:"<<FileName<<endl;

    file.open(FileName);
    if (!file) throw std::logic_error("calibration params file not exist");
    file>>a;

    int postureNum{ 0 };

    for (double tem; !file.eof(); file >> tem)  ++postureNum;
    if (postureNum % 6 != 0) throw std::logic_error("calibration params file invalid, because the num of numbers is not valid");
    postureNum /= 6;
    file.close();

    param.postureNum = postureNum;

    file.open(FileName);
    //file>>a;
    for (int i = 0; !file.eof(); file >> param.postureLib[i++]);
    file.close();

    cout<<"postureNum:"<<postureNum<<endl;

    msg_out.copyStruct(param);
}

auto CalibrationWrapper::visionCalibrate(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & cali_param)->int
{
    auto &robot=static_cast<Robots::RobotTypeIII &>(model);
    auto &pSP=static_cast< const VISION_CALIBRATION_PARAM &>(cali_param);
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static int localCount=0;
    static int postureCount=0;
    static double targetPosture[6];
    double currentPeb[6];
    double s;

    if(pSP.count==0)
    {
        rt_printf("calibration gait begins\n");
    }

    localCount = localCount%pSP.gaitLength;

    switch(calibrationState)
    {
    case None:
        calibrationState=Go;
        break;
    case Go:
        if(localCount==0)
        {
            rt_printf("calibration posture %d\n",postureCount);
            memcpy(targetPosture,&pSP.postureLib[6*postureCount],sizeof(targetPosture));
            rt_printf("%lf %lf %lf %lf %lf %lf \n", targetPosture[0], targetPosture[1], targetPosture[2], targetPosture[3], targetPosture[4], targetPosture[5]);
        }

        s=PI*(localCount+1)/pSP.gaitLength;// (0,pi]

        currentPeb[2]=0.6 * targetPosture[2]*M_PI/180.0*(1-cos(s))/2;
        currentPeb[1]=0.6 * targetPosture[1]*M_PI/180.0*(1-cos(s))/2;
        currentPeb[0]=0.6 * targetPosture[0]*M_PI/180.0*(1-cos(s))/2;
        currentPeb[3]=0.6 * targetPosture[3]*(1-cos(s))/2;
        currentPeb[4]=0.6 * targetPosture[4]*(1-cos(s))/2;
        currentPeb[5]=0.6 * targetPosture[5]*(1-cos(s))/2;

        double bodyPose[6];
        TransM(currentPeb, bodyPose);

        robot.SetPeb(bodyPose,beginMak);
        robot.SetWa(0);
        robot.SetPee(feetPosi, beginMak);
        localCount+=1;

        if(pSP.gaitLength-localCount==0)
        {
            calibrationState=Processing;
            calibrationPipe.sendToNrt(postureCount);
            localCount = 0;
            rt_printf("begin capture !\n");
            rt_printf("raw: %lf %lf %lf \n", currentPeb[0], currentPeb[1], currentPeb[2]);
            rt_printf("target: %lf %lf %lf \n", bodyPose[3], bodyPose[4], bodyPose[5]);
        }
        break;
    case Processing:
        if(isTerrainCaliRecorded==true)
        {
            calibrationState=Back;
            isTerrainCaliRecorded=false;
            rt_printf("end capture !\n");
        }
        break;
    case Back:
        if(localCount==0)
        {
            rt_printf("calibration posture finished going back %d\n",postureCount);
        }
        s=PI*(localCount+1)/pSP.gaitLength;// (0,pi]

        currentPeb[2]=0.6 * targetPosture[2]*M_PI/180.0*(1+cos(s))/2;
        currentPeb[1]=0.6 * targetPosture[1]*M_PI/180.0*(1+cos(s))/2;
        currentPeb[0]=0.6 * targetPosture[0]*M_PI/180.0*(1+cos(s))/2;
        currentPeb[3]=0.6 * targetPosture[3]*(1+cos(s))/2;
        currentPeb[4]=0.6 * targetPosture[4]*(1+cos(s))/2;
        currentPeb[5]=0.6 * targetPosture[5]*(1+cos(s))/2;

        double bodyPose1[6];
        TransM(currentPeb, bodyPose1);

        robot.SetPeb(bodyPose1,beginMak);
        robot.SetWa(0);
        robot.SetPee(feetPosi, beginMak);
        localCount+=1;

        if(pSP.gaitLength-localCount==0)
        {
            postureCount+=1;
            calibrationState=None;
            localCount = 0;
            if(postureCount==pSP.postureNum)
                return 0;
        }
        break;
    default:
        break;
    }

    return 1;
}

}
