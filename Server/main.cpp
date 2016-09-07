#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <map>
#include <string>

using namespace std;

#include <aris.h>
#include <Basic_Gait.h>
#include <Robot_Type_III.h>
#include "Vision_Gait.h"
#include "Kinect2.h"

#include "rtdk.h"
#include "unistd.h"
#include "VisionSensor.h"
#include "ClimbStair.h"
#include "PassStepDitch.h"
#include "Calibration.h"
#include "TreePass.h"
#include "UpOneSep.h"

using namespace aris::core;

int main(int argc, char *argv[])
{
    //kinect2.Start();

    //velodyne1.Start();

    PassStepDitch::adjustWrapper.AdjustStart();

    Calibration::calibrationWrapper.CalibrationStart();

    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
        xml_address = "/home/hex/Desktop/codes/chaixun/VisionIX/Robot_IX.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        xml_address = "/home/hex/Desktop/codes/chaixun/VisionIX/Robot_IX.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        xml_address = "/home/hex/Desktop/codes/chaixun/VisionIX/Robot_IX.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = aris::server::ControlServer::instance();

    rs.createModel<Robots::RobotTypeIII>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::Gait::basicParse, nullptr);
    rs.addCmd("ds", Robots::Gait::basicParse, nullptr);
    rs.addCmd("hm", Robots::Gait::basicParse, nullptr);
    rs.addCmd("hmsw", Robots::Gait::basicParse, nullptr);
    rs.addCmd("rc", Robots::Gait::recoverParse, Robots::Gait::recoverGait);
    rs.addCmd("wk", Robots::Gait::walkParse, Robots::Gait::walkGait);
    rs.addCmd("ro", Robots::Gait::resetOriginParse, Robots::Gait::resetOriginGait);
    rs.addCmd("ec", Robots::Gait::extendChainParse, Robots::Gait::extendChainGait);
    //waist cmd
    rs.addCmd("rcw", Robots::Gait::recoverWaistParse, Robots::Gait::recoverWaistGait);
    rs.addCmd("aw", Robots::Gait::adjustWaistParse, Robots::Gait::adjustWaistGait);

    rs.addCmd("sdwk", PassStepDitch::adjustWrapper.PassStepDitchParse, PassStepDitch::adjustWrapper.PassStepDitchGait);
    rs.addCmd("ssdwk", PassStepDitch::adjustWrapper.StopPassStepDitchParse, PassStepDitch::adjustWrapper.PassStepDitchGait);

    rs.addCmd("ca", Calibration::calibrationWrapper.visionCalibrateParse, Calibration::calibrationWrapper.visionCalibrate);
    rs.addCmd("cap", Calibration::calibrationWrapper.captureParse, nullptr);

    rs.addCmd("up", parseMoveWithupstairs, moveupstairs);
    rs.addCmd("dw", parseMoveWithdownstairs, movedownstairs);

    rs.addCmd("mr",parseMoveWithRotate,moveWithRotate);

    rs.addCmd("twk", TreePass::treePassWrapper.TreePassParse, TreePass::treePassWrapper.TreePaseWalk);
    rs.addCmd("swk", TreePass::treePassWrapper.StopTreePassParse, TreePass::treePassWrapper.TreePaseWalk);

//    rs.addCmd("up25", ParseUp25Step, Up25StepGait);
        rs.addCmd("up25", ParseUp25Step, Up25StepTwoTwoGait);
    rs.addCmd("up15", ParseUp15Step, Up15StepGait);
//    rs.addCmd("dw25", ParseDown25Step, Down25StepGait);
//    rs.addCmd("dw15", ParseDown15Step, Down15StepGait);

    rs.open();

    rs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile(xml_address.c_str());
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)
            throw std::runtime_error("can't find Model element in xml file");
        rs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });

    aris::core::runMsgLoop();

    return 0;
}
