#ifndef UPONESEP_H
#define UPONESEP_H

#include <aris.h>
#include <Basic_Gait.h>
#include <Robot_Type_III.h>
#include <rtdk.h>

struct UpOneStepParam final :public aris::server::GaitParamBase
{
    double stepHeight;
    double bodyHeight;
    double normalHeight;
    std::int32_t totalCount;
};
void ParseUp25Step(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
int Up25StepGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

int Up25StepTwoTwoGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

void ParseUp15Step(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
int Up15StepGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

void ParseDown25Step(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
int Down25StepGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

//void ParseDown15Step(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
//int Down15StepGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

#endif // UPONESEP_H
