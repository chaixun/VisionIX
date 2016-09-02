#ifndef CLIMBSTAIR_H
#define CLIMBSTAIR_H

#include <cstdint>
#include <map>

#include <aris.h>
#include <Basic_Gait.h>
#include <Robot_Type_III.h>

struct  Moveupstairs final:public aris::server::GaitParamBase
{
    std::int32_t totalCount;
    double d;
    double d2;
    //double d3;
    double h;
    double h1;
    double h2;
    double h3;
    double w;
};
auto parseMoveWithupstairs(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto moveupstairs(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param)->int;

struct  Movedownstairs final:public aris::server::GaitParamBase
{
    std::int32_t totalCount;
    double d;
    double d2;
    //double d3;
    double h;
    double h1;
    double h2;
    double h3;
    double w;
};
auto parseMoveWithdownstairs(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto movedownstairs(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param)->int;

#endif // CLIMBSTAIR_H
