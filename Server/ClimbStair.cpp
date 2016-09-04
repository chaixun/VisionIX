#include "ClimbStair.h"
#include "rtdk.h"

#ifdef UNIX
#include "rtdk.h"
#endif
#ifdef WIN32
#define rt_printf printf
#endif

#include <cstring>
#include <cmath>
#include <algorithm>
#include <memory>

void parseMoveWithupstairs(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    Moveupstairs param;// You set param in the shape of Moveupstairs

    for(auto &i:params)
    {
        if (i.first == "distance")
        {
            param.d = stod(i.second);
        }
        else if (i.first == "distance2")
        {
            param.d2 = stod(i.second);
        }
        else if (i.first == "height")
        {
            param.h = stod(i.second);
        }
        else if(i.first=="height1")
        {
            param.h1=stod(i.second);
        }
        else if(i.first=="height2")
        {
            param.h2=stod(i.second);
        }
        else if(i.first=="height3")
        {
            param.h3=stod(i.second);
        }
        else if(i.first=="alpha")
        {
            param.w=stod(i.second);
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
    std::cout<<param.totalCount<<std::endl;

    std::cout<<"finished parse"<<std::endl;
}

void parseMoveWithdownstairs(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    Movedownstairs param;// You set param in the shape of Moveupstairs

    for(auto &i:params)
    {
        if (i.first == "distance")
        {
            param.d = stod(i.second);
        }
        else if (i.first == "distance2")
        {
            param.d2 = stod(i.second);
        }
        else if (i.first == "height")
        {
            param.h = stod(i.second);
        }
        else if(i.first=="height1")
        {
            param.h1=stod(i.second);
        }
        else if(i.first=="height2")
        {
            param.h2=stod(i.second);
        }
        else if(i.first=="height3")
        {
            param.h3=stod(i.second);
        }
        else if(i.first=="alpha")
        {
            param.w=stod(i.second);
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
    std::cout<<param.totalCount<<std::endl;

    std::cout<<"finished parse"<<std::endl;
}

int moveupstairs(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotTypeIII &>(model);
    auto &param = static_cast<const Moveupstairs &>(param_in);// change PlanParamBase to MoveupstairsParam

    static aris::dynamic::FloatMarker beginMaker{ robot.ground() };

    static double beginBodyPE213[6];
    static double pEE[18];
   //static double beginWa;
    static double l = 0.65;
    static double d3=l-param.d-0.5*param.d2+0.05;
    static int tl=param.d2*param.totalCount/(param.d);
    static int tr=d3*param.totalCount/(param.d);
    static double realBody[6];
    static double realpEE[18];
    static double NewPEE[18];
   //static double Wa;

    static double k;

    if(param.h==0.25)
    {
        k=1;
    }
    else
    {
        k=0 ;
    }

    if(param.count==0)
    {
        beginMaker.setPrtPm(*robot.body().pm());
        beginMaker.update();

        robot.GetPeb(beginBodyPE213,robot.ground(),"213");
        //robot.GetWa(beginWa);
        robot.GetPee(pEE);
        memcpy(realpEE,pEE,sizeof(realpEE));
        memcpy(realBody,beginBodyPE213,sizeof(beginBodyPE213));
    }
    //    rt_printf("pEE\n");
    //    for(int i=0;i<18;i++)
    //    {
    //        rt_printf("%f\n",pEE[i]);
    //    }

    //    std::copy(realBody,realBody + 6, beginBodyPE213);
    //    std::copy(realpEE, realpEE + 18, pEE);

    if(param.count>0 && param.count<param.totalCount)
    {
        double s = -(param.h3 / 2)*cos(PI * (param.count + 1) / (param.totalCount) ) + param.h3/ 2;
           //     double c=-(param.w*PI / (180*2))*cos(PI * (param.count + 1) / (param.totalCount) ) + param.w*PI/(180* 2);
        realBody[1]=beginBodyPE213[1]+s;
        //        realBody[4]=beginBodyPE213[4]+c;

    }

    else if(param.count>=param.totalCount && param.count<2*param.totalCount)
    {
        // rt_printf("2\n");
        double c=-(param.w*PI / (180*2))*cos(PI * (param.count-param.totalCount + 1) / (param.totalCount) ) + param.w*PI/(180* 2);
        realBody[4]=beginBodyPE213[4]+c;
        //Wa=beginWa+c;
        realBody[1]=beginBodyPE213[1]+param.h3;
        realBody[2]=beginBodyPE213[2]-(-(param.d-param.d*cos(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(2*param.totalCount))))/2);
        for(int i=0;i<18;i++)
        {
            if(i==1)
            {
                realpEE[i]=pEE[i]+(param.h+param.h1)*sin(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(2*param.totalCount)));
            }
            else if (i == 7)
            {
                realpEE[i] = pEE[i] + (param.h1)*sin(PI / 2 - PI / 2 * cos(PI*(param.count - param.totalCount + 1) / (param.totalCount + param.d2*param.totalCount / (param.d)))) ;
            }
            else if(i== 13)
            {
                realpEE[i]=pEE[i]+(param.h1)*sin(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(param.totalCount+param.d2*param.totalCount/(param.d))));
            }
            else if(i== 2 )
            {
                realpEE[i]=pEE[i]-(-param.d+param.d*cos(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(2*param.totalCount))));
            }
            else if(i==  8 )
            {
                realpEE[i]=pEE[i]-(-(param.d+param.d2)/2+(param.d+param.d2)/2*cos(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(param.totalCount+param.d2*param.totalCount/(param.d)))));
            }
            else if(i==  14)
            {
                realpEE[i]=pEE[i]-(-(param.d+param.d2)/2+(param.d+param.d2)/2*cos(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(param.totalCount+param.d2*param.totalCount/(param.d)))));
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
        /* rt_printf("realpEE\n");
       for(int i=0;i<18;i++)
      {
          rt_printf("%f\n",realpEE[i]);

       }*/
        /*rt_printf("realbody\n");
       for(int i=0;i<6;i++)
      {
          rt_printf("%f\n",realBody[i]);

       }*/

    }

    else if(param.count>=2*param.totalCount && param.count<2*param.totalCount+tl)
    {   //double s = -(H / 2)*cos(PI * (param.count -2*param.totalCount+ 1) / (param.totalCount) ) + H/ 2;
        double c=-(param.w*PI / (180*2*11))*cos(PI * (param.count-2*param.totalCount + 1) / (tl) ) + param.w*PI/(180* 2*11);
        realBody[4]=beginBodyPE213[4]+param.w*PI/180-c;
        // Wa=beginWa+param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]+param.h3;
        realBody[2]=beginBodyPE213[2]-(-(param.d-param.d2*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-param.totalCount+1)/(2*param.totalCount*param.d2)+PI/2-PI*(param.d)/(2*param.d2))))/2);

        for(int i=0;i<18;i++)
        {

            if(i==1 )
            {
                realpEE[i]=pEE[i]+param.h+param.h1*sin(PI/2-PI/2*cos(PI*(param.d)*(param.count-param.totalCount+1)/(2*param.totalCount*param.d2)+PI/2-PI*(param.d)/(2*param.d2)));

            }
            else if (i == 7)
            {
                realpEE[i] = pEE[i] + (param.h1)*sin(PI / 2 - PI / 2 * cos(PI*(param.count - param.totalCount + 1) / (param.totalCount + param.d2*param.totalCount / (param.d)))) ;
            }
            else if(i==13 )
            {
                realpEE[i]=pEE[i]+(param.h1)*sin(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(param.totalCount+param.d2*param.totalCount /(param.d))));
            }
            else if(i== 2 )
            {
                realpEE[i]=pEE[i]-(-param.d+param.d2*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-param.totalCount+1)/(2*param.totalCount*param.d2)+PI/2-PI*(param.d)/(2*param.d2))));

            }
            else if(i==  8 )
            {
                realpEE[i]=pEE[i]-(-(param.d+param.d2)/2+(param.d+param.d2)/2*cos(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(param.totalCount+param.d2*param.totalCount /(param.d)))));
            }
            else if(i==  14)
            {
                realpEE[i]=pEE[i]-(-(param.d+param.d2)/2+(param.d+param.d2)/2*cos(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(param.totalCount+param.d2*param.totalCount /(param.d)))));
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
        /*rt_printf("realpEE\n");
              for(int i=0;i<18;i++)
             {
                 rt_printf("%f\n",realpEE[i]);

              }*/
        /* rt_printf("realbody\n");
              for(int i=0;i<6;i++)
             {
                 rt_printf("%f\n",realBody[i]);
              }*/
    }

    else if( param.count>=2*param.totalCount+tl && param.count<3*param.totalCount+tl)
    {
        double c=-(param.w*PI / (180*2*11))*cos(PI * (param.count-(2*param.totalCount+tl) + 1) / (param.totalCount) ) + param.w*PI/(180* 2*11);
        realBody[4]=beginBodyPE213[4]+10.0/11.0*param.w*PI/180-c;
         //Wa=beginWa+10.0/11.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]+param.h3;
        realBody[2]=beginBodyPE213[2]-(-((param.d+param.d2)/2+param.d-(param.d)*cos(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(2*param.totalCount))))/2 - (param.d + param.d2) / 4);
        for(int i=0;i<18;i++)
        {
            if(i==10)
            {
                realpEE[i]=pEE[i]+(param.h+param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(2*param.totalCount)));
            }
            else if(i== 4 || i== 16)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i== 11 )
            {
                realpEE[i]=pEE[i]-(-param.d+param.d*cos(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(2*param.totalCount))));
            }
            else if(i==  5 || i==17)
            {
                realpEE[i]=pEE[i]-(-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==1 )
            {
                realpEE[i]=pEE[i]+param.h;
            }
            else if(i==2 || i== 8 ||i== 14)
            {
                realpEE[i]=pEE[i]-(-param.d-param.d2);
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
        /* rt_printf("realpEE\n");
            for(int i=0;i<18;i++)
           {
               rt_printf("%f\n",realpEE[i]);

            }
      rt_printf("realbody\n");
            for(int i=0;i<3;i++)
           {
               rt_printf("%f\n",realBody[i]);
            }*/

    }

    else if(param.count>=3*param.totalCount+tl && param.count<3*param.totalCount+tl+tr)
    {   double c=-(param.w*PI / (180*2*11))*cos(PI * (param.count-(3*param.totalCount+tl) + 1) / (tr) ) + param.w*PI/(180* 2*11);
        realBody[4]=beginBodyPE213[4]+9.0/11.0*param.w*PI/180-c;
       // Wa=beginWa+9.0/11.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]+param.h3;
        realBody[2]=beginBodyPE213[2]-(-((param.d+param.d2)/2+param.d-d3*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(2*param.totalCount+tl)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))))/2 - (param.d + param.d2) / 4);
        for(int i=0;i<18;i++)
        {
            if(i==10 )
            {
                realpEE[i]=pEE[i]+param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.d)*(param.count-(2*param.totalCount+tl)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3)));

            }
            else if(i== 4 || i==16)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i== 11 )
            {
                realpEE[i]=pEE[i]-(-param.d+d3*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(2*param.totalCount+tl)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))));
            }
            else if(i==  5 || i==17)
            {
                realpEE[i]=pEE[i]-(-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==1)
            {
                realpEE[i]=pEE[i]+param.h;
            }
            else if(i==2 ||i== 8 || i==14)
            {
                realpEE[i]=pEE[i]-(-param.d-param.d2);
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
        /*rt_printf("realpEE\n");
                for(int i=0;i<18;i++)
               {
                   rt_printf("%f\n",realpEE[i]);

                }
         /* rt_printf("realbody\n");
                for(int i=0;i<3;i++)
               {
                   rt_printf("%f\n",realBody[i]);
                }*/

    }

    else if(param.count>=3*param.totalCount+tl+tr && param.count<4*param.totalCount+tl+tr)
    {
        double c=-(param.w*PI / (180*2*11))*cos(PI * (param.count-(3*param.totalCount+tl+tr) + 1) / (param.totalCount) ) + param.w*PI/(180* 2*11);
        realBody[4]=beginBodyPE213[4]+8.0/11.0*param.w*PI/180-c;
        //Wa=beginWa+8.0/11.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]+param.h3;

        double s1=-(d3*0.05 / (2))*cos(PI * (param.count-(3*param.totalCount+tl+tr) + 1) / (param.totalCount) ) + d3*0.05/( 2);
        realBody[2]=beginBodyPE213[2]-(-((2*param.d+param.d2+d3)/2+param.d-param.d*cos(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(2*param.totalCount))))/2 - (2 * param.d + param.d2 + d3) / 4)-s1*k;

        double s3=-(d3*0.25 / (2))*cos(PI * (param.count-(3*param.totalCount+tl+tr) + 1) / (param.totalCount) ) + d3*0.25/( 2);
        for(int i=0;i<18;i++)
        {   if(i==0)
            {
                 realpEE[i]=pEE[i]+s3*k;
            }
            else if(i==12)
            {
                 realpEE[i]=pEE[i]+s3*k*2;
            }
             else if(i==1)
            {
                realpEE[i]=pEE[i]+param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==7)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==13)
            {
                realpEE[i]=pEE[i]+(param.h+param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(2*param.totalCount)));
            }
            else if(i==2 ||i== 8)
            {
                realpEE[i]=pEE[i]-(-param.d-param.d2-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==14)
            {
                realpEE[i]=pEE[i]-(-param.d-param.d2-param.d+param.d*cos(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(2*param.totalCount))));
            }
            else if(i==10)
            {
                realpEE[i]=pEE[i]+param.h;
            }
            else if(i==5 || i==11 || i==17)
            {
                realpEE[i]=pEE[i]-(-param.d-d3);
            }
            else
            {
                realpEE[i]=pEE[i];
            }

        }
        /*rt_printf("realpEE\n");
              for(int i=0;i<18;i++)
             {
                 rt_printf("%f\n",realpEE[i]);

              }*/
       /* rt_printf("realbody\n");
              for(int i=0;i<3;i++)
             {
                 rt_printf("%f\n",realBody[i]);
              }*/

    }

    else if(param.count>=4*param.totalCount+tl+tr && param.count<4*param.totalCount+tl+2*tr)
    {   double c=-(param.w*PI / (180*2*11))*cos(PI * (param.count-(4*param.totalCount+tl+tr) + 1) / (tr) ) + param.w*PI/(180* 2*11);
        realBody[4]=beginBodyPE213[4]+7.0/11.0*param.w*PI/180-c;
       // Wa=beginWa+7.0/11.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]+param.h3;

        //double s1=-(d3*0.05 / (2))*cos(PI * (param.count-(4*param.totalCount+tl+tr) + 1) / (tr) ) + d3*0.05/( 2);
        realBody[2]=beginBodyPE213[2]-(-((2*param.d+param.d2+d3)/2+param.d-d3*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(3*param.totalCount+tl+tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))))/2 - (2 * param.d + param.d2 + d3) / 4)-d3*0.05*k;
        double s3=-(d3*0.25 / (2))*cos(PI * (param.count-(4*param.totalCount+tl+tr) + 1) / (tr) ) + d3*0.25/( 2);

        for(int i=0;i<18;i++)
        {
            if(i==0)
            {
                 realpEE[i]=pEE[i]-s3*k+d3*0.25*k;
            }
            else if(i==12)
            {
                 realpEE[i]=pEE[i]-s3*k*2+d3*0.25*k*2;
            }

             else if(i==1)
            {
                realpEE[i]=pEE[i]+param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==7)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==13)
            {
                realpEE[i]=pEE[i]+param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.d)*(param.count-(3*param.totalCount+tl+tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3)));
            }
            else if(i==2 || i==8)
            {
                realpEE[i]=pEE[i]-(-param.d-param.d2-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==14)
            {
                realpEE[i]=pEE[i]-(-param.d-param.d2-param.d+d3*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(3*param.totalCount+tl+tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))));
            }
            else if(i==10)
            {
                realpEE[i]=pEE[i]+param.h;
            }
            else if(i==5 || i==11 || i== 17)
            {
                realpEE[i]=pEE[i]-(-param.d-d3);
            }
            else
            {
                realpEE[i]=pEE[i];
            }

        }

       /* rt_printf("realpEE\n");
              for(int i=0;i<18;i++)
             {
                 rt_printf("%f\n",realpEE[i]);

              }*/
        /*rt_printf("realbody\n");
                      for(int i=0;i<3;i++)
                     {
                         rt_printf("%f\n",realBody[i]);
                      }*/
    }

    else if(param.count>=4*param.totalCount+tl+2*tr && param.count<5*param.totalCount+tl+2*tr)
    {   double c=-(param.w*PI / (180*2*11))*cos(PI * (param.count-(4*param.totalCount+tl+2*tr) + 1) / (param.totalCount) ) + param.w*PI/(180* 2*11);
        realBody[4]=beginBodyPE213[4]+6.0/11.0*param.w*PI/180-c;
       // Wa=beginWa+6.0/11.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]+param.h3;

        double s1=-(d3*0.18 / (2))*cos(PI * (param.count-(4*param.totalCount+tl+2*tr) + 1) / (param.totalCount) ) + d3*0.18/( 2);
        realBody[2]=beginBodyPE213[2]-(-((3*param.d+param.d2+2*d3)/2+param.d-param.d*cos(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(2*param.totalCount))))/2- (3 * param.d + param.d2 + 2 * d3)/4)-d3*0.05*k-s1*k;
        double s3=-(d3*0.3 / (2))*cos(PI * (param.count-(4*param.totalCount+tl+2*tr) + 1) / (param.totalCount) ) + d3*0.3/( 2);

        for(int i=0;i<18;i++)
        {
           if(i==3)
           {
               realpEE[i]=pEE[i]-s3*k;
           }

            else if(i== 9 )
           {
               realpEE[i]=pEE[i]-s3*k;
           }
            else if(i== 10 )
            {
                realpEE[i]=pEE[i]+param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==16)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==4)
            {
                realpEE[i]=pEE[i]+(param.h+param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(2*param.totalCount)));
            }
            else if(i==11 ||i== 17)
            {
                realpEE[i]=pEE[i]-(-param.d-d3-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==5)
            {
                realpEE[i]=pEE[i]-(-param.d-d3-param.d+param.d*cos(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(2*param.totalCount))));
            }
            else if(i==2 ||i== 8 || i==14 )
            {
                realpEE[i]=pEE[i]-(-2*param.d-param.d2-d3);
            }
            else if(i==1 || i==13)
            {
                realpEE[i]=pEE[i]+param.h;
            }

            else
            {
                realpEE[i]=pEE[i];
            }
        }
    }

    else if(param.count>=5*param.totalCount+tl+2*tr && param.count<5*param.totalCount+tl+3*tr)
    {   double c=-(param.w*PI / (180*2*11))*cos(PI * (param.count-(5*param.totalCount+tl+2*tr) + 1) / (tr) ) + param.w*PI/(180* 2*11);
        realBody[4]=beginBodyPE213[4]+5.0/11.0*param.w*PI/180-c;
        //Wa=beginWa+5.0/11.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]+param.h3;

        double s1=-(d3*0.18 / (2))*cos(PI * (param.count-(5*param.totalCount+tl+2*tr) + 1) / (tr) ) + d3*0.18/(2);

        realBody[2]=beginBodyPE213[2]-(-((3*param.d+param.d2+2*d3)/2+param.d-d3*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(4*param.totalCount+tl+2*tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))))/2- (3 * param.d + param.d2 + 2 * d3)/4)-s1*k-0.23*d3*k;

        double s3=-(d3*0.3 / (2))*cos(PI * (param.count-(5*param.totalCount+tl+2*tr) + 1) / (tr) ) + d3*0.3/(2);
        for(int i=0;i<18;i++)
        {
            if(i==3)
            {
                realpEE[i]=pEE[i]+s3*k-d3*0.3*k;
            }
            else if(i== 9 )
            {
                realpEE[i]=pEE[i]+s3*k-d3*0.3*k;
            }
            else if(i== 10 )
            {
                realpEE[i]=pEE[i]+param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==16)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==4)
            {
                realpEE[i]=pEE[i]+param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.d)*(param.count-(4*param.totalCount+tl+2*tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3)));
            }
            else if(i==11 || i==17)
            {
                realpEE[i]=pEE[i]-(-param.d-d3-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==5)
            {
                realpEE[i]=pEE[i]-(-param.d-d3-param.d+d3*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(4*param.totalCount+tl+2*tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))));
            }
            else if(i==2 || i==8 ||i== 14 )
            {
                realpEE[i]=pEE[i]-(-2*param.d-param.d2-d3);
            }
            else if(i==1 || i==13)
            {
                realpEE[i]=pEE[i]+param.h;
            }

            else
            {
                realpEE[i]=pEE[i];
            }

        }
    }

    else if(param.count>=5*param.totalCount+tl+3*tr && param.count<6*param.totalCount+tl+3*tr)
    {   double c=-(param.w*PI / (180*2*11))*cos(PI * (param.count-(5*param.totalCount+tl+3*tr) + 1) / (param.totalCount) ) + param.w*PI/(180* 2*11);
        realBody[4]=beginBodyPE213[4]+4.0/11.0*param.w*PI/180-c;
       //Wa=beginWa+4.0/11.0*param.w*PI/180-c;

        double s2=-(d3*0.08 / (2))*cos(PI * (param.count-(5*param.totalCount+tl+3*tr) + 1) / (param.totalCount) ) + d3*0.08/( 2);
        realBody[1]=beginBodyPE213[1]+param.h3+s2*k;


        double s1=-(d3*0.3 / (2))*cos(PI * (param.count-(5*param.totalCount+tl+3*tr) + 1) / (param.totalCount) ) + d3*0.3/( 2);
        realBody[2]=beginBodyPE213[2]-(-((4*param.d+param.d2+3*d3)/2+param.d-param.d*cos(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(2*param.totalCount))))/2- (4 * param.d + param.d2 + 3 * d3)/4)+s1*k-0.41*d3*k;

        double s3=-(d3*0.1 / (2))*cos(PI * (param.count-(5*param.totalCount+tl+3*tr) + 1) / (param.totalCount) ) + d3*0.1/( 2);
        for(int i=0;i<18;i++)
        {
            if(i== 6 )
            {
                realpEE[i]=pEE[i]+s3*k;
            }


            else if(i== 12 )
           {
               realpEE[i]=pEE[i]+s3*k;
           }
             else if(i==1 || i==13)
            {
                realpEE[i]=pEE[i]+param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==7)
            {
                realpEE[i]=pEE[i]+(param.h+param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(2*param.totalCount)));
            }
            else if(i==2 || i==14)
            {
                realpEE[i]=pEE[i]-(-2*param.d-param.d2-d3-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==8)
            {
                realpEE[i]=pEE[i]-(-2*param.d-param.d2-d3-param.d+param.d*cos(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(2*param.totalCount))));
            }
            else if(i==4 || i==10)
            {
                realpEE[i]=pEE[i]+param.h;
            }
            else if(i==5 ||i== 11 || i==17)
            {
                realpEE[i]=pEE[i]-(-2*param.d-2*d3);
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
    }

    else if(param.count>=6*param.totalCount+tl+3*tr && param.count<6*param.totalCount+tl+4*tr)
    {   double c=-(param.w*PI / (180*2*11))*cos(PI * (param.count-(6*param.totalCount+tl+3*tr) + 1) / (tr) ) + param.w*PI/(180* 2*11);
        realBody[4]=beginBodyPE213[4]+3.0/11.0*param.w*PI/180-c;
        //Wa=beginWa+3.0/11.0*param.w*PI/180-c;

        double s2=-(d3 *0.08/ (2))*cos(PI * (param.count-(6*param.totalCount+tl+3*tr) + 1) / (tr) ) + d3*0.08/(2);
        realBody[1]=beginBodyPE213[1]+param.h3-s2*k+d3*0.08*k;

        double s1=-(d3 *0/ (2))*cos(PI * (param.count-(6*param.totalCount+tl+3*tr) + 1) / (tr) ) + d3*0/(2);
        realBody[2]=beginBodyPE213[2]-(-((4*param.d+param.d2+3*d3)/2+param.d-d3*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(5*param.totalCount+tl+3*tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))))/2- (4 * param.d + param.d2 + 3 * d3)/4)+s1*k-0.11*d3*k;

        double s3=-(d3 *0.1/ (2))*cos(PI * (param.count-(6*param.totalCount+tl+3*tr) + 1) / (tr) ) + d3*0.1/(2);

        for(int i=0;i<18;i++)
        {
            if(i== 6 )
            {
                realpEE[i]=pEE[i]-s3*k+d3*0.1*k;
            }
            else if(i== 12 )
           {
               realpEE[i]=pEE[i]-s3*k+d3*0.1*k;
           }
            else if(i==1 || i==13)
            {
                realpEE[i]=pEE[i]+param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==7)
            {
                realpEE[i]=pEE[i]+param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.d)*(param.count-(5*param.totalCount+tl+3*tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3)));
            }
            else if(i==2 || i==14)
            {
                realpEE[i]=pEE[i]-(-2*param.d-param.d2-d3-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==8)
            {
                realpEE[i]=pEE[i]-(-2*param.d-param.d2-d3-param.d+d3*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(5*param.totalCount+tl+3*tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))));
            }
            else if(i==4 || i==10)
            {
                realpEE[i]=pEE[i]+param.h;
            }
            else if(i==5 || i==11 || i==17)
            {
                realpEE[i]=pEE[i]-(-2*param.d-2*d3);
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
    }

    else if(param.count>=6*param.totalCount+tl+4*tr && param.count<7*param.totalCount+tl+4*tr)
    {   double c=-(param.w*PI / (180*2*11))*cos(PI * (param.count-(6*param.totalCount+tl+4*tr) + 1) / (param.totalCount) ) + param.w*PI/(180* 2*11);
        realBody[4]=beginBodyPE213[4]+2.0/11.0*param.w*PI/180-c;
        //Wa=beginWa+2.0/11.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]+param.h3;

        double s1=-(d3 *0.11/ (2))*cos(PI * (param.count-(6*param.totalCount+tl+4*tr) + 1) / (param.totalCount) ) + d3*0.11/(2);
        realBody[2]=beginBodyPE213[2]-(-((5*param.d+param.d2+4*d3)/2+param.d-param.d*cos(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(2*param.totalCount))))/2- (5 * param.d + param.d2 + 4 * d3)/4)+s1*k-0.11*d3*k;

        double s3=-(d3 *0.45/ (2))*cos(PI * (param.count-(6*param.totalCount+tl+4*tr) + 1) / (param.totalCount) ) + d3*0.45/(2);
        for(int i=0;i<18;i++)
        {
            if(i==3)
            {
                realpEE[i]=pEE[i]-s3*k;
            }
            else if(i==9)
            {
                realpEE[i]=pEE[i]-s3*k;
            }
            else if(i==15)
            {
                realpEE[i]=pEE[i]-s3*k;
            }

            else if(i==4 || i==10)
            {
                realpEE[i]=pEE[i]+param.h+param.h1*sin(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(param.totalCount+param.d2*param.totalCount/(param.d))));
            }
            else if(i==16)
            {
                realpEE[i]=pEE[i]+(param.h+param.h1)*sin(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(2*param.totalCount)));
            }
            else if(i==5 || i==11)
            {
                realpEE[i]=pEE[i]-(-2*param.d-2*d3-(param.d+param.d2)/2+(param.d+param.d2)/2*cos(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(param.totalCount+param.d2*param.totalCount/(param.d)))));
            }
            else if(i==17)
            {
                realpEE[i]=pEE[i]-(-2*param.d-2*d3-param.d+param.d*cos(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(2*param.totalCount))));
            }
            else if(i==1 || i==7 || i==13)
            {
                realpEE[i]=pEE[i]+param.h;
            }
            else if(i==2 || i==8 || i==14)
            {
                realpEE[i]=pEE[i]-(-3*param.d-param.d2-2*d3);
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
       /* rt_printf("realbody\n");
                              for(int i=0;i<3;i++)
                             {
                                 rt_printf("%f\n",realBody[i]);
                              }*/
    }

    else if(param.count>=7*param.totalCount+tl+4*tr && param.count<7*param.totalCount+2*tl+4*tr)
    {   double c=-(param.w*PI / (180*2*11))*cos(PI * (param.count-(7*param.totalCount+tl+4*tr) + 1) / (tl) ) + param.w*PI/(180* 2*11);
       realBody[4]=beginBodyPE213[4]+1.0/11.0*param.w*PI/180-c;
        //Wa=beginWa+1.0/11.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]+param.h3;
        realBody[2]=beginBodyPE213[2]-(-((5*param.d+param.d2+4*d3)/2+param.d-param.d2*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(6*param.totalCount+tl+4*tr)+1)/(2*param.totalCount*param.d2)+PI/2-PI*(param.d)/(2*param.d2))))/2- (5 * param.d + param.d2 + 4 * d3)/4);

        double s3=-(d3 *0.45/ (2))*cos(PI * (param.count-(7*param.totalCount+tl+4*tr) + 1) / (tl) ) + d3*0.45/(2);

        for(int i=0;i<18;i++)
        {
            if(i==3)
            {
                realpEE[i]=pEE[i]+s3*k-d3*0.45*k;
            }
            else if(i==9)
            {
                realpEE[i]=pEE[i]+s3*k-d3*0.45*k;
            }
            else if(i==15)
            {
                realpEE[i]=pEE[i]+s3*k-d3*0.45*k;
            }

            else if(i==4 || i==10)
            {
                realpEE[i]=pEE[i]+param.h+param.h1*sin(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(param.totalCount+param.d2*param.totalCount/(param.d))));
            }
            else if(i==16)
            {
                realpEE[i]=pEE[i]+param.h+param.h1*sin(PI/2-PI/2*cos(PI*(param.d)*(param.count-(6*param.totalCount+tl+4*tr)+1)/(2*param.totalCount*param.d2)+PI/2-PI*(param.d)/(2*param.d2)));
            }
            else if(i==5 || i==11)
            {
                realpEE[i]=pEE[i]-(-2*param.d-2*d3-(param.d+param.d2)/2+(param.d+param.d2)/2*cos(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(param.totalCount+param.d2*param.totalCount/(param.d)))));
            }
            else if(i==17)
            {
                realpEE[i]=pEE[i]-(-2*param.d-2*d3-param.d+param.d2*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(6*param.totalCount+tl+4*tr)+1)/(2*param.totalCount*param.d2)+PI/2-PI*(param.d)/(2*param.d2))));
            }
            else if(i==1 || i==7 || i==13)
            {
                realpEE[i]=pEE[i]+param.h;
            }
            else if(i==2 || i==8 || i==14)
            {
                realpEE[i]=pEE[i]-(-3*param.d-param.d2-2*d3);
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
    }

    NewPEE[0] = realpEE[15] - 0.6;
    NewPEE[1] = realpEE[16] ;
    NewPEE[2] = realpEE[17] - 1.3;
    NewPEE[3] = -realpEE[12];
    NewPEE[4] = realpEE[13];
    NewPEE[5] = realpEE[14];
    NewPEE[6] = realpEE[9] - 0.6;
    NewPEE[7] = realpEE[10];
    NewPEE[8] = realpEE[11] + 1.3;
    NewPEE[9] = realpEE[6] + 0.6;
    NewPEE[10] = realpEE[7];
    NewPEE[11] = realpEE[8] - 1.3;
    NewPEE[12] = -realpEE[3];
    NewPEE[13] = realpEE[4];
    NewPEE[14] = realpEE[5];
    NewPEE[15] = realpEE[0] + 0.6;
    NewPEE[16] = realpEE[1];
    NewPEE[17] = realpEE[2] + 1.3;

    //    rt_printf("%f, %f, %f, %f, %f, %f, %f, %f, %f,%f, %f, %f, %f, %f, %f, %f, %f, %f\n", realpEE[0], realpEE[1], realpEE[2], realpEE[3], realpEE[4], realpEE[5], realpEE[6], realpEE[7], realpEE[8], realpEE[9], realpEE[10], realpEE[11], realpEE[12], realpEE[13], realpEE[14], realpEE[15], realpEE[16], realpEE[17]);

    //    rt_printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", NewPEE[0], NewPEE[1], NewPEE[2], NewPEE[3], NewPEE[4], NewPEE[5], NewPEE[6], NewPEE[7], NewPEE[8], NewPEE[9], NewPEE[10], NewPEE[11], NewPEE[12], NewPEE[13], NewPEE[14], NewPEE[15], NewPEE[16], NewPEE[17]);
//rt_printf(" %f, %f, %f, %f, %f, %f\n", NewPEE[0], NewPEE[3], NewPEE[6],  NewPEE[9], NewPEE[12],NewPEE[15]);
    robot.SetPeb(realBody,robot.ground(),"213");
    robot.SetWa(0);
    robot.SetPee(NewPEE);

    //    robot.SetPeb(realBody);
    //    robot.SetPee(realpEE);
    /*rt_printf("realpEE\n");
    for(int i=0;i<18;i++)
   {
       rt_printf("%f\n",realpEE[i]);

    }
/*rt_printf("realbody\n");
    for(int i=0;i<3;i++)
   {
       rt_printf("%f\n",realBody[i]);

    }*/


    /*    rt_printf("%d\t%d\t%d\t%d\t%d\n"
              ,param.count
              , 7*param.totalCount+2*(int)tl+4*(int)tr - param.count - 1
              ,param.totalCount
              ,(int)tl
              ,(int)tr);*/
    return 7*param.totalCount+2*(int)tl+4*(int)tr - param.count - 1;
}

int movedownstairs(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{
    auto &robot = static_cast<Robots::RobotTypeIII &>(model);
    auto &param = static_cast<const Movedownstairs &>(param_in);// change PlanParamBase to MoveupstairsParam

    static aris::dynamic::FloatMarker beginMaker{ robot.ground() };

    static double beginBodyPE213[6];
    static double pEE[18];
    static double l = 0.65;
    static double d3=l-param.d-0.5*param.d2+0.05;
    static int tl=param.d2*param.totalCount/(param.d);
    static int tr=d3*param.totalCount/(param.d);
    static double realBody[6];
    static double NewPEE[18];
    static double realpEE[18];
    //static double H=0.16;
    static double k;

    if(param.h==0.25)
    {
        k=1;
    }
    else
    {
        k=0 ;
    }

    if(param.count==0)
    {
        beginMaker.setPrtPm(*robot.body().pm());
        beginMaker.update();

        robot.GetPeb(beginBodyPE213,robot.ground(),"213");
        robot.GetPee(pEE);
        memcpy(realpEE,pEE,sizeof(realpEE));
        memcpy(realBody,beginBodyPE213,sizeof(beginBodyPE213));
    }

    if(param.count>0 && param.count<param.totalCount)
    {
        double s = -(-(param.h3 / 2)*cos(PI * (param.count + 1) / (param.totalCount) ) + param.h3/ 2);
        realBody[1]=beginBodyPE213[1]+s;

    }

    else if(param.count>param.totalCount && param.count<2*param.totalCount)
    {

        double c=-(0.5*param.w*PI / (180*2))*cos(PI * (param.count-param.totalCount + 1) / (param.totalCount) ) +0.5* param.w*PI/(180*2);
        realBody[4]=beginBodyPE213[4]+c;
        realBody[1]=beginBodyPE213[1]-param.h3;

        double s1=-(d3*0.18 / (2))*cos(PI * (param.count-(param.totalCount) + 1) / (param.totalCount) ) + d3*0.18/( 2);
        realBody[2]=beginBodyPE213[2]-(-(param.d-param.d*cos(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(2*param.totalCount))))/2)+s1*k;

        double s3=-(d3*0.25 / (2))*cos(PI * (param.count-(param.totalCount) + 1) / (param.totalCount) ) + d3*0.25/( 2);
        for(int i=0;i<18;i++)
        {    if(i==0)
            {
               realpEE[i]=pEE[i]+s3*k;
           }
           else if(i==6)
           {
               realpEE[i]=pEE[i]+s3*k;
           }
           else if(i==12)
           {
               realpEE[i]=pEE[i]+s3*k;
           }
            else if(i==1)
            {
                realpEE[i]=pEE[i]+(param.h1)*sin(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(2*param.totalCount)));
            }
            else if (i == 7)
            {
                realpEE[i] = pEE[i] + (param.h1)*sin(PI / 2 - PI / 2 * cos(PI*(param.count-param.totalCount + 1) / (param.totalCount + param.d2*param.totalCount / (param.d)))) ;
            }
            else if(i== 13)
            {
                realpEE[i]=pEE[i]+(param.h1)*sin(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(param.totalCount+param.d2*param.totalCount/(param.d))));
            }
            else if(i== 2 )
            {
                realpEE[i]=pEE[i]-(-param.d+param.d*cos(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(2*param.totalCount))));
            }
            else if(i==  8 )
            {
                realpEE[i]=pEE[i]-(-(param.d+param.d2)/2+(param.d+param.d2)/2*cos(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(param.totalCount+param.d2*param.totalCount/(param.d)))));
            }
            else if(i==  14)
            {
                realpEE[i]=pEE[i]-(-(param.d+param.d2)/2+(param.d+param.d2)/2*cos(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(param.totalCount+param.d2*param.totalCount/(param.d)))));
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
        /* rt_printf("realpEE\n");
       for(int i=0;i<18;i++)
      {
          rt_printf("%f\n",realpEE[i]);

       }*/
        /*rt_printf("realbody\n");
       for(int i=0;i<6;i++)
      {
          rt_printf("%f\n",realBody[i]);

       }*/

    }

    else if(param.count>=2*param.totalCount && param.count<2*param.totalCount+tl)
    {   double c=-(0.5*param.w*PI / (180*2))*cos(PI * (param.count-2*param.totalCount + 1) / (tl) ) + 0.5*param.w*PI/(180* 2);
        realBody[4]=beginBodyPE213[4]+c+0.5*param.w*PI/180;
        realBody[1]=beginBodyPE213[1]-param.h3;

        double s1=-(d3*0.18 / (2))*cos(PI * (param.count-2*param.totalCount + 1) / (tl) ) + d3*0.18/( 2);
        realBody[2]=beginBodyPE213[2]-(-(param.d-param.d2*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-param.totalCount+1)/(2*param.totalCount*param.d2)+PI/2-PI*(param.d)/(2*param.d2))))/2)+0.18*d3*k+s1*k;

        double s3=-(d3*0.25 / (2))*cos(PI * (param.count-2*param.totalCount + 1) / (tl) ) + d3*0.25/( 2);

        for(int i=0;i<18;i++)
        {
           if(i==0)
            {
              realpEE[i]=pEE[i]+d3*0.25*k-s3*k;
           }
           else if(i==6)
           {
               realpEE[i]=pEE[i]+d3*0.25*k-s3*k;
           }
           else if(i==12)
           {
              realpEE[i]=pEE[i]+d3*0.25*k-s3*k;
           }

           else if(i==1 )
            {
                realpEE[i]=pEE[i]-param.h+(param.h1+param.h)*sin(PI/2-PI/2*cos(PI*(param.d)*(param.count-param.totalCount+1)/(2*param.totalCount*param.d2)+PI/2-PI*(param.d)/(2*param.d2)));

            }
            else if (i == 7)
            {
                realpEE[i] = pEE[i] + ( param.h1)*sin(PI / 2 - PI / 2 * cos(PI*(param.count-param.totalCount+ 1) / (param.totalCount + param.d2*param.totalCount / (param.d)))) ;
            }
            else if(i==13 )
            {
                realpEE[i]=pEE[i]+(param.h1)*sin(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(param.totalCount+param.d2*param.totalCount /(param.d))));
            }
            else if(i== 2 )
            {
                realpEE[i]=pEE[i]-(-param.d+param.d2*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-param.totalCount+1)/(2*param.totalCount*param.d2)+PI/2-PI*(param.d)/(2*param.d2))));

            }
            else if(i==  8 )
            {
                realpEE[i]=pEE[i]-(-(param.d+param.d2)/2+(param.d+param.d2)/2*cos(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(param.totalCount+param.d2*param.totalCount /(param.d)))));
            }
            else if(i==  14)
            {
                realpEE[i]=pEE[i]-(-(param.d+param.d2)/2+(param.d+param.d2)/2*cos(PI/2-PI/2*cos(PI*(param.count-param.totalCount+1)/(param.totalCount+param.d2*param.totalCount /(param.d)))));
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
        /*rt_printf("realpEE\n");
              for(int i=0;i<18;i++)
             {
                 rt_printf("%f\n",realpEE[i]);

              }*/
        /* rt_printf("realbody\n");
              for(int i=0;i<6;i++)
             {
                 rt_printf("%f\n",realBody[i]);
              }*/
    }

    else if( param.count>=2*param.totalCount+tl && param.count<3*param.totalCount+tl)
    {   double c=-(param.w*PI / (180*2*10))*cos(PI * (param.count-(2*param.totalCount+tl) + 1) / (param.totalCount) ) + param.w*PI/(180* 2*10);
        realBody[4]=beginBodyPE213[4]+10.0/10.0*param.w*PI/180-c;

        realBody[1]=beginBodyPE213[1]-param.h3;

        double s1=-(d3*0.0/ (2))*cos(PI * (param.count-(2*param.totalCount+tl) + 1) / (param.totalCount) ) + d3*0.0/( 2);
        realBody[2]=beginBodyPE213[2]-(-((param.d+param.d2)/2+param.d-(param.d)*cos(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(2*param.totalCount))))/2 - (param.d + param.d2) / 4)+0.36*d3*k-s1*k;
       double s3=-(d3*0.5/ (2))*cos(PI * (param.count-(2*param.totalCount+tl) + 1) / (param.totalCount) ) + d3*0.5/( 2);

        for(int i=0;i<18;i++)
        {
            if(i==9)
             {
               realpEE[i]=pEE[i]-s3*k;
            }
            else if(i==3)
            {
                realpEE[i]=pEE[i]-s3*k;
            }
            else if(i==15)
            {
               realpEE[i]=pEE[i]-s3*k*0.5;
            }

            else if(i==10)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(2*param.totalCount)));
            }
            else if(i== 4 || i== 16)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i== 11 )
            {
                realpEE[i]=pEE[i]-(-param.d+param.d*cos(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(2*param.totalCount))));
            }
            else if(i==  5 || i==17)
            {
                realpEE[i]=pEE[i]-(-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==1 )
            {
                realpEE[i]=pEE[i]-param.h;
            }
            else if(i==2 || i== 8 ||i== 14)
            {
                realpEE[i]=pEE[i]-(-param.d-param.d2);
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
        /*rt_printf("realpEE\n");
            for(int i=0;i<18;i++)
           {
               rt_printf("%f\n",realpEE[i]);

            }*/
        /* rt_printf("realbody\n");
            for(int i=0;i<6;i++)
           {
               rt_printf("%f\n",realBody[i]);
            }*/

    }

    else if(param.count>=3*param.totalCount+tl && param.count<3*param.totalCount+tl+tr)
    {   double c=-(param.w*PI / (180*2*10))*cos(PI * (param.count-(3*param.totalCount+tl) + 1) / (tr) ) + param.w*PI/(180* 2*10);
        realBody[4]=beginBodyPE213[4]+9.0/10.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]-param.h3;

        double s1=-(d3*0.0 / (2))*cos(PI * (param.count-(3*param.totalCount+tl) + 1) / (tr) ) + d3*0.0/( 2);
        realBody[2]=beginBodyPE213[2]-(-((param.d+param.d2)/2+param.d-d3*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(2*param.totalCount+tl)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))))/2 - (param.d + param.d2) / 4)+d3*0.36*k-s1*k;
         double s3=-(d3*0.5 / (2))*cos(PI * (param.count-(3*param.totalCount+tl) + 1) / (tr) ) + d3*0.5/( 2);

        for(int i=0;i<18;i++)
        {

            if(i==9)
             {
               realpEE[i]=pEE[i]-d3*0.5*k+s3*k;
            }
            else if(i==3)
            {
                realpEE[i]=pEE[i]-d3*0.5*k+s3*k;
            }
            else if(i==15)
            {
               realpEE[i]=pEE[i]-d3*0.5*k*0.5+s3*k*0.5;
            }
            else if(i==10 )
            {
                realpEE[i]=pEE[i]-param.h+(param.h+param.h2)*sin(PI/2-PI/2*cos(PI*(param.d)*(param.count-(2*param.totalCount+tl)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3)));

            }
            else if(i== 4 || i==16)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i== 11 )
            {
                realpEE[i]=pEE[i]-(-param.d+d3*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(2*param.totalCount+tl)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))));
            }
            else if(i==  5 || i==17)
            {
                realpEE[i]=pEE[i]-(-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(2*param.totalCount+tl)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==1)
            {
                realpEE[i]=pEE[i]-param.h;
            }
            else if(i==2 ||i== 8 || i==14)
            {
                realpEE[i]=pEE[i]-(-param.d-param.d2);
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
        /* rt_printf("realpEE\n");
                for(int i=0;i<18;i++)
               {
                   rt_printf("%f\n",realpEE[i]);

                }
         /* rt_printf("realbody\n");
                for(int i=0;i<3;i++)
               {
                   rt_printf("%f\n",realBody[i]);
                }*/

    }

    else if(param.count>=3*param.totalCount+tl+tr && param.count<4*param.totalCount+tl+tr)
    {   double c=-(param.w*PI / (180*2*10))*cos(PI * (param.count-(3*param.totalCount+tl+tr) + 1) / (param.totalCount) ) + param.w*PI/(180* 2*10);
        realBody[4]=beginBodyPE213[4]+8.0/10.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]-param.h3;

        double s1=-(d3*0.0 / (2))*cos(PI * (param.count-(3*param.totalCount+tl+tr) + 1) / (param.totalCount) ) + d3*0.0/( 2);
        realBody[2]=beginBodyPE213[2]-(-((2*param.d+param.d2+d3)/2+param.d-param.d*cos(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(2*param.totalCount))))/2 - (2 * param.d + param.d2 + d3) / 4)+d3*0.36*k-s1*k;
        for(int i=0;i<18;i++)
        {
            if(i==1)
            {
                realpEE[i]=pEE[i]-param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==7)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==13)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(2*param.totalCount)));
            }
            else if(i==2 ||i== 8)
            {
                realpEE[i]=pEE[i]-(-param.d-param.d2-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==14)
            {
                realpEE[i]=pEE[i]-(-param.d-param.d2-(l-param.d2)+(l-param.d2)*cos(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(2*param.totalCount))));
            }
            else if(i==10)
            {
                realpEE[i]=pEE[i]-param.h;
            }
            else if(i==5 || i==11 || i==17)
            {
                realpEE[i]=pEE[i]-(-param.d-d3);
            }
            else
            {
                realpEE[i]=pEE[i];
            }

        }
        /* rt_printf("realpEE\n");
              for(int i=0;i<18;i++)
             {
                 rt_printf("%f\n",realpEE[i]);

              }
        /*rt_printf("realbody\n");
              for(int i=0;i<3;i++)
             {
                 rt_printf("%f\n",realBody[i]);
              }*/

    }

    else if(param.count>=4*param.totalCount+tl+tr && param.count<4*param.totalCount+tl+2*tr)
    {   double c=-(param.w*PI / (180*2*10))*cos(PI * (param.count-(4*param.totalCount+tl+tr) + 1) / (tr) ) + param.w*PI/(180* 2*10);
        realBody[4]=beginBodyPE213[4]+7.0/10.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]-param.h3;

        realBody[2]=beginBodyPE213[2]-(-((2*param.d+param.d2+d3)/2+param.d-d3*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(3*param.totalCount+tl+tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))))/2 - (2 * param.d + param.d2 + d3) / 4)+d3*0.36*k;


        for(int i=0;i<18;i++)
        {
            if(i==1)
            {
                realpEE[i]=pEE[i]-param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==7)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==13)
            {
                realpEE[i]=pEE[i]-param.h+(param.h+param.h2)*sin(PI/2-PI/2*cos(PI*(param.d)*(param.count-(3*param.totalCount+tl+tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3)));
            }
            else if(i==2 || i==8)
            {
                realpEE[i]=pEE[i]-(-param.d-param.d2-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(3*param.totalCount+tl+tr)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==14)
            {
                realpEE[i]=pEE[i]-(-param.d-param.d2-(l-param.d2)+(param.d+d3-(l-param.d2))*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(3*param.totalCount+tl+tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))));
            }
            else if(i==10)
            {
                realpEE[i]=pEE[i]-param.h;
            }
            else if(i==5 || i==11 || i== 17)
            {
                realpEE[i]=pEE[i]-(-param.d-d3);
            }
            else
            {
                realpEE[i]=pEE[i];
            }

        }
        /*rt_printf("realpEE\n");
                for(int i=0;i<18;i++)
               {
                   rt_printf("%f\n",realpEE[i]);

                }*/
    }

    else if(param.count>=4*param.totalCount+tl+2*tr && param.count<5*param.totalCount+tl+2*tr)
    {  double c=-(param.w*PI / (180*2*10))*cos(PI * (param.count-(4*param.totalCount+tl+2*tr) + 1) / (param.totalCount) ) + param.w*PI/(180* 2*10);
        realBody[4]=beginBodyPE213[4]+6.0/10.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]-param.h3;

        double s1=-(d3*0.2 / (2))*cos(PI * (param.count-(4*param.totalCount+tl+2*tr) + 1) / (param.totalCount) ) + d3*0.2/( 2);
        realBody[2]=beginBodyPE213[2]-(-((3*param.d+param.d2+2*d3)/2+param.d-param.d*cos(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(2*param.totalCount))))/2- (3 * param.d + param.d2 + 2 * d3)/4)+d3*0.36*k-s1*k;
        for(int i=0;i<18;i++)
        {
            if(i== 10 )
            {
                realpEE[i]=pEE[i]-param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==16)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==4)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(2*param.totalCount)));
            }
            else if(i==11 ||i== 17)
            {
                realpEE[i]=pEE[i]-(-param.d-d3-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==5)
            {
                realpEE[i]=pEE[i]-(-param.d-d3-(l-d3)+(l-d3)*cos(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(2*param.totalCount))));
            }
            else if(i==2 ||i== 8 || i==14 )
            {
                realpEE[i]=pEE[i]-(-2*param.d-param.d2-d3);
            }
            else if(i==1 || i==13)
            {
                realpEE[i]=pEE[i]-param.h;
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }

    }

    else if(param.count>=5*param.totalCount+tl+2*tr && param.count<5*param.totalCount+tl+3*tr)
    {  double c=-(param.w*PI / (180*2*10))*cos(PI * (param.count-(5*param.totalCount+tl+2*tr) + 1) / (tr) ) + param.w*PI/(180* 2*10);
        realBody[4]=beginBodyPE213[4]+5.0/10.0*param.w*PI/180-c;

        double s2=-(d3*0.2 / (2))*cos(PI * (param.count-(5*param.totalCount+tl+2*tr) + 1) / (tr) ) + d3*0.2/( 2);
        realBody[1]=beginBodyPE213[1]-param.h3-s2*k;

        double s1=-(d3*0.16 / (2))*cos(PI * (param.count-(5*param.totalCount+tl+2*tr) + 1) / (tr) ) + d3*0.16/( 2);
        realBody[2]=beginBodyPE213[2]-(-((3*param.d+param.d2+2*d3)/2+param.d-d3*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(4*param.totalCount+tl+2*tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))))/2- (3 * param.d + param.d2 + 2 * d3)/4)+d3*0.16*k-s1*k;

       double s3=-(d3*0.25 / (2))*cos(PI * (param.count-(5*param.totalCount+tl+2*tr) + 1) / (tr) ) + d3*0.25/( 2);
        for(int i=0;i<18;i++)
        {   if(i==15)
            {
                realpEE[i]=pEE[i]-s3*k;
            }
            else if(i== 10 )
            {
                realpEE[i]=pEE[i]-param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==16)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==4)
            {
                realpEE[i]=pEE[i]-param.h+(param.h+param.h2)*sin(PI/2-PI/2*cos(PI*(param.d)*(param.count-(4*param.totalCount+tl+2*tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3)));
            }
            else if(i==11 || i==17)
            {
                realpEE[i]=pEE[i]-(-param.d-d3-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(4*param.totalCount+tl+2*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==5)
            {
                realpEE[i]=pEE[i]-(-param.d-d3-(l-d3)+(param.d+d3-(l-d3))*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(4*param.totalCount+tl+2*tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))));
            }
            else if(i==2 || i==8 ||i== 14 )
            {
                realpEE[i]=pEE[i]-(-2*param.d-param.d2-d3);
            }
            else if(i==1 || i==13)
            {
                realpEE[i]=pEE[i]-param.h;
            }
            else
            {
                realpEE[i]=pEE[i];
            }

        }
      /*  rt_printf("realbody\n");
                    for(int i=0;i<6;i++)
                   {
                       rt_printf("%f\n",realBody[i]);
                    }*/
    }

    else if(param.count>=5*param.totalCount+tl+3*tr && param.count<6*param.totalCount+tl+3*tr)
    {   double c=-(param.w*PI / (180*2*10))*cos(PI * (param.count-(5*param.totalCount+tl+3*tr) + 1) / (param.totalCount) ) + param.w*PI/(180* 2*10);
        realBody[4]=beginBodyPE213[4]+4.0/10.0*param.w*PI/180-c;

        double s2=-(d3*0.1 / (2))*cos(PI * (param.count-(5*param.totalCount+tl+3*tr) + 1) / (param.totalCount) ) + d3*0.1/( 2);
        realBody[1]=beginBodyPE213[1]-param.h3-d3*0.2*k+s2*k;

        double s1=-(d3*0.0 / (2))*cos(PI * (param.count-(5*param.totalCount+tl+3*tr) + 1) / (param.totalCount) ) + d3*0.0/( 2);
        realBody[2]=beginBodyPE213[2]-(-((4*param.d+param.d2+3*d3)/2+param.d-param.d*cos(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(2*param.totalCount))))/2- (4 * param.d + param.d2 + 3 * d3)/4)+d3*0.0*k-s1*k;

        double s3=-(d3*0.6 / (2))*cos(PI * (param.count-(5*param.totalCount+tl+3*tr) + 1) / (param.totalCount) ) + d3*0.6/( 2);
        for(int i=0;i<18;i++)
        {   if(i==6)
            {
                realpEE[i]=pEE[i]+s3*k;
            }
            else if(i==1 || i==13)
            {
                realpEE[i]=pEE[i]-param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==7)
            {
                realpEE[i]=pEE[i]+(param.h2)*sin(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(2*param.totalCount)));
            }
            else if(i==2 || i==14)
            {
                realpEE[i]=pEE[i]-(-2*param.d-param.d2-d3-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==8)
            {
                realpEE[i]=pEE[i]-(-2*param.d-param.d2-d3-(2*l+param.d-(2*param.d+param.d2+d3))+(2*l+param.d-(2*param.d+param.d2+d3))*cos(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(2*param.totalCount))));
            }
            else if(i==4 || i==10)
            {
                realpEE[i]=pEE[i]-param.h;
            }
            else if(i==5 ||i== 11 || i==17)
            {
                realpEE[i]=pEE[i]-(-2*param.d-2*d3);
            }
            else if(i==15)
            {
                realpEE[i]=pEE[i]-d3*0.25*k;
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
        /*rt_printf("realbody\n");
                            for(int i=0;i<6;i++)
                           {
                               rt_printf("%f\n",realBody[i]);
                            }*/


    }

    else if(param.count>=6*param.totalCount+tl+3*tr && param.count<6*param.totalCount+tl+4*tr)
    {    double c=-(param.w*PI / (180*2*10))*cos(PI * (param.count-(6*param.totalCount+tl+3*tr) + 1) / (tr) ) + param.w*PI/(180* 2*10);
        realBody[4]=beginBodyPE213[4]+3.0/10.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]-param.h3-d3*0.1*k;

        double s1=-(d3*0.0 / (2))*cos(PI * (param.count-(6*param.totalCount+tl+3*tr) + 1) / (tr) ) + d3*0.0/( 2);
        double s3=-(d3*0.6 / (2))*cos(PI * (param.count-(6*param.totalCount+tl+3*tr) + 1) / (tr) ) + d3*0.6/( 2);
        realBody[2]=beginBodyPE213[2]-(-((4*param.d+param.d2+3*d3)/2+param.d-d3*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(5*param.totalCount+tl+3*tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))))/2- (4 * param.d + param.d2 + 3 * d3)/4)+d3*0.0*k+s1*k;
        for(int i=0;i<18;i++)
        {   if(i==6)
            {
                realpEE[i]=pEE[i]-s3*k+d3*0.6*k;
            }

            else if(i==1 || i==13)
            {
                realpEE[i]=pEE[i]-param.h+param.h2*sin(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d))));
            }
            else if(i==7)
            {
                realpEE[i]=pEE[i]-param.h+(param.h+param.h2)*sin(PI/2-PI/2*cos(PI*(param.d)*(param.count-(5*param.totalCount+tl+3*tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3)));
            }
            else if(i==2 || i==14)
            {
                realpEE[i]=pEE[i]-(-2*param.d-param.d2-d3-(param.d+d3)/2+(param.d+d3)/2*cos(PI/2-PI/2*cos(PI*(param.count-(5*param.totalCount+tl+3*tr)+1)/(param.totalCount+d3*param.totalCount/(param.d)))));
            }
            else if(i==8)
            {
                realpEE[i]=pEE[i]-(-2*param.d-param.d2-d3-(2*l+param.d-(2*param.d+param.d2+d3))+(param.d+d3-(2*l+param.d-(2*param.d+param.d2+d3)))*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(5*param.totalCount+tl+3*tr)+1)/(2*param.totalCount*d3)+PI/2-PI*(param.d)/(2*d3))));
            }
            else if(i==4 || i==10)
            {
                realpEE[i]=pEE[i]-param.h;
            }
            else if(i==5 || i==11 || i==17)
            {
                realpEE[i]=pEE[i]-(-2*param.d-2*d3);
            }
            else if(i==15)
            {
                realpEE[i]=pEE[i]-d3*0.25*k;
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
        /* rt_printf("realbody\n");
                            for(int i=0;i<6;i++)
                           {
                               rt_printf("%f\n",realBody[i]);
                            }*/

    }

    else if(param.count>=6*param.totalCount+tl+4*tr && param.count<7*param.totalCount+tl+4*tr)
    {   double c=-(param.w*PI / (180*2*10))*cos(PI * (param.count-(6*param.totalCount+tl+4*tr) + 1) / (param.totalCount) ) + param.w*PI/(180* 2*10);
        realBody[4]=beginBodyPE213[4]+2.0/10.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]-param.h3-d3*0.1*k;
        realBody[2]=beginBodyPE213[2]-(-((5*param.d+param.d2+4*d3)/2+param.d-param.d*cos(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(2*param.totalCount))))/2- (5 * param.d + param.d2 + 4 * d3)/4)+d3*0.0*k;
         double s3=-(d3*0.25 / (2))*cos(PI * (param.count-(6*param.totalCount+tl+4*tr) + 1) / (param.totalCount) ) + d3*0.25/( 2);


        for(int i=0;i<18;i++)
        {
            if(i==15)
           {
                 realpEE[i]=pEE[i]-d3*0.25*k-s3*k;
            }

            else if(i==4 || i==10)
            {
                realpEE[i]=pEE[i]-param.h+param.h1*sin(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(param.totalCount+param.d2*param.totalCount/(param.d))));
            }
            else if(i==16)
            {
                realpEE[i]=pEE[i]+(param.h1)*sin(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(2*param.totalCount)));
            }
            else if(i==5 || i==11)
            {
                realpEE[i]=pEE[i]-(-2*param.d-2*d3-(param.d+param.d2)/2+(param.d+param.d2)/2*cos(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(param.totalCount+param.d2*param.totalCount/(param.d)))));
            }
            else if(i==17)
            {
                realpEE[i]=pEE[i]-(-2*param.d-2*d3-(2*l+param.d-(2*param.d+2*d3))+(2*l+param.d-(2*param.d+2*d3))*cos(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(2*param.totalCount))));
            }
            else if(i==1 || i==7 || i==13)
            {
                realpEE[i]=pEE[i]-param.h;
            }
            else if(i==2 || i==8 || i==14)
            {
                realpEE[i]=pEE[i]-(-3*param.d-param.d2-2*d3);
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
    }

    else if(param.count>=7*param.totalCount+tl+4*tr && param.count<7*param.totalCount+2*tl+4*tr)
    {   double c=-(param.w*PI / (180*2*10))*cos(PI * (param.count-(7*param.totalCount+tl+4*tr) + 1) / (tl) ) + param.w*PI/(180* 2*10);
        realBody[4]=beginBodyPE213[4]+1.0/10.0*param.w*PI/180-c;
        realBody[1]=beginBodyPE213[1]-param.h3-d3*0.1*k;


        double s1=-(d3*0.16 / (2))*cos(PI * (param.count-(7*param.totalCount+tl+4*tr) + 1) / (tl) ) + d3*0.16/( 2);
        realBody[2]=beginBodyPE213[2]-(-((5*param.d+param.d2+4*d3)/2+param.d-param.d2*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(6*param.totalCount+tl+4*tr)+1)/(2*param.totalCount*param.d2)+PI/2-PI*(param.d)/(2*param.d2))))/2- (5 * param.d + param.d2 + 4 * d3)/4)+d3*0.0*k-s1*k;
        double s3=-(d3*0.5 / (2))*cos(PI * (param.count-(7*param.totalCount+tl+4*tr) + 1) / (tl) ) + d3*0.5/( 2);


        for(int i=0;i<18;i++)
        {

            if(i==15)
           {
                 realpEE[i]=pEE[i]-d3*0.5*k+s3*k;
            }
            else if(i==4 || i==10)
            {
                realpEE[i]=pEE[i]-param.h+param.h1*sin(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(param.totalCount+param.d2*param.totalCount/(param.d))));
            }
            else if(i==16)
            {
                realpEE[i]=pEE[i]-param.h+(param.h+param.h1)*sin(PI/2-PI/2*cos(PI*(param.d)*(param.count-(6*param.totalCount+tl+4*tr)+1)/(2*param.totalCount*param.d2)+PI/2-PI*(param.d)/(2*param.d2)));
            }
            else if(i==5 || i==11)
            {
                realpEE[i]=pEE[i]-(-2*param.d-2*d3-(param.d+param.d2)/2+(param.d+param.d2)/2*cos(PI/2-PI/2*cos(PI*(param.count-(6*param.totalCount+tl+4*tr)+1)/(param.totalCount+param.d2*param.totalCount/(param.d)))));
            }
            else if(i==17)
            {
                realpEE[i]=pEE[i]-(-2*param.d-2*d3-(2*l+param.d-(2*param.d+2*d3))+(param.d+param.d2-(2*l+param.d-(2*param.d+2*d3)))*cos(PI/2-PI/2*cos(PI*(param.d)*(param.count-(6*param.totalCount+tl+4*tr)+1)/(2*param.totalCount*param.d2)+PI/2-PI*(param.d)/(2*param.d2))));
            }
            else if(i==1 || i==7 || i==13)
            {
                realpEE[i]=pEE[i]-param.h;
            }
            else if(i==2 || i==8 || i==14)
            {
                realpEE[i]=pEE[i]-(-3*param.d-param.d2-2*d3);
            }
            else
            {
                realpEE[i]=pEE[i];
            }
        }
    }

    NewPEE[0] = realpEE[15] - 0.6;
    NewPEE[1] = realpEE[16] ;
    NewPEE[2] = realpEE[17] - 1.3;
    NewPEE[3] = -realpEE[12];
    NewPEE[4] = realpEE[13];
    NewPEE[5] = realpEE[14];
    NewPEE[6] = realpEE[9] - 0.6;
    NewPEE[7] = realpEE[10];
    NewPEE[8] = realpEE[11] + 1.3;
    NewPEE[9] = realpEE[6] + 0.6;
    NewPEE[10] = realpEE[7];
    NewPEE[11] = realpEE[8] - 1.3;
    NewPEE[12] = -realpEE[3];
    NewPEE[13] = realpEE[4];
    NewPEE[14] = realpEE[5];
    NewPEE[15] = realpEE[0] + 0.6;
    NewPEE[16] = realpEE[1];
    NewPEE[17] = realpEE[2] + 1.3;

    robot.SetPeb(realBody,robot.ground(),"213");
    robot.SetWa(0);
    robot.SetPee(NewPEE);

    return 7*param.totalCount+2*(int)tl+4*(int)tr - param.count - 1;
}


