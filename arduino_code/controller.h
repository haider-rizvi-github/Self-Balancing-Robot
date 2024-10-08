#include "pid.h"
#include "tb6612fng.h"



class controller
{
private:
  TB6612FNG myMotors;
  PIDController pidController;
public:
  controller(TB6612FNG myMotors, PIDController pidController) : 
  myMotors(myMotors), pidController(pidController) {};
  ~controller();
  balance();
};

controller::~controller() {
}

controller::balance(){

}
