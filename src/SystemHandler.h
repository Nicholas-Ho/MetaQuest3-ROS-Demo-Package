#ifndef SYSTEM_HANDLER_H
#define SYSTEM_HANDLER_H

#include "OdeSolver.h"
#include "spring_boxes/UnityUpdate.h"

class SystemHandler {
  private:
    OdeSolver solver1, solver2;
    bool initialised = false;
  public:
    SystemHandler(OdeSolver s1, OdeSolver s2);
    void unity_update_callback(const spring_boxes::UnityUpdate::ConstPtr& msg);
    bool is_initialised();
};

#endif