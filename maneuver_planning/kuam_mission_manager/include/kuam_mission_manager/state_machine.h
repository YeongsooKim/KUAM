#ifndef __ITEM_DESCRIP_H__
#define __ITEM_DESCRIP_H__

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>

using namespace std;

namespace kuam{
namespace mission{

enum class Mode : int{
    Manual,
    Offboard,
    Emerg,

    ItemNum
};

enum class Status : int{
    Todo,
    Doing,
    Done,

    ItemNum
};

enum class Trans : int{
    Arm,
    Disarm, 
    Docking,
    Undocking,
    Takeoff,
    Landing,
    Flight,
    Transition,

    ItemNum
};

enum class State : int{
    Standby,
    Arm,
    Docking,
    Undocking,
    Takeoff,
    Hovering,
    Flight,
    Transition,
    Landing,

    ItemNum
};


using Task = pair<Trans, Status>;
using TaskList = vector<Task>;


inline string Ascii2Lower(string str)
{
    size_t size = str.size();

    for (char &c : str){
        if (c <= 'Z' && c >= 'A')
            c = c - ('Z' - 'z');
    }
    return str;
}

inline string Ascii2Upper(string str)
{
    size_t size = str.size();

    for (char &c : str){
        if (c >= 'a' && c <= 'z')
            c = c - ('a' - 'A');
    }
    return str;
}

inline string Enum2String(State state)
{
    switch((int)state){
        case (int)State::Standby: return "standby";
        case (int)State::Arm: return "arm";
        case (int)State::Docking: return "docking";
        case (int)State::Undocking: return "undocking";
        case (int)State::Takeoff: return "takeoff";
        case (int)State::Hovering: return "hovering";
        case (int)State::Flight: return "flight";
        case (int)State::Transition: return "transition";
        case (int)State::Landing: return "landing";
        default: return "invalid";
    }
}

inline string Enum2String(Trans trans)
{
    switch((int)trans){
        case (int)Trans::Arm: return "arm";
        case (int)Trans::Disarm: return "disarm";
        case (int)Trans::Docking: return "docking";
        case (int)Trans::Undocking: return "undocking";
        case (int)Trans::Takeoff: return "takeoff";
        case (int)Trans::Landing: return "landing";
        case (int)Trans::Flight: return "flight";
        case (int)Trans::Transition: return "transition";
        default: return "invalid";
    }
}

inline string Enum2String(Mode mode)
{
    switch((int)mode){
        case (int)Mode::Manual: return "manual";
        case (int)Mode::Offboard: return "offboard";
        case (int)Mode::Emerg: return "emerg";
        default: return "invalid";
    }
}

inline string Enum2String(Status status)
{
    switch((int)status){
        case (int)Status::Todo: return "todo";
        case (int)Status::Doing: return "doing";
        case (int)Status::Done: return "done";
        default: return "invalid";
    }
}

inline Trans String2Trans(string str)
{
    str = Ascii2Lower(str);
    if (str == "arm") return Trans::Arm;
    else if (str == "disarm") return Trans::Disarm;
    else if (str == "docking") return Trans::Docking;
    else if (str == "undocking") return Trans::Undocking;
    else if (str == "takeoff") return Trans::Takeoff;
    else if (str == "landing") return Trans::Landing;
    else if (str == "flight") return Trans::Flight;
    else if (str == "transition") return Trans::Transition;
    else ROS_ERROR_STREAM("Fail String2Trans");
}

inline State String2State(string str)
{
    str = Ascii2Lower(str);
    if (str == "standby") return State::Standby;
    else if (str == "arm") return State::Arm;
    else if (str == "docking") return State::Docking;
    else if (str == "takeoff") return State::Takeoff;
    else if (str == "undocking") return State::Undocking;
    else if (str == "hovering") return State::Hovering;
    else if (str == "flight") return State::Flight;
    else if (str == "transition") return State::Transition;
    else if (str == "landing") return State::Landing;
    else ROS_ERROR_STREAM("Fail String2State");
}

}
}
#endif //  __ITEM_DESCRIP_H__