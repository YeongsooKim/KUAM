#ifndef __STATE_MACHINE_H__
#define __STATE_MACHINE_H__

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>

using namespace std;

namespace kuam{
namespace mission{

enum class Mode : int{
    Manual,
    Altitude,
    Offboard,
    Emergy,
    Land,

    ItemNum
};

enum class Status : int{
    Todo,
    Doing,
    Done,

    ItemNum
};

enum class Trans : int{
    Takeoff,
    Flight,
    Landing,

    ItemNum
};

enum class State : int{
    Standby,
    Takeoff,
    Hovering,
    Flight,
    Landing,

    ItemNum
};

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
        case (int)State::Takeoff: return "takeoff";
        case (int)State::Hovering: return "hovering";
        case (int)State::Flight: return "flight";
        case (int)State::Landing: return "landing";
        default: return "invalid";
    }
}

inline string Enum2String(Trans trans)
{
    switch((int)trans){
        case (int)Trans::Takeoff: return "takeoff";
        case (int)Trans::Flight: return "flight";
        case (int)Trans::Landing: return "landing";
        default: return "invalid";
    }
}

inline string Enum2String(Mode mode)
{
    switch((int)mode){
        case (int)Mode::Manual: return "manual";
        case (int)Mode::Altitude: return "altctl";
        case (int)Mode::Offboard: return "offboard";
        case (int)Mode::Emergy: return "emergy";
        case (int)Mode::Land: return "land";
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
    if (str == "takeoff") return Trans::Takeoff;
    else if (str == "flight") return Trans::Flight;
    else if (str == "landing") return Trans::Landing;
    // else ROS_ERROR_STREAM("Fail String2Trans");
}

inline State String2State(string str)
{
    str = Ascii2Lower(str);
    if (str == "standby") return State::Standby;
    else if (str == "takeoff") return State::Takeoff;
    else if (str == "flight") return State::Flight;
    else if (str == "landing") return State::Landing;
    // else ROS_ERROR_STREAM("Fail String2State");
}

}
}
#endif //  __STATE_MACHINE_H__