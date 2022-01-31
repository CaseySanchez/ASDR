// Generated by gencpp from file coverage_path_planner/make_plan.msg
// DO NOT EDIT!


#ifndef COVERAGE_PATH_PLANNER_MESSAGE_MAKE_PLAN_H
#define COVERAGE_PATH_PLANNER_MESSAGE_MAKE_PLAN_H

#include <ros/service_traits.h>


#include <coverage_path_planner/make_planRequest.h>
#include <coverage_path_planner/make_planResponse.h>


namespace coverage_path_planner
{

struct make_plan
{

typedef make_planRequest Request;
typedef make_planResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct make_plan
} // namespace coverage_path_planner


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::coverage_path_planner::make_plan > {
  static const char* value()
  {
    return "0002bc113c0259d71f6cf8cbc9430e18";
  }

  static const char* value(const ::coverage_path_planner::make_plan&) { return value(); }
};

template<>
struct DataType< ::coverage_path_planner::make_plan > {
  static const char* value()
  {
    return "coverage_path_planner/make_plan";
  }

  static const char* value(const ::coverage_path_planner::make_plan&) { return value(); }
};


// service_traits::MD5Sum< ::coverage_path_planner::make_planRequest> should match
// service_traits::MD5Sum< ::coverage_path_planner::make_plan >
template<>
struct MD5Sum< ::coverage_path_planner::make_planRequest>
{
  static const char* value()
  {
    return MD5Sum< ::coverage_path_planner::make_plan >::value();
  }
  static const char* value(const ::coverage_path_planner::make_planRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::coverage_path_planner::make_planRequest> should match
// service_traits::DataType< ::coverage_path_planner::make_plan >
template<>
struct DataType< ::coverage_path_planner::make_planRequest>
{
  static const char* value()
  {
    return DataType< ::coverage_path_planner::make_plan >::value();
  }
  static const char* value(const ::coverage_path_planner::make_planRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::coverage_path_planner::make_planResponse> should match
// service_traits::MD5Sum< ::coverage_path_planner::make_plan >
template<>
struct MD5Sum< ::coverage_path_planner::make_planResponse>
{
  static const char* value()
  {
    return MD5Sum< ::coverage_path_planner::make_plan >::value();
  }
  static const char* value(const ::coverage_path_planner::make_planResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::coverage_path_planner::make_planResponse> should match
// service_traits::DataType< ::coverage_path_planner::make_plan >
template<>
struct DataType< ::coverage_path_planner::make_planResponse>
{
  static const char* value()
  {
    return DataType< ::coverage_path_planner::make_plan >::value();
  }
  static const char* value(const ::coverage_path_planner::make_planResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // COVERAGE_PATH_PLANNER_MESSAGE_MAKE_PLAN_H
