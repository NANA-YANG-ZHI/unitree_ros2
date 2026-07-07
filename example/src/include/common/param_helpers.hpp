#ifndef _PARAM_HELPERS_HPP_
#define _PARAM_HELPERS_HPP_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"

// Declare a parameter with a compile-time default and immediately read back
// its (possibly CLI/launch-overridden) value. Used by the excitation nodes so
// tuning knobs like param_range/duration/seed don't require a rebuild.
inline int declare_and_get_int(rclcpp::Node *node, const std::string &name, int default_value)
{
    node->declare_parameter<int>(name, default_value);
    return static_cast<int>(node->get_parameter(name).as_int());
}

inline unsigned declare_and_get_uint(rclcpp::Node *node, const std::string &name, unsigned default_value)
{
    node->declare_parameter<int>(name, static_cast<int>(default_value));
    return static_cast<unsigned>(node->get_parameter(name).as_int());
}

inline double declare_and_get_double(rclcpp::Node *node, const std::string &name, double default_value)
{
    node->declare_parameter<double>(name, default_value);
    return node->get_parameter(name).as_double();
}

inline std::vector<double> declare_and_get_double_array(
    rclcpp::Node *node, const std::string &name, const std::vector<double> &default_value)
{
    node->declare_parameter<std::vector<double>>(name, default_value);
    return node->get_parameter(name).as_double_array();
}

#endif
