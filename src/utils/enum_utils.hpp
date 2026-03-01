//
// Created by Richard Mule on 7/5/25.
//

#pragma once

#include <algorithm>
#include <cinttypes>
#include <execution>
#include <regex>
#include <sstream>
#include <typeinfo>


#include "../../cpp_dependencies/cpp_dependencies.hpp"

namespace enum_utils
{
    using magic_enum::iostream_operators::operator<<;

    template <typename Enumeration>
    auto from_string(const std::string& name, bool use_enum_prefix = false)
    {
        if ( !use_enum_prefix )
            return magic_enum::enum_cast<Enumeration>(name);

        std::string copy_name = "ENUM_" + name;
        return magic_enum::enum_cast<Enumeration>(copy_name);
    }

    template <typename Enumeration>
    std::string to_string(Enumeration const value)
    {
        std::stringstream ss;
        ss << value;
        return ss.str();
    }

    template <typename Enumeration>
    auto to_int(Enumeration const value) -> typename std::underlying_type<Enumeration>::type
    {
        return static_cast<typename std::underlying_type<Enumeration>::type>(value);
    }

    template <typename Enumeration>
    auto increment_enum(Enumeration value, int inc_amount = 1) -> Enumeration
    {
        return static_cast<Enumeration>(to_int(value) + inc_amount);
    }

    template <typename Enumeration>
    constexpr std::vector<Enumeration> rangedEnum(Enumeration start, Enumeration end)
    {
        std::vector<Enumeration> v_enum;
        for ( Enumeration e = start; e <= end; e = increment_enum(e) )
            v_enum.push_back(e);
        return v_enum;
    }

}


