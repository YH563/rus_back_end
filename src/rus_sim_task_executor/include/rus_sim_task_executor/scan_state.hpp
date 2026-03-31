#pragma once

#include "yasmin/state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace RusScanState {
    // 状态枚举
    enum class StateId
    {

    };

    // 动作结果枚举
    enum class Outcome
    {
        SUCCESS,
        ERROR,
        TIMEOUT,
    };

    // 状态枚举转字符串
    inline std::string Enum2String(StateId stateId)
    {
        switch (stateId) {
            
        }
    }

    // 动作结果枚举转字符串
    inline std::string Enum2String(Outcome outcome)
    {
        switch (outcome) {
            case Outcome::SUCCESS: return "SUCCESS";
            case Outcome::ERROR: return "ERROR";
            case Outcome::TIMEOUT: return "TIMEOUT";
            default: return "UNKNOWN";
        }
    }

}