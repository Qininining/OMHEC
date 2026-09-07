#pragma once

#include "ErrorCode.h"
#include "LogUtil.h"
#include <exception>
#include <string>

namespace MAR {

class RobotException : public std::exception {
  public:
    ErrorCode code;
    std::string msg;
    RobotException() noexcept : code(ErrorCode::OK) {};
    RobotException(ErrorCode code) noexcept { this->code = code; }
    RobotException(ErrorCode code, std::string msg) noexcept {
        this->code = code;
        try {
            this->msg = msg;
        } catch (...) {
            // Ignore string copy errors in noexcept constructor
        }
    }
    RobotException(const RobotException &other) noexcept : code(other.code) {
        try {
            this->msg = other.msg;
        } catch (...) {}
    }
    RobotException &operator=(const RobotException &other) noexcept {
        this->code = other.code;
        try {
            this->msg = other.msg;
            m_what.clear();
        } catch (...) {}
        return *this;
    }

    const char *what() const noexcept override {
        try {
            if (m_what.empty()) {
                if (!this->msg.empty()) {
                    m_what = (this->msg + " Errorcode: " + std::to_string(this->code));
                } else {
                    m_what = ("The Error code: " + std::to_string(this->code));
                }
            }
            return m_what.c_str();
        } catch (...) {
            return "RobotException: Error formatting message";
        }
    }

  private:
    mutable std::string m_what;
};

} // namespace MAR
