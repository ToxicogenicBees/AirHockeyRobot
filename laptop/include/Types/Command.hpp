#pragma once

#include <unordered_map>
#include <functional>
#include <stdexcept>
#include <string>

class Command {
    private:
        const std::unordered_map<std::string, std::function<void()>> _OPTIONS;

    public:
        /**
         * @brief Create a new command
         * 
         * @param options   The desired command options
         */
        Command(std::unordered_map<std::string, std::function<void()>> options)
            : _OPTIONS(std::move(options)) {}

        /**
         * @brief Run this command with the provided argument
         * 
         * @param argument  The desired argument (blank by default)
         */
        void run(const std::string& argument = "") const {
            auto result = _OPTIONS.find(argument);
            if (result == _OPTIONS.end()) {
                if (argument.empty())
                    throw std::runtime_error("This command does not take a flag/no argument");
                else
                    throw std::runtime_error("Invalid argument '" + argument + "'");
            }
            result->second();
        }
};