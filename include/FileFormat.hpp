#pragma once

#include "Types/Matrix.hpp"
#include <sstream>
#include <string>

namespace ML {
    template <class T>
    std::string matrix(const std::string& name, Matrix<T> matrix) {
        std::stringstream stream;
        stream << name << " = " << matrix << ";\n";
        return stream.str();
    }

    std::string points(const std::string& mat1, const std::string& mat2) {
        std::stringstream stream;
        stream << mat1 << ", " << mat2 << ", \".\"";
        return stream.str();
    }

    std::string figure(size_t figure) {
        return "figure(" + std::to_string(figure) + ")\n";
    }

    template <class... Args>
    std::string plot(const Args&... args) {
        std::stringstream stream;
        stream << "plot(";

        ((stream << args << ", "), ...);
        std::string result = stream.str();

        if (result.size() >= 2 && result.substr(result.size() - 2) == ", ")
            result.erase(result.size() - 2);
        result += ");\n";

        return result;
    }

    std::string xlabel(const std::string& label) {
        std::stringstream stream;
        stream << "xlabel('" << label << "');\n";
        return stream.str();
    }

    std::string ylabel(const std::string& label) {
        std::stringstream stream;
        stream << "ylabel('" << label << "');\n";
        return stream.str();
    }

    std::string title(const std::string& title) {
        std::stringstream stream;
        stream << "title('" << title << "');\n";
        return stream.str();
    }
}