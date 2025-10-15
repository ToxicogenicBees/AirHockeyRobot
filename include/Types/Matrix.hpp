#pragma once

#include <initializer_list>
#include <stdexcept>
#include <stdint.h>
#include <ostream>
#include <vector>

// NOTE: Elements are stored in column-major order for MATLAB compatibility.

namespace MatrixVector {
    using MatrixVectorType = bool;

    MatrixVectorType column = true;
    MatrixVectorType row = false;
}

template <class T>
class Matrix {
    private:
        size_t _rows, _cols;      // Matrix rows and columns
        std::vector<T> _data;     // Matrix entries

        // Access a constant matrix element
        // Throws std::out_of_range if row >= rows() or col >= cols()
        const T& _access(size_t row, size_t col) const;

        // Access a matrix element
        // Throws std::out_of_range if row >= rows() or col >= cols()
        T& _access(size_t row, size_t col);

    public:
        /***
         * @brief Creates a matrix with the desired rows and columns, populated with an initial value
         * 
         * @param rows      The number of rows in the matrix
         * @param cols      The number of columns in the matrix
         * @param init_val  The initial value populating the matrix
         */
        Matrix(size_t rows, size_t cols, const T& init_val) noexcept;

        /***
         * @brief Creates a matrix with the desired rows and columns
         * 
         * @param rows      The number of rows in the matrix
         * @param cols      The number of columns in the matrix
         */
        Matrix(size_t rows, size_t cols) noexcept;

        /***
         * @brief Creates a vector of the desired size, populated with an initial value
         * 
         * @param size      The number of elements in the vector
         * @param type      The vector type (column or row)
         */
        Matrix(size_t size, const T& init_val, MatrixVector::MatrixVectorType type = MatrixVector::column) noexcept;

        /***
         * @brief Creates a vector of the desired size
         * 
         * @param size      The number of elements in the vector
         * @param type      The vector type (column or row)
         */
        Matrix(size_t size, MatrixVector::MatrixVectorType type = MatrixVector::column) noexcept;

        /***
         * @brief Creates a matrix from the provided arguments
         *        Throws std::invalid_argument if the inner lists are not all the same size
         * 
         * @param values    An initializer list of array values
         */
        Matrix(const std::initializer_list<std::initializer_list<T>>& values);

        /***
         * @brief Creates a vector from the provided arguments
         * 
         * @param vec       An initializer list of array values
         * @param type      The vector type (column/row)
         */
        Matrix(const std::initializer_list<T>& vec, MatrixVector::MatrixVectorType type = MatrixVector::column) noexcept;

        /***
         * @brief Creates a vector from the provided arguments
         * 
         * @param type      The vector type (column/row)
         * @param vec       An initializer list of array values
         */
        Matrix(MatrixVector::MatrixVectorType type, const std::initializer_list<T>& vec) noexcept;

        /***
         * @brief Creates a matrix copying the values of another matrix
         * 
         * @param m     The matrix being copied
         */
        Matrix(const Matrix& m) noexcept;

        /***
         * @brief Creates an empty matrix with zero rows and columns
         */
        Matrix() noexcept;

        /***
         * @brief Overloaded function operator, accessing an element of a constant 2D matrix
         *        Throws std::out_of_range if the provided indexes are invalid
         * 
         * @param row   The desired row
         * @param col   The desired column
         * 
         * @return Constant reference to the desired item
         */
        constexpr const T& operator()(size_t row, size_t col) const;

        /***
         * @brief Overloaded function operator, accessing an element of a constant column/row vector
         *        Throws std::out_of_range if the provided indexes are invalid
         *        Throws std::invalid_argument if the matrix is not a column/row vector
         * 
         * @param index The desired element in the column/row vector
         * 
         * @return Constant reference to the desired item
         */
        constexpr const T& operator()(size_t index) const;

        /***
         * @brief Overloaded function operator, accessing an element of a 2D matrix
         *        Throws std::out_of_range if the provided indexes are invalid
         * 
         * @param row   The desired row
         * @param col   The desired column
         * 
         * @return Reference to the desired item
         */
        T& operator()(size_t row, size_t col);

        /***
         * @brief Overloaded function operator, accessing an element of a column/row vector
         *        Throws std::out_of_range if the provided indexes are invalid
         *        Throws std::invalid_argument if the matrix is not a column/row vector
         * 
         * @param index The desired element in the column/row vector
         * 
         * @return Reference to the desired item
         */
        T& operator()(size_t index);

        /***
         * @brief Overloaded comparison operator
         * 
         * @param m     Another matrix object
         * 
         * @return Whether the two matrices are equal
         */
        constexpr bool operator==(const Matrix& m) const noexcept;

        /***
         * @brief Overloaded inequality comparison operator
         * 
         * @param m     Another matrix object
         * 
         * @return Whether the two matrices are not equal
         */
        constexpr bool operator!=(const Matrix& m) const noexcept;

        /***
         * @brief Overloaded assignment operator
         * 
         * @param m     Another matrix object
         * 
         * @return A reference to this matrix
         */
        Matrix& operator=(const Matrix& m) noexcept;

        /***
         * @brief Overloaded addition operator
         *        Throws std::invalid_argument if the matrices are different sizes
         * 
         * @param m     Another matrix object
         * 
         * @return The resulting matrix
         */
        Matrix operator+(const Matrix& m) const;

        /***
         * @brief Overloaded chained addition + assignment operator
         *        Throws std::invalid_argument if the matrices are different sizes
         * 
         * @param m     Another matrix object
         */
        void operator+=(const Matrix& m);

        /***
         * @brief Overloaded subtraction operator
         *        Throws std::invalid_argument if the matrices are different sizes
         * 
         * @param m     Another matrix object
         * 
         * @return The resulting matrix
         */
        Matrix operator-(const Matrix& m) const ;

        /***
         * @brief Overloaded chained subtraction + assignment operator
         *        Throws std::invalid_argument if the matrices are different sizes
         * 
         * @param m     Another matrix object
         */
        void operator-=(const Matrix& m);

        /***
         * @brief Overloaded unary negation operator
         * 
         * @return The resulting matrix
         */
        Matrix operator-() const noexcept;

        /***
         * @brief Overloaded multiplication operator
         *        Throws std::invalid_argument if the matrices are incompatible sizes
         * 
         * @param m     Another matrix object
         * 
         * @return The resulting matrix
         */
        Matrix operator*(const Matrix& m) const;

        /***
         * @brief Overloaded multiplication operator
         * 
         * @param s     A scalar value
         * 
         * @return The resulting matrix
         */
        Matrix operator*(T s) const noexcept;

        /***
         * @brief Overloaded chained multiplication + assignment operator
         *        Throws std::invalid_argument if the matrices are incompatible sizes
         * 
         * @param m     Another matrix object
         */
        void operator*=(const Matrix& m);

        /***
         * @brief Overloaded chained multiplication + assignment operator
         * 
         * @param s     A scalar value
         */
        void operator*=(T s) noexcept;

        /***
         * @brief Overloaded division operator
         * 
         * @param s     A scalar value
         * 
         * @return The resulting matrix
         */
        Matrix operator/(T s) const noexcept;

        /***
         * @brief Overloaded chained division + assignment operator
         * 
         * @param s     A scalar value
         */
        void operator/=(T s) noexcept;

        /***
         * @brief Get the number of rows in this matrix
         * 
         * @return The number of rows in this matrix
         */
        constexpr size_t rows() const noexcept { return _rows; }

        /***
         * @brief Get the number of columns in this matrix
         * 
         * @return The number of columns in this matrix
         */
        constexpr size_t cols() const noexcept { return _cols; }

        /***
         * @brief Determine if this matrix is a row vector
         * 
         * @return Whether this matrix is a row vector
         */
        constexpr bool isRowVec() const noexcept { return _rows == 1; }

        /***
         * @brief Determine if this matrix is a column vector
         * 
         * @return Whether this matrix is a column vector
         */
        constexpr bool isColVec() const noexcept { return _cols == 1; }

        /***
         * @brief Determine if this matrix is a row/column vector
         * 
         * @return Whether this matrix is a row/column vector
         */
        constexpr bool isVec() const noexcept { return isRowVec() || isColVec(); }

        /***
         * @brief Get an iterator for the beginning of the matrix elements
         * 
         * @return Beginning iterator for the matrix elements
         */
        auto begin() noexcept { return _data.begin(); }

        /***
         * @brief Get an iterator for the end of the matrix elements
         * 
         * @return End iterator for the matrix elements
         */
        auto end() noexcept { return _data.end(); }

        /***
         * @brief Get an iterator for the beginning of the matrix elements
         * 
         * @return Beginning iterator for the matrix elements
         */
        auto begin() const noexcept { return _data.begin(); }

        /***
         * @brief Get an iterator for the end of the matrix elements
         * 
         * @return End iterator for the matrix elements
         */
        auto end() const noexcept { return _data.end(); }

        /***
         * @brief Overloaded insertion into an output stream into MATLAB matrix format
         * 
         * @param o     A reference to an output stream
         * @param m     The matrix being output to the stream
         * 
         * @return A reference to the output stream being output to
         */
        friend std::ostream& operator<<(std::ostream& o, const Matrix<T>& m) {
            o << "[";
            
            for (size_t row = 0; row < m._rows; row++) {
                for (size_t col = 0; col < m._cols; col++) {
                    o << m(row, col);

                    if (col < m._cols - 1)
                        o << ", ";
                }

                if (row < m._rows - 1)
                    o << "; ";
            }

            o << "]";

            return o;
        }
};

template <class T>
const T& Matrix<T>::_access(size_t row, size_t col) const {
    if (row >= _rows)
        throw std::out_of_range("Row out of range");
    else if (col >= _cols)
        throw std::out_of_range("Column out of range");
    return _data[col * _rows + row];
}

template <class T>
T& Matrix<T>::_access(size_t row, size_t col) {
    if (row >= _rows)
        throw std::out_of_range("Row out of range");
    else if (col >= _cols)
        throw std::out_of_range("Column out of range");
    return _data[col * _rows + row];
}

template <class T>
Matrix<T>::Matrix(size_t rows, size_t cols, const T& init_val) noexcept
    : _rows(rows), _cols(cols), _data(rows * cols, init_val) {}

template <class T>
Matrix<T>::Matrix(size_t rows, size_t cols) noexcept 
    : _rows(rows), _cols(cols), _data(rows * cols) {}

template <class T>
Matrix<T>::Matrix(size_t size, const T& init_val, MatrixVector::MatrixVectorType type) noexcept
    : _rows(type == MatrixVector::column ? size : 1), _cols(type == MatrixVector::row ? size : 1), _data(size, init_val) {}

template <class T>
Matrix<T>::Matrix(size_t size, MatrixVector::MatrixVectorType type) noexcept
    : _rows(type == MatrixVector::column ? size : 1), _cols(type == MatrixVector::row ? size : 1), _data(size) {}

template <class T>
Matrix<T>::Matrix(const std::initializer_list<std::initializer_list<T>>& values) {
    // Determine row and column size
    _cols = values.begin()->size();
    _rows = values.size();

    // Ensure all rows are the same size
    for (auto iter_r = values.begin() + 1; iter_r != values.end(); iter_r++) {
        if (_cols != iter_r->size())
            throw std::invalid_argument("Invalid initializer values");
    }

    // Resize matrix
    _data.resize(_rows * _cols);

    // Populate matrix
    size_t cur_row = 0, cur_col = 0;

    for (auto iter_r = values.begin(); iter_r != values.end(); iter_r++) {
        for (auto iter_c = iter_r->begin(); iter_c != iter_r->end(); iter_c++) {
            _access(cur_row, cur_col) = *iter_c;
            cur_col++;
        }

        cur_col = 0;
        cur_row++;
    }
}

template <class T>
Matrix<T>::Matrix(const std::initializer_list<T>& vec, MatrixVector::MatrixVectorType type) noexcept {
    // Determine row and column size
    _rows = (type == MatrixVector::column) ? vec.size() : 1;
    _cols = (type == MatrixVector::row) ? vec.size() : 1;

    // Resize matrix
    _data.resize((type == MatrixVector::column) ? _rows : _cols);

    // Populate matrix
    size_t i = 0;
    for (auto iter = vec.begin(); iter != vec.end(); iter++)
        (*this)(i++) = *iter;
}

template <class T>
Matrix<T>::Matrix(MatrixVector::MatrixVectorType type, const std::initializer_list<T>& vec) noexcept {
    // Determine row and column size
    _rows = (type == MatrixVector::column) ? vec.size() : 1;
    _cols = (type == MatrixVector::row) ? vec.size() : 1;

    // Resize matrix
    _data.resize((type == MatrixVector::column) ? _rows : _cols);

    // Populate matrix
    size_t i = 0;
    for (auto iter = vec.begin(); iter != vec.end(); iter++)
        (*this)(i++) = *iter;
}

template <class T>
Matrix<T>::Matrix(const Matrix& m) noexcept {
    _data = m._data;
    _rows = m._rows;
    _cols = m._cols;
}

template <class T>
Matrix<T>::Matrix() noexcept {
    _rows = _cols = 0;
}

template <class T>
constexpr const T& Matrix<T>::operator()(size_t row, size_t col) const {
    return _access(row, col);
}

template <class T>
constexpr const T& Matrix<T>::operator()(size_t index) const {
    if (!isVec())
        throw std::invalid_argument("Matrix isn't a vector, requires multiple indexes");
    if (index >= _data.size())
        throw std::out_of_range("Index out of range");
    return _data[index];
}

template <class T>
T& Matrix<T>::operator()(size_t row, size_t col) {
    return _access(row, col);
}

template <class T>
T& Matrix<T>::operator()(size_t index) {
    if (!isVec())
        throw std::invalid_argument("Matrix isn't a vector, requires multiple indexes");
    if (index >= _data.size())
        throw std::out_of_range("Index out of range");
    return _data[index];
}

template <class T>
constexpr bool Matrix<T>::operator==(const Matrix& m) const noexcept {
    // Sizes don't match
    if (_cols != m._cols || _rows != m._rows)
        return false;

    // Check if elements match
    return _data == m._data;
}

template <class T>
constexpr bool Matrix<T>::operator!=(const Matrix& m) const noexcept {
    return !(*this == m);
}

template <class T>
Matrix<T>& Matrix<T>::operator=(const Matrix& m) noexcept {
    _data = m._data;
    _rows = m._rows;
    _cols = m._cols;

    return *this;
}

template <class T>
Matrix<T> Matrix<T>::operator+(const Matrix& m) const {
    if (_rows != m._rows || _cols != m._cols)
        throw std::invalid_argument("Incompatible matrix sizes");

    Matrix result(_rows, _cols);

    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            result(i, j) = (*this)(i, j) + m(i, j);
        }
    }

    return result;
}

template <class T>
void Matrix<T>::operator+=(const Matrix& m) {
    if (_rows != m._rows || _cols != m._cols)
        throw std::invalid_argument("Incompatible matrix sizes");

    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            (*this)(i, j) += m(i, j);
        }
    }
}

template <class T>
Matrix<T> Matrix<T>::operator-(const Matrix& m) const {
    if (_rows != m._rows || _cols != m._cols)
        throw std::invalid_argument("Incompatible matrix sizes");

    Matrix result(_rows, _cols);

    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            result(i, j) = (*this)(i, j) - m(i, j);
        }
    }

    return result;
}

template <class T>
void Matrix<T>::operator-=(const Matrix& m) {
    if (_rows != m._rows || _cols != m._cols)
        throw std::invalid_argument("Incompatible matrix sizes");

    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            (*this)(i, j) -= m(i, j);
        }
    }
}

template <class T>
Matrix<T> Matrix<T>::operator-() const noexcept {
    Matrix result(_rows, _cols);

    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            result(i, j) = -(*this)(i, j);
        }
    }

    return result;
}

template <class T>
Matrix<T> Matrix<T>::operator*(const Matrix& m) const {
    if (_cols != m._rows)
        throw std::invalid_argument("Incompatible matrix sizes");

    Matrix result(_rows, m._cols);

    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < m._cols; j++) {
            T sum = T();
            for (size_t k = 0; k < _cols; k++)
                sum += (*this)(i, k) * m(k, j);
            result(i, j) = sum;
        }
    }

    return result;
}

template <class T>
Matrix<T> Matrix<T>::operator*(T s) const noexcept {
    Matrix result(_rows, _cols);

    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            result(i, j) = (*this)(i, j) * s;
        }
    }

    return result;
}

template <class T>
void Matrix<T>::operator*=(const Matrix& m) {
    (*this) = (*this) * m;
}

template <class T>
void Matrix<T>::operator*=(T s) noexcept {
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            (*this)(i, j) *= s;
        }
    }
}

template <class T>
Matrix<T> Matrix<T>::operator/(T s) const noexcept {
    Matrix result(_rows, _cols);

    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            result(i, j) = (*this)(i, j) / s;
        }
    }

    return result;
}

template <class T>
void Matrix<T>::operator/=(T s) noexcept {
    for (size_t i = 0; i < _rows; i++) {
        for (size_t j = 0; j < _cols; j++) {
            (*this)(i, j) /= s;
        }
    }
}