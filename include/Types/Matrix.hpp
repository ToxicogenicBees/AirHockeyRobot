#pragma once

#include <stdexcept>
#include <iostream>
#include <stdint.h>
#include <vector>

template <class T>
class Matrix;

using dMatrix = Matrix<double>;

template <class T>
class Matrix {
    private:
        uint16_t _rows, _cols;      // Matrix rows and columns
        std::vector<T> _data;       // Matrix entries

        // Access a constant matrix element
        const T& _access(uint16_t row, uint16_t col) const;

        // Access a matrix element
        T& _access(uint16_t row, uint16_t col);

    public:
        /***
         * @brief Creates a matrix with the desired rows and columns, populated with an initial value
         * 
         * @param rows      The number of rows in the matrix
         * @param cols      The number of columns in the matrix
         * @param init_val  The initial value populating the matrix
         */
        Matrix(uint16_t rows, uint16_t cols, const T& init_val);

        /***
         * @brief Creates a matrix with the desired rows and columns
         * 
         * @param rows      The number of rows in the matrix
         * @param cols      The number of columns in the matrix
         */
        Matrix(uint16_t rows, uint16_t cols);

        /***
         * @brief Creates a matrix copying the values of another matrix
         * 
         * @param m     The matrix being copied
         */
        Matrix(const Matrix& m);

        /***
         * @brief Overloaded function operator, accessing an element of a constant 2D matrix
         *        Throws std::out_of_range if the provided indexes are invalid
         * 
         * @param row   The desired row
         * @param col   The desired column
         * 
         * @return Constant reference to the desired item
         */
        const T& operator()(uint16_t row, uint16_t col) const;

        /***
         * @brief Overloaded function operator, accessing an element of a constant column/row vector
         *        Throws std::out_of_range if the provided indexes are invalid
         *        Throws std::invalid_argument if the matrix is not a column/row vector
         * 
         * @param row   The desired element in the column/row vector
         * 
         * @return Constant reference to the desired item
         */
        const T& operator()(uint16_t index) const;

        /***
         * @brief Overloaded function operator, accessing an element of a 2D matrix
         *        Throws std::out_of_range if the provided indexes are invalid
         * 
         * @param row   The desired row
         * @param col   The desired column
         * 
         * @return Reference to the desired item
         */
        T& operator()(uint16_t row, uint16_t col);

        /***
         * @brief Overloaded function operator, accessing an element of a column/row vector
         *        Throws std::out_of_range if the provided indexes are invalid
         *        Throws std::invalid_argument if the matrix is not a column/row vector
         * 
         * @param row   The desired element in the column/row vector
         * 
         * @return Reference to the desired item
         */
        T& operator()(uint16_t index);

        /***
         * @brief Overloaded comparison operator
         * 
         * @param m     Another matrix object
         * 
         * @return A reference to this matrix
         */
        bool operator==(const Matrix& m);

        /***
         * @brief Overloaded assignment operator
         * 
         * @param m     Another matrix object
         * 
         * @return A reference to this matrix
         */
        Matrix& operator=(const Matrix& m);

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
        Matrix operator-(const Matrix& m) const;

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
         * @param m     Another matrix object
         * 
         * @return The resulting matrix
         */
        Matrix operator-() const;

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
        uint16_t rows() const { return _rows; }

        /***
         * @brief Get the number of columns in this matrix
         * 
         * @return The number of columns in this matrix
         */
        uint16_t cols() const { return _cols; }

        /***
         * @brief Overloaded insertion into an output stream into MATLAB matrix format
         * 
         * @param o     A reference to an output stream
         * @param m     The matrix being output to the stream
         * 
         * @result A reference to the output stream being output to
         */
        friend std::ostream& operator<<(std::ostream& o, const Matrix<T>& m) {
            o << "[";
            
            for (uint16_t row = 0; row < m._rows; row++) {
                for (uint16_t col = 0; col < m._cols; col++) {
                    o << m(row, col);

                    if (col == m._cols - 1 && row == m._rows - 1)
                        o << "]";
                    else if (col == m._cols - 1)
                        o << "; ";
                    else
                        o << ", ";
                }
            }

            return o;
        }
};

template <class T>
const T& Matrix<T>::_access(uint16_t row, uint16_t col) const {
    if (row >= _rows || row < 0)
        throw std::out_of_range("Row out of range");
    else if (col >= _cols || col < 0)
        throw std::out_of_range("Column out of range");
    return _data[col * _rows + row];
}


template <class T>
T& Matrix<T>::_access(uint16_t row, uint16_t col) {
    if (row >= _rows || row < 0)
        throw std::out_of_range("Row out of range");
    else if (col >= _cols || col < 0)
        throw std::out_of_range("Column out of range");
    return _data[col * _rows + row];
}

template <class T>
Matrix<T>::Matrix(uint16_t rows, uint16_t cols, const T& init_val) {
    _data.resize(rows * cols);
    
    _rows = rows;
    _cols = cols;

    for (uint16_t i = 0; i < _rows; i++) {
        for (uint16_t j = 0; j < _cols; j++) {
            _access(i, j) = T(init_val);
        }
    }
}

template <class T>
Matrix<T>::Matrix(uint16_t rows, uint16_t cols) {
    _data.resize(rows * cols);
    _rows = rows;
    _cols = cols;
}

template <class T>
Matrix<T>::Matrix(const Matrix& m) {
    _data = m._data;
    _rows = m._rows;
    _cols = m._cols;
}

template <class T>
const T& Matrix<T>::operator()(uint16_t row, uint16_t col) const {
    return _access(row, col);
}

template <class T>
const T& Matrix<T>::operator()(uint16_t index) const {
    if (_rows > 1 && _cols > 1)
        throw std::invalid_argument("Matrix isn't a vector, requires multiple indexes");
    if (index >= (_rows > _cols ? _rows : _cols))
        throw std::out_of_range("Index out of range");
    return (_rows == 1 ? _access(0, index) : _access(index, 0));
}

template <class T>
T& Matrix<T>::operator()(uint16_t row, uint16_t col) {
    return _access(row, col);
}

template <class T>
T& Matrix<T>::operator()(uint16_t index) {
    if (_rows > 1 && _cols > 1)
        throw std::invalid_argument("Matrix isn't a vector, requires multiple indexes");
    if (index >= (_rows > _cols ? _rows : _cols))
        throw std::out_of_range("Index out of range");
    return (_rows == 1 ? _access(0, index) : _access(index, 0));
}

template <class T>
bool Matrix<T>::operator==(const Matrix& m) {
    // Sizes don't match
    if (_cols != m._cols || _rows != m._cols)
        return false;

    // Elements don't match
    for (uint16_t i = 0; i < _rows; i++) {
        for (uint16_t j = 0; j < _cols; j++) {
            if (_access(i, j) != m(i, j))
                return false;
        }
    }

    // Matrices are the same
    return true;
}

template <class T>
Matrix<T>& Matrix<T>::operator=(const Matrix& m) {
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

    for (uint16_t i = 0; i < _rows; i++) {
        for (uint16_t j = 0; j < _cols; j++) {
            result(i, j) = (i, j) + m(i, j);
        }
    }

    return result;
}

template <class T>
void Matrix<T>::operator+=(const Matrix& m) {
    if (_rows != m._rows || _cols != m._cols)
        throw std::invalid_argument("Incompatible matrix sizes");

    for (uint16_t i = 0; i < _rows; i++) {
        for (uint16_t j = 0; j < _cols; j++) {
            (*this)(i, j) += m(i, j);
        }
    }
}

template <class T>
Matrix<T> Matrix<T>::operator-(const Matrix& m) const {
    if (_rows != m._rows || _cols != m._cols)
        throw std::invalid_argument("Incompatible matrix sizes");

    Matrix result(_rows, _cols);

    for (uint16_t i = 0; i < _rows; i++) {
        for (uint16_t j = 0; j < _cols; j++) {
            result(i, j) = (*this)(i, j) - m(i, j);
        }
    }

    return result;
}

template <class T>
void Matrix<T>::operator-=(const Matrix& m) {
    if (_rows != m._rows || _cols != m._cols)
        throw std::invalid_argument("Incompatible matrix sizes");

    for (uint16_t i = 0; i < _rows; i++) {
        for (uint16_t j = 0; j < _cols; j++) {
            (*this)(i, j) -= m(i, j);
        }
    }
}

template <class T>
Matrix<T> Matrix<T>::operator-() const {
    Matrix result(_rows, _cols);

    for (uint16_t i = 0; i < _rows; i++) {
        for (uint16_t j = 0; j < _cols; j++) {
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

    for (uint16_t i = 0; i < _rows; i++) {
        for (uint16_t j = 0; j < m._cols; j++) {
            T sum = T();
            for (uint16_t k = 0; k < _cols; k++)
                sum += (*this)(i, k) * m(k, j);
            result(i, j) = sum;
        }
    }

    return result;
}

template <class T>
Matrix<T> Matrix<T>::operator*(T s) const noexcept {
    Matrix result(_rows, _cols);

    for (uint16_t i = 0; i < _rows; i++) {
        for (uint16_t j = 0; j < _cols; j++) {
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
    for (uint16_t i = 0; i < _rows; i++) {
        for (uint16_t j = 0; j < _cols; j++) {
            (*this)(i, j) *= s;
        }
    }
}

template <class T>
Matrix<T> Matrix<T>::operator/(T s) const noexcept {
    Matrix result(_rows, _cols);

    for (uint16_t i = 0; i < _rows; i++) {
        for (uint16_t j = 0; j < _cols; j++) {
            result(i, j) = (*this)(i, j) / s;
        }
    }

    return result;
}

template <class T>
void Matrix<T>::operator/=(T s) noexcept {
    for (uint16_t i = 0; i < _rows; i++) {
        for (uint16_t j = 0; j < _cols; j++) {
            (*this)(i, j) /= s;
        }
    }
}
