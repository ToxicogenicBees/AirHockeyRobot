#pragma once

#include <iostream>
#include <stdint.h>
#include <cmath>

template <class T>
struct Point2 {
    // The x- and y-components of the point
    T x, y;

    /***
     * @brief Creates a point with the desired x- and y-components
     * 
     * @param x     The desired x-component
     * @param y     The desired y-component
     */
    Point2(T x = T(), T y = T());

    /***
     * @brief Creates a point copying the values of another point
     * 
     * @param point The point being copied
     */
    Point2(const Point2& point);

    /***
     * @brief Overloaded assignment operator
     * 
     * @param point Another point object
     * 
     * @return A reference to this point
     */
    Point2& operator=(const Point2& point);
    
    /***
     * @brief Overloaded addition operator
     * 
     * @param point Another point object
     * 
     * @return The resulting point
     */
    Point2 operator+(const Point2& point) const;

    /***
     * @brief Overloaded chained addition + assignment operator
     * 
     * @param point Another point object
     */
    void operator+=(const Point2& point);
    
    /***
     * @brief Overloaded subtraction operator
     * 
     * @param point Another point object
     * 
     * @return The resulting point
     */
    Point2 operator-(const Point2& point) const;

    /***
     * @brief Overloaded chained subtraction + assignment operator
     * 
     * @param point Another point object
     */
    void operator-=(const Point2& point);

    /***
     * @brief Overloaded unary negation operator
     * 
     * @param point Another point object
     * 
     * @return The resulting point
     */
    Point2 operator-() const;

    /***
     * @brief Overloaded multiplication operator [Hadamard (component-wise) product]
     * 
     * @param point Another point object
     * 
     * @return The Hadamard (component-wise) product of the points
     */
    Point2 operator*(const Point2& point) const;
    
    /***
     * @brief Overloaded multiplication operator
     * 
     * @param s     A scalar value
     * 
     * @return The resulting point
     */
    Point2 operator*(T s) const;

    /***
     * @brief Overloaded chained multiplication + assignment operator [Hadamard (component-wise) product]
     * 
     * @param point Another point object
     */
    void operator*=(const Point2& point);

    /***
     * @brief Overloaded chained multiplication + assignment operator
     * 
     * @param s     A scalar value
     */
    void operator*=(T s);

    /***
     * @brief Overloaded division operator [Hadamard (component-wise) quotient]
     * 
     * @param point Another point object
     * 
     * @return The Hadamard (component-wise) quotient of the points
     */
    Point2 operator/(const Point2& point) const;
    
    /***
     * @brief Overloaded division operator
     * 
     * @param s     A scalar value
     * 
     * @return The resulting point
     */
    Point2 operator/(T s) const;

    /***
     * @brief Overloaded chained division + assignment operator [Hadamard (component-wise) quotient]
     * 
     * @param point Another point object
     */
    void operator/=(const Point2& point);

    /***
     * @brief Overloaded chained division + assignment operator
     * 
     * @param s     A scalar value
     */
    void operator/=(T s);

    /***
     * @brief Calculate the dot product between this point and another point
     * 
     * @param point Another point object
     * 
     * @return The dot product
     */
    T dot(const Point2& point) const;

    /***
     * @brief Calculate the normal of this point
     * 
     * @return The normal of this point
     */
    Point2 normal() const;

    /***
     * @brief Calculate the magnitude of this point
     * 
     * @return The magnitude of this point
     */
    T magnitude() const;

    /***
     * @brief Calculate the squared magnitude of this point
     * 
     * @return The squared magnitude of this point
     */
    T squaredMagnitude() const;

    /***
     * @brief Overloaded insertion operator
     * 
     * @param o     A reference to an output stream
     * @param point The point being output to the stream
     * 
     * @result A reference to the output stream being output to
     */
    friend std::ostream& operator<<(std::ostream& o, const Point2<T>& point) {
        o << "(" << point.x << ", " << point.y << ")";
        return o;
    }
};

template <class T>
Point2<T>::Point2(T x, T y) {
    this->x = x;
    this->y = y;
}

template <class T>
Point2<T>::Point2(const Point2<T>& point) {
    x = point.x;
    y = point.y;
}

template <class T>
Point2<T>& Point2<T>::operator=(const Point2<T>& point) {
    x = point.x;
    y = point.y;

    return *this;
}

template <class T>
Point2<T> Point2<T>::operator+(const Point2<T>& point) const {
    return Point2<T>(x + point.x, y + point.y);
}

template <class T>
void Point2<T>::operator+=(const Point2<T>& point) {
    x += point.x;
    y += point.y;
}

template <class T>
Point2<T> Point2<T>::operator-(const Point2<T>& point) const {
    return Point2(x - point.x, y - point.y);
}

template <class T>
void Point2<T>::operator-=(const Point2<T>& point) {
    x -= point.x;
    y -= point.y;
}

template <class T>
Point2<T> Point2<T>::operator-() const {
    return Point2(-x, -y);
}

template <class T>
Point2<T> Point2<T>::operator*(const Point2& point) const {
    return Point2(x * point.x, y * point.y);
}

template <class T>
Point2<T> operator*(T s, const Point2<T>& point) {
    return point * s;
}

template <class T>
Point2<T> Point2<T>::operator*(T s) const {
    return Point2(x * s, y * s);
}

template <class T>
void Point2<T>::operator*=(const Point2& point) {
    x *= point.x;
    y *= point.y;
}

template <class T>
void Point2<T>::operator*=(T s) {
    x *= s;
    y *= s;
}

template <class T>
Point2<T> Point2<T>::operator/(const Point2& point) const {
    return Point2(x / point.x, y / point.y);
}

template <class T>
Point2<T> operator/(T s, const Point2<T>& point) {
    return point / s;
}

template <class T>
Point2<T> Point2<T>::operator/(T s) const {
    return Point2(x / s, y / s);
}

template <class T>
void Point2<T>::operator/=(const Point2& point) {
    x /= point.x;
    y /= point.y;
}

template <class T>
void Point2<T>::operator/=(T s) {
    x /= s;
    y /= s;
}

template <class T>
T Point2<T>::dot(const Point2<T>& point) const {
    return (x * point.x) + (y * point.y);
}

template <class T>
Point2<T> Point2<T>::normal() const {
    double mag = magnitude();
    return (mag != 0 ? (*this / mag) : *this);
}

template <class T>
T Point2<T>::magnitude() const {
    return std::sqrt(dot(*this));
}

template <class T>
T Point2<T>::squaredMagnitude() const {
    return dot(*this);
}