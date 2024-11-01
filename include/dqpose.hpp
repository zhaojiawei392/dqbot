/** 
 *     This file is part of dqbot.
 *  
 *     dqbot is free software: you can redistribute it and/or modify 
 *     it under the terms of the GNU General Public License as published 
 *     by the Free Software Foundation, either version 3 of the License, 
 *     or (at your option) any later version.
 *  
 *     dqbot is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *     See the GNU General Public License for more details.
 *  
 *     You should have received a copy of the GNU General Public License
 *     along with dqbot. If not, see <https://www.gnu.org/licenses/>.
 */

/**
 *     \file include/dqbot/quat.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2024-2025
 *
 *     \brief A header file defining Quaternion operations
 * 
 *     This file provides the necessary classes and functions to represent 
 *     and manipulate Quaternions.
 * 
 *     \cite https://github.com/zhaojiawei392/dqbot.git
 */

#pragma once
#include <iostream>
#include <iomanip>
#include <cmath>
#include <array>
#include <cstdint>

namespace dqbot
{


constexpr int PRINT_PRECISION = 12;

// Forward declarations
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class Quat;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class Translation;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class Rotation;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class UnitAxis;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class DualQuat;
template<typename qScalar, typename = std::enable_if_t<std::is_arithmetic_v<qScalar>>>
class Pose;

// T square
template <typename T>
constexpr inline T square(const T& x) {
    return x * x;
}
// T operator<<
template<typename Scalar>
inline std::ostream& operator<<(std::ostream& os, const Quat<Scalar>& quat) {
    os << quat.to_string();  
    return os;
}
// T operator<<
template<typename Scalar>
inline std::ostream& operator<<(std::ostream& os, const DualQuat<Scalar>& dq) {
    os << dq.to_string();  
    return os;
}
// T T operator*
template<typename Scalar1, typename Scalar2> 
inline std::enable_if_t<std::is_arithmetic_v<Scalar1>, Quat<Scalar2>>
operator*(const Scalar1 scalar, const Quat<Scalar2>& quat) noexcept {return quat * scalar;}
// T T operator*
template<typename Scalar1, typename Scalar2>
inline std::enable_if_t<std::is_arithmetic_v<Scalar1>, DualQuat<Scalar2>>
operator*(const Scalar1 scalar, const DualQuat<Scalar2>& dq) noexcept {return dq * scalar;}
// T T operator*
template<typename Scalar1, typename Scalar2>
inline DualQuat<Scalar1> operator*(const Quat<Scalar1>& quat, const DualQuat<Scalar2>& dq) noexcept {
    return DualQuat<Scalar1>( quat * dq.real(), quat * dq.dual() );
}

template<typename qScalar, typename>
class Quat {
public:
protected:
    std::array<qScalar, 4> _data;
    constexpr inline qScalar& _w() noexcept {return _data[0];};
    constexpr inline qScalar& _x() noexcept {return _data[1];};
    constexpr inline qScalar& _y() noexcept {return _data[2];};
    constexpr inline qScalar& _z() noexcept {return _data[3];};
public:
    // Default Constructor
    constexpr explicit Quat() noexcept
    : _data{ 0, 0, 0, 0 } {

    }
    // Array Constructor
    constexpr explicit Quat(const std::array<qScalar, 4>& arr4) noexcept
    : _data( arr4 ) {

    }
    // Scalar Constructor
    constexpr explicit Quat(const qScalar w, const qScalar x=0, const qScalar y=0, const qScalar z=0) noexcept
    : _data{ w, x, y, z } {

    }
    // T Copy Constructors 
    template<typename Scalar>
    constexpr Quat(const Quat<Scalar>& other) noexcept
    : _data{ static_cast<qScalar>(other.w()), static_cast<qScalar>(other.x()), static_cast<qScalar>(other.y()), static_cast<qScalar>(other.z()) } {

    }
    // T Copy Assignment
    template<typename Scalar>
    constexpr inline Quat& operator=(const Quat<Scalar>& other) noexcept {
        _w() = static_cast<qScalar>(other.w());
        _x() = static_cast<qScalar>(other.x());
        _y() = static_cast<qScalar>(other.y());
        _z() = static_cast<qScalar>(other.z());
        return *this;
    }
    // normalize
    constexpr inline Quat& normalize() {
        const qScalar this_norm = this->norm();
        if (this_norm == 0) {
            throw std::runtime_error("Error: Quat& Quat::normalize() Cannot normalize a 0-norm Quaternion.");
        } else if (this_norm != 1){
            _w() /= this_norm;
            _x() /= this_norm;
            _y() /= this_norm;
            _z() /= this_norm;
        }
        return *this;
    }
    // purify
    constexpr inline Quat& purify() noexcept {
        _w() = 0;
        return *this;
    }
    // operator+
    template<typename Scalar>
    constexpr inline Quat operator+(const Quat<Scalar>& other) const noexcept {
        const qScalar result_w = w() + static_cast<qScalar>(other.w());
        const qScalar result_x = x() + static_cast<qScalar>(other.x()); 
        const qScalar result_y = y() + static_cast<qScalar>(other.y()); 
        const qScalar result_z = z() + static_cast<qScalar>(other.z()); 
        return Quat(result_w, result_x, result_y, result_z);
    }
    // operator-
    template<typename Scalar>
    constexpr inline Quat operator-(const Quat<Scalar>& other) const noexcept {
        const qScalar result_w = w() - static_cast<qScalar>(other.w());
        const qScalar result_x = x() - static_cast<qScalar>(other.x());
        const qScalar result_y = y() - static_cast<qScalar>(other.y());
        const qScalar result_z = z() - static_cast<qScalar>(other.z());
        return Quat(result_w, result_x, result_y, result_z);
    }
    // operator*
    template<typename Scalar>
    constexpr inline Quat operator*(const Quat<Scalar>& other) const noexcept {
        const qScalar other_w = static_cast<qScalar>(other.w());
        const qScalar other_x = static_cast<qScalar>(other.x());
        const qScalar other_y = static_cast<qScalar>(other.y());
        const qScalar other_z = static_cast<qScalar>(other.z());
        const qScalar result_w = w()*other_w - x()*other_x - y()*other_y - z()*other_z;  
        const qScalar result_x = x()*other_w + w()*other_x - z()*other_y + y()*other_z; 
        const qScalar result_y = y()*other_w + z()*other_x + w()*other_y - x()*other_z; 
        const qScalar result_z = z()*other_w - y()*other_x + x()*other_y + w()*other_z; 
        return Quat(result_w, result_x, result_y, result_z);
    }
    // operator*
    constexpr inline Quat operator*(const qScalar scalar) const noexcept {
        const qScalar result_w = w() * scalar;  
        const qScalar result_x = x() * scalar; 
        const qScalar result_y = y() * scalar; 
        const qScalar result_z = z() * scalar; 
        return Quat(result_w, result_x, result_y, result_z);
    }
    // -operator
    constexpr inline Quat operator-() const noexcept { return Quat(-w(), -x(), -y(), -z()); }
    // operator==
    template<typename Scalar>
    constexpr inline bool operator==(const Quat<Scalar>& other) const noexcept { return _data == other._data; }
    // operator!=
    template<typename Scalar>
    constexpr inline bool operator!=(const Quat<Scalar>& other) const noexcept { return _data != other._data; }
    // dot
    template<typename Scalar>
    constexpr inline qScalar dot(const Quat<Scalar>& other) const noexcept {
        return    w() * static_cast<qScalar>(other.w()) 
                + x() * static_cast<qScalar>(other.x()) 
                + y() * static_cast<qScalar>(other.y()) 
                + z() * static_cast<qScalar>(other.z());
    }
    // norm
    constexpr inline qScalar norm() const noexcept {
        return std::sqrt( square( w() ) + square( x() ) + square( y() ) + square( z() ));
    }
    // norm3
    constexpr inline qScalar norm3() const noexcept {
        return std::sqrt( square( x() ) + square( y() ) + square( z() ));
    }
    // copied
    constexpr inline Quat copied() const noexcept {
        return *this;
    }
    // normalized
    constexpr inline Quat normalized() const {
        return copied().normalize(); 
    }
    // purified
    constexpr inline Quat purified() const noexcept {
        return copied().purify(); 
    }
    // conj
    constexpr inline Quat conj() const noexcept {
        const qScalar result_w = w();  
        const qScalar result_x = - x(); 
        const qScalar result_y = - y(); 
        const qScalar result_z = - z(); 
        return Quat(result_w, result_x, result_y, result_z);  
    }
    // inv
    constexpr inline Quat inv() const noexcept {
        const qScalar norm2 = square(norm());
        const qScalar result_w = w() / norm2;  
        const qScalar result_x = - x() / norm2; 
        const qScalar result_y = - y() / norm2; 
        const qScalar result_z = - z() / norm2; 
        return Quat(result_w, result_x, result_y, result_z);  
    }
    // log
    constexpr inline Quat log() const noexcept {
        const qScalar vec3_norm = norm3();
        if (vec3_norm == 0) {
            return Quat(std::log(w()));
        }
        const qScalar this_norm = norm();
        const qScalar this_theta = acos(w() / this_norm);
        const qScalar result_w = std::log(this_norm);
        const qScalar result_x = this_theta * x() / vec3_norm;
        const qScalar result_y = this_theta * y() / vec3_norm;
        const qScalar result_z = this_theta * z() / vec3_norm;
        return Quat(result_w, result_x, result_y, result_z);  
    }
    // exp
    constexpr inline Quat exp() const noexcept {        
        const qScalar vec3_norm = norm3();
        const qScalar exp_w = std::exp(w());        
        if (vec3_norm == 0) {
            return Quat(exp_w);
        }
        const qScalar cos_norm3 = cos(vec3_norm);
        const qScalar sin_norm3 = sin(vec3_norm);
        const qScalar result_w = exp_w * cos_norm3;
        const qScalar result_x = exp_w * sin_norm3 * x() / vec3_norm;
        const qScalar result_y = exp_w * sin_norm3 * y() / vec3_norm;
        const qScalar result_z = exp_w * sin_norm3 * z() / vec3_norm;
        return Quat(result_w, result_x, result_y, result_z); 
    }
    // pow
    constexpr inline Quat pow(const qScalar index) const noexcept{
        return (this->log() * index).exp();
    }
    // hamiplus
    constexpr inline std::array<std::array<qScalar, 4>, 4> hamiplus() const noexcept {
        return std::array<std::array<qScalar, 4>, 4> { { w(), -x(), -y(), -z() },
                                                    { x(),  w(), -z(),  y() },
                                                    { y(),  z(),  w(), -x() },
                                                    { z(), -y(),  x(),  w() } };
    }
    // haminus
    constexpr inline std::array<std::array<qScalar, 4>, 4> haminus() const noexcept {
        return std::array<std::array<qScalar, 4>, 4> { { w(), -x(), -y(), -z() },
                                                    { x(),  w(),  z(), -y() },
                                                    { y(), -z(),  w(),  x() },
                                                    { z(),  y(), -x(),  w() } };            
    }
    // Query const
    constexpr inline qScalar w() const noexcept {return _data[0];}
    constexpr inline qScalar x() const noexcept {return _data[1];}
    constexpr inline qScalar y() const noexcept {return _data[2];}
    constexpr inline qScalar z() const noexcept {return _data[3];}
    // to_string
    constexpr inline std::string to_string() const {    
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION) << w() << " + " << x() << " î + " << y() << " ĵ + " << z() << " k̂";
        return oss.str();
    };
    // data
    constexpr inline const qScalar* data3() const noexcept { return _data.data()+1; }
    constexpr inline const qScalar* data4() const noexcept { return _data.data(); }
    constexpr inline std::array<qScalar, 3> array3() const noexcept { return std::array<qScalar, 3>{x(), y(), z()}; }
    constexpr inline std::array<qScalar, 4> array4() const noexcept { return _data; }
    constexpr inline std::array<qScalar, 4> vrep_array4() const noexcept { return std::array<qScalar, 4>{x(), y(), z(), w()}; }
    // Defaults
    virtual ~Quat()=default;    
            Quat(const Quat&)=default;
            Quat(Quat&&)=default;
    Quat& operator=(const Quat&)=default;
    Quat& operator=(Quat&&)=default;
};

template<typename qScalar, typename>
class Translation: public Quat<qScalar>
{ 
public:
    // Default Constructor
    constexpr explicit Translation() noexcept
    : Quat<qScalar>( ) {

    }
    // Array Constructor
    constexpr explicit Translation(const std::array<qScalar, 3> arr3) noexcept
    : Quat<qScalar>( 0, arr3[0], arr3[1], arr3[2] ) {

    }
    // Scalar Constructor
    constexpr explicit Translation(const qScalar x, const qScalar y=0, const qScalar z=0) noexcept
    : Quat<qScalar>( 0, x, y, z ) {

    }
    // Quat Constructor
    template<typename Scalar>
    constexpr Translation(const Quat<Scalar>& other) noexcept
    : Quat<qScalar>( 0, other.x(), other.y(), other.z() ) {

    }
    // Quat Assignment
    template<typename Scalar>
    constexpr inline Translation& operator=(const Quat<Scalar>& other) noexcept {
        this->_w() = 0;
        this->_x() = static_cast<qScalar>(other.x());
        this->_y() = static_cast<qScalar>(other.y());
        this->_z() = static_cast<qScalar>(other.z());
        return *this;
    }
    // active_rotate 
    template<typename Scalar>
    constexpr inline Translation& active_rotate(const Rotation<Scalar>& rotation) noexcept {
        this->operator=(rotation * *this * rotation.conj());
        return *this;
    }
    // passive_rotate 
    template<typename Scalar>
    constexpr inline Translation& passive_rotate(const Rotation<Scalar>& rotation) noexcept {
        this->operator=(rotation.conj() * *this * rotation);
        return *this;
    }
    // active_rotated
    template<typename Scalar>
    constexpr inline Translation active_rotated(const Rotation<Scalar>& rotation) const noexcept {
        return Translation(rotation * *this * rotation.conj());
    }    
    // passive_rotated
    template<typename Scalar>
    constexpr inline Translation passive_rotated(const Rotation<Scalar>& rotation) const noexcept {
        return Translation(rotation.conj() * *this * rotation);
    }
    // perpendicular
    template<typename Scalar>
    constexpr inline UnitAxis<qScalar> perpendicular(const Translation<Scalar>& other) const noexcept {
        const qScalar axis_x = this->y() * other.z() - this->z() * other.y();
        const qScalar axis_y = this->z() * other.x() - this->x() * other.z();
        const qScalar axis_z = this->x() * other.y() - this->y() * other.x();
        return UnitAxis(axis_x, axis_y, axis_z);
    }
    // angle
    template<typename Scalar>
    constexpr inline qScalar angle(const Translation<Scalar>& other) const noexcept {
        return std::acos(this->normalized().dot(other.normalized()));
    }
    // Defaults
        virtual ~Translation()=default;
                Translation(const Translation&)=default;
                Translation(Translation&&)=default;
    Translation& operator=(const Translation&)=default;
    Translation& operator=(Translation&&)=default;
};

template<typename qScalar, typename>
class Rotation : public Quat<qScalar>
{
public:
    // Default Constructor 
    constexpr explicit Rotation() noexcept
    : Quat<qScalar>( 1 ) {
    
    }
    // Array Constructor
    constexpr explicit Rotation(const std::array<qScalar, 4>& arr4) noexcept
    : Quat<qScalar>( arr4 ) {
        this->normalize();
    }
    // Scalar Constructor 
    constexpr explicit Rotation(const qScalar w, const qScalar x=0, const qScalar y=0, const qScalar z=0) noexcept
    : Quat<qScalar>( w, x, y, z ) {
        this->normalize();
    }
    // Quat Constructor
    template<typename Scalar>
    constexpr Rotation(const Quat<Scalar>& other) noexcept
    : Quat<qScalar>(other) {
        this->normalize();
    }
    // Quat Assignment 
    template<typename Scalar>
    constexpr inline Rotation& operator=(const Quat<Scalar>& other) noexcept {
        this->_w() = static_cast<qScalar>(other.w());
        this->_x() = static_cast<qScalar>(other.x());
        this->_y() = static_cast<qScalar>(other.y());
        this->_z() = static_cast<qScalar>(other.z());
        this->normalize();
        return *this;
    }    
    // Axis-Angle Constructor 
    template<typename Scalar>
    constexpr explicit Rotation(const UnitAxis<Scalar>& rotate_axis, const qScalar rotate_angle) noexcept
    : Quat<qScalar>(1) {
        this->_w() = cos(0.5 * rotate_angle);
        const qScalar sin_ = sin(0.5 * rotate_angle);
        this->_x() = rotate_axis.x() * sin_;
        this->_y() = rotate_axis.y() * sin_;
        this->_z() = rotate_axis.z() * sin_;
    }
    // rotation_axis
    constexpr inline UnitAxis<qScalar> rotation_axis() const noexcept {
        const qScalar vec3_norm = this->norm3();
        if (vec3_norm == 0){
            return UnitAxis<qScalar>(0,0,1);
        }
        const qScalar result_x = this->x() / vec3_norm;
        const qScalar result_y = this->y() / vec3_norm;
        const qScalar result_z = this->z() / vec3_norm;
        return UnitAxis<qScalar>(result_x, result_y, result_z);
    }
    // rotation_angle
    constexpr inline qScalar rotation_angle() const noexcept {
        return 2 * acos(this->w() / this->norm());
    }
    // Defaults
        virtual ~Rotation()=default;
                Rotation(const Rotation&)=default;
                Rotation(Rotation&&)=default;
    Rotation& operator=(const Rotation&)=default;
    Rotation& operator=(Rotation&&)=default;
};

template<typename qScalar, typename>
class UnitAxis : public Quat<qScalar>
{
public:
    // Default Constructor
    constexpr explicit UnitAxis() noexcept 
        : Quat<qScalar>( 0, 1, 0, 0 ) {

        }
    // Array Constructor
    constexpr explicit UnitAxis(const std::array<qScalar, 3> arr3) noexcept
        : Quat<qScalar>( 0, arr3[0], arr3[1], arr3[2] ) {
        this->normalize();
        }
    // Scalar Constructor
    constexpr explicit UnitAxis(const qScalar x, const qScalar y=0, const qScalar z=0) noexcept
        : Quat<qScalar>( 0, x, y, z ) {
        this->normalize();
        }
    // Quat Constructor
    template<typename Scalar>
    constexpr UnitAxis(const Quat<Scalar>& other) noexcept
        : Quat<qScalar>( other ) {
        this->_w() = 0;
        this->normalize();
        }
    // Quat Assignment
    template<typename Scalar>
    constexpr inline UnitAxis& operator=(const Quat<Scalar>& other) noexcept {
        this->_w() = 0;
        this->_x() = static_cast<qScalar>(other.x());
        this->_y() = static_cast<qScalar>(other.y());
        this->_z() = static_cast<qScalar>(other.z());
        this->normalize();
        return *this;
    } 
    // Default
            virtual ~UnitAxis()=default;
                    UnitAxis(const UnitAxis&)=default;
                    UnitAxis(UnitAxis&&)=default;
    UnitAxis& operator=(const UnitAxis&)=default;
    UnitAxis& operator=(UnitAxis&&)=default;
};

template<typename qScalar, typename>
class DualQuat{
protected:
    std::array<Quat<qScalar>, 2> _data;
    constexpr inline Quat<qScalar>& _real() {return _data[0];}
    constexpr inline Quat<qScalar>& _dual() {return _data[1];}
public:
    // Default Constructor
    constexpr explicit DualQuat() noexcept
    : _data{ Quat<qScalar>(), Quat<qScalar>() } {

    }
    // Array Constructor
    constexpr explicit DualQuat(const std::array<qScalar, 8> arr8) noexcept
    : _data{ Quat<qScalar>(arr8[0], arr8[1], arr8[2], arr8[3]), Quat<qScalar>(arr8[4], arr8[5], arr8[6], arr8[7]) } {

    }
    // Scalar Constructor
    constexpr explicit DualQuat(const qScalar w1, const qScalar x1=0, const qScalar y1=0, const qScalar z1=0, 
                      const qScalar w2=0, const qScalar x2=0, const qScalar y2=0, const qScalar z2=0) noexcept
    : _data{ Quat<qScalar>(w1, x1, y1, z1), Quat<qScalar>(w2, x2, y2, z2) } {

    }
    // Real Constructor
    template<typename Scalar>
    constexpr explicit DualQuat(const Quat<Scalar>& real) noexcept
    : _data{ Quat<qScalar>(real), Quat<qScalar>() } {

    }
    // Real-Dual Constructor
    template<typename Scalar1, typename Scalar2>
    constexpr explicit DualQuat(const Quat<Scalar1>& real, const Quat<Scalar2>& dual) noexcept
    : _data{ Quat<qScalar>(real), Quat<qScalar>(dual) } {

    }
    // Copy Constructor
    template<typename Scalar>
    constexpr DualQuat(const DualQuat<Scalar>& other) noexcept
    : _data{ Quat<qScalar>(other.real()), Quat<qScalar>(other.dual()) } {

    }
    // Copy Assignment
    template<typename Scalar>
    constexpr inline DualQuat& operator=(const DualQuat<Scalar>& other) noexcept {
        _real() = other.real();
        _dual() = other.dual();
        return *this;
    }
    // normalize
    constexpr inline DualQuat& normalize() {
        const qScalar this_norm = real().norm();
        if (this_norm == 0) {
            throw std::runtime_error("Error: DualQuat& normalize() Cannot normalize a 0 Dual Quaternion.");
        } else if (this_norm != 1){
            _real() = real() * ( 1 / this_norm );
            _dual() = dual() * ( 1 / this_norm );
        }
        return *this;
    }
    // purifiy
    constexpr inline DualQuat& purify() noexcept {
        real().purify();
        dual().purify();
        return *this;
    }
    // operator+    
    template<typename Scalar>
    constexpr inline DualQuat operator+(const DualQuat<Scalar>& other) const noexcept {
        return DualQuat( real() + other.real(), dual() + other.dual() );
    }
    // operator-
    template<typename Scalar>
    constexpr inline DualQuat operator-(const DualQuat<Scalar>& other) const noexcept {
        return DualQuat( real() - other.real(), dual() - other.dual() );
    }
    // operator*  
    template<typename Scalar>
    constexpr inline DualQuat operator*(const DualQuat<Scalar>& other) const noexcept {
        const Quat<qScalar>& result_dual = real() * other.dual() + dual() * other.real();
        const Quat<qScalar>& result_real = real() * other.real();
        return DualQuat( result_real, result_dual );
    }
    // operator*  
    template<typename Scalar>
    constexpr inline DualQuat operator*(const Quat<Scalar>& quat) const noexcept {
        return DualQuat( real() * quat, dual() * quat );
    }
    // operator*  
    constexpr inline DualQuat operator*(const qScalar scalar) const noexcept {
        return DualQuat( real() * scalar, dual() * scalar );
    }
    // -operator  
    constexpr inline DualQuat operator-() const noexcept {
        return DualQuat( -real(), -dual() );
    }
    // operator== 
    constexpr inline bool operator==(const DualQuat& other) const noexcept {
        return real() == other.real() && dual() == other.dual();
    }
    // operator!= 
    constexpr inline bool operator!=(const DualQuat& other) const noexcept {
        return real() != other.real() || dual() != other.dual();
    }
    // norm
    constexpr inline DualQuat norm() const noexcept {
        const qScalar result_realnorm = real().norm();
        if (result_realnorm == 0) 
            return DualQuat(0);
        const qScalar res_result_dualnorm = real().dot(dual()) / result_realnorm;
        return DualQuat(Quat<qScalar>(result_realnorm), Quat<qScalar>(res_result_dualnorm));
    }
    // copied
    constexpr inline DualQuat copied() const noexcept {
        return *this;
    }
    // normalized
    constexpr inline DualQuat normalized() const {
        return copied().normalize();
    }
    // purified
    constexpr inline DualQuat purified() const noexcept {
        return copied().purify();
    }
    // conj
    constexpr inline DualQuat conj() const noexcept {
        return DualQuat(real().conj(), dual().conj());
    }
    // inv
    constexpr inline DualQuat inv() const noexcept {
        const Quat<qScalar>& result_real = real().inv();
        const Quat<qScalar>& result_dual = - result_real * dual() * result_real;
        return DualQuat( result_real, result_dual );
    }
    // log
    constexpr inline DualQuat log() const noexcept {
        const Quat<qScalar>& result_real = real().log();
        const Quat<qScalar>& result_dual = real().inv() * dual();
        return DualQuat( result_real, result_dual );
    }
    // exp
    constexpr inline DualQuat exp() const noexcept {
        const Quat<qScalar>& result_real = real().exp();
        const Quat<qScalar>& result_dual = result_real * real().inv() * dual();
        return DualQuat( result_real, result_dual );
    }
    // pow
    constexpr inline DualQuat pow(const qScalar index) const noexcept {
        return (this->log() * index).exp();
    }
    // hamiplus
    constexpr inline std::array<std::array<qScalar, 8>, 8> hamiplus() const noexcept {
        const std::array<std::array<qScalar, 4>, 4> result_realhami = real().hamiplus();
        const std::array<std::array<qScalar, 4>, 4> result_dualhami = dual().hamiplus();
        // Initialize a zero container
        std::array<std::array<qScalar, 8>, 8> res{}; 
        for (int i=0; i<4; ++i) {
            // fill the first 4x4 block
            std::copy(result_realhami[i].begin(), result_realhami[i].end(), res[i].begin());
            // the Second 4x4 block is [0]
            // fill the third 4x4 block
            std::copy(result_dualhami[i].begin(), result_dualhami[i].end(), res[i+4].begin());
            std::copy(result_realhami[i].begin(), result_realhami[i].end(), res[i+4].begin()+4);
        }
        return res;
    }
    // haminus
    constexpr inline std::array<std::array<qScalar, 8>, 8> haminus() const noexcept {        
        const std::array<std::array<qScalar, 4>, 4> result_realhami = real().haminus();
        const std::array<std::array<qScalar, 4>, 4> result_dualhami = dual().haminus();
        // Initialize a zero container
        std::array<std::array<qScalar, 8>, 8> res{};
        for (int i=0; i<4; ++i) {
            // fill the first 4x4 block
            std::copy(result_realhami[i].begin(), result_realhami[i].end(), res[i].begin());
            // the Second 4x4 block is [0]
            // fill the third 4x4 block
            std::copy(result_dualhami[i].begin(), result_dualhami[i].end(), res[i+4].begin());
            // fill the fourth 4x4 block
            std::copy(result_realhami[i].begin(), result_realhami[i].end(), res[i+4].begin()+4);
        }
        return res;
    }
    // query 
    constexpr inline Quat<qScalar> real() const noexcept { return _data[0]; }
    constexpr inline Quat<qScalar> dual() const noexcept { return _data[1]; }
    // data
    constexpr inline const qScalar* data() const noexcept { return _data.data()[0].data(); }
    constexpr inline std::array<qScalar, 8> array() const noexcept { 
        std::array<qScalar, 8> res;
        std::copy(data(), data()+8, res.begin());
        return res; 
    }
    // to_string
    constexpr inline std::string to_string() const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(PRINT_PRECISION) <<  real() << " + " << " ϵ ( " << dual() << " )";
        return oss.str();
    }
    // Default
            virtual ~DualQuat()=default;
                    DualQuat(const DualQuat& dq)=default;
                    DualQuat(DualQuat&& dq)=default;
    DualQuat& operator=(const DualQuat& dq)=default;
    DualQuat& operator=(DualQuat&& dq)=default;
};
template<typename qScalar, typename>
class Pose: public DualQuat<qScalar>{
public:
    // Default Constructor
    constexpr explicit Pose() noexcept
        : DualQuat<qScalar>( 1 ) {
    }
    // Array Constructor
    constexpr explicit Pose(const std::array<qScalar, 8> arr8) noexcept
        : DualQuat<qScalar>( arr8 ) {
        this->normalize();
    }
    // Scalar Constructor
    constexpr explicit Pose(const qScalar w1, const qScalar x1=0, const qScalar y1=0, const qScalar z1=0, 
                          const qScalar w2=0, const qScalar x2=0, const qScalar y2=0, const qScalar z2=0) noexcept
        : DualQuat<qScalar>( w1, x1, y1, z1, w2, x2, y2, z2 ) {
        this->normalize();
    }
    // Rotation-Translation Constructor 
    template<typename Scalar1, typename Scalar2>
    constexpr explicit Pose(const Rotation<Scalar1>& rotation, const Translation<Scalar2> translation) noexcept
        : DualQuat<qScalar>(rotation, translation * rotation * 0.5) {
    }
    // Rotation Constructor
    template<typename Scalar>
    constexpr explicit Pose(const Rotation<Scalar>& rotation) 
        : DualQuat<qScalar>(rotation) {

    }
    // Translation Constructor
    template<typename Scalar>
    constexpr explicit Pose(const Translation<Scalar> translation) 
        : DualQuat<qScalar>(Quat<qScalar>(1), translation * 0.5) {

    }
    // DualQuat Constructor
    template <typename Scalar>
    constexpr Pose(const DualQuat<Scalar>& other) noexcept
        : DualQuat<qScalar>( other ) {
        this->normalize();
    }
    // DualQuat Assignment
    template<typename Scalar>
    constexpr inline Pose& operator=(const DualQuat<Scalar>& other) noexcept {
        this->_real() = other.real();
        this->_dual() = other.dual();
        this->normalize();
        return *this;
    }
    constexpr Rotation<qScalar> rotation() const noexcept { return Rotation<qScalar>(this->real()); }
    constexpr Translation<qScalar> translation() const noexcept { return Translation<qScalar>(this->dual() * this->real().conj() * 2); }

    template<typename First_, typename... Args_>
    constexpr static Pose build_from(const First_& first, const Args_&... args){
        return Pose(build_from(first) * build_from(args...));
    }  
    template<typename Scalar>
    constexpr static Rotation<qScalar> build_from(const Rotation<Scalar>& rotation){
        return rotation;
    }
    template<typename Scalar>
    constexpr static Pose build_from(const Translation<Scalar>& translation){
        return Pose(Rotation<qScalar>(), translation);
    }
    template<typename Scalar>
    constexpr static Pose build_from(const Pose<Scalar>& pose){
        return pose;
    }
    // Default
            virtual ~Pose()=default;
                    Pose(const Pose& dq)=default;
                    Pose(Pose&& dq)=default;
    Pose& operator=(const Pose& dq)=default;
    Pose& operator=(Pose&& dq)=default;
};

using Quatu = Quat<std::uint8_t>;
using Rotu = Rotation<std::uint8_t>;
using Tranu = Translation<std::uint8_t>;
using Unitu = UnitAxis<std::uint8_t>;
using DQu = DualQuat<std::uint8_t>;
using Poseu = Pose<std::uint8_t>;

using Quati = Quat<int>;
using Roti = Rotation<int>;
using Trani = Translation<int>;
using Uniti = UnitAxis<int>;
using DQi = DualQuat<int>;
using Posei = Pose<int>;

using Quatf = Quat<float>;
using Rotf = Rotation<float>;
using Tranf = Translation<float>;
using Unitf = UnitAxis<float>;
using DQf = DualQuat<float>;
using Posef = Pose<float>;

using Quatd = Quat<double>;
using Rotd = Rotation<double>;
using Trand = Translation<double>;
using Unitd = UnitAxis<double>;
using DQd = DualQuat<double>;
using Posed = Pose<double>;

using Quatld = Quat<long double>;
using Rotld = Rotation<long double>;
using Tranld = Translation<long double>;
using Unitld = UnitAxis<long double>;
using DQld = DualQuat<long double>;
using Poseld = Pose<long double>;

constexpr UnitAxis<std::uint8_t> i_(1,0,0);
constexpr UnitAxis<std::uint8_t> j_(0,1,0);
constexpr UnitAxis<std::uint8_t> k_(0,0,1);

}