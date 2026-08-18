#ifndef PROPORTIONAL_NAV_HPP
#define PROPORTIONAL_NAV_HPP

#include <cmath>
#include <vector>   
#include <iostream>
#include <algorithm>

struct Vector {
    double x;
    double y;

    Vector() : x(0), y(0) {}
    Vector(double x_, double y_) : x(x_), y(y_) {}

    Vector operator+(const Vector& other) const {
        return Vector(x + other.x, y + other.y);
    }

    Vector operator-(const Vector& other) const {
        return Vector(x - other.x, y - other.y);
    }

    Vector operator*(double scalar) const {
        return Vector(x * scalar, y * scalar);
    }

    Vector operator/(double scalar) const {
        if (std::abs(scalar) < 1e-10) return Vector(0, 0);
        return Vector(x / scalar, y / scalar);
    }

    Vector componentwiseMultiply(const Vector& other) const {
        return Vector(x * other.x, y * other.y);
    }

    Vector componentwiseDivide(const Vector& other) const {
        double newX = (std::abs(other.x) < 1e-10) ? 0 : x / other.x;
        double newY = (std::abs(other.y) < 1e-10) ? 0 : y / other.y;
        return Vector(newX, newY);
    }

    double magnitude() const {
        return std::sqrt(x * x + y * y);
    }

    double dot(const Vector& other) const {
        return x * other.x + y * other.y;
    }

    Vector normalized() const {
        double mag = magnitude();
        if (mag < 1e-10) return Vector(0, 0);
        return Vector(x / mag, y / mag);
    }

    Vector perpendicular() const {
        return Vector(-y, x);
    }

    Vector minus() const {
        return Vector(-x, -y);
    }

    Vector rotate(double angle) const {
        double cosA = std::cos(angle);
        double sinA = std::sin(angle);
        return Vector(x * cosA - y * sinA, x * sinA + y * cosA);
    }

    Vector projectOnto(const Vector& other) const { 
        double otherMagSq = other.x * other.x + other.y * other.y;
        if (otherMagSq < 1e-10) return Vector(0, 0);
        double dotProduct = dot(other);
        return other * (dotProduct / otherMagSq);
    }

    friend Vector operator*(double scalar, const Vector& v) {
        return v * scalar;
    }

    double cross(const Vector& other) const {
        return x * other.y - y * other.x;
    }

    double angleBetween(const Vector& other) const {
        double dotProd = dot(other);
        double mags = magnitude() * other.magnitude();
        if (mags < 1e-10) return 0.0;
        double cosTheta = std::clamp(dotProd / mags, -1.0, 1.0);
        return std::acos(cosTheta);
    }

    double angle() const {
        return std::atan2(y, x);
    }
};

struct Bird {
    Vector position;
    Vector velocity;
    Vector acceleration;
    double mass;
    double theta;

    Bird(const Vector& pos, const Vector& vel, double m)
        : position(pos), velocity(vel), acceleration(0, 0), mass(m), theta(vel.angle()) {}
};

struct Target {
    Vector position;
    Vector velocity;

    Target(const Vector& pos, const Vector& vel)
        : position(pos), velocity(vel) {}
};

#endif