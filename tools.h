#ifndef TOOLS_H_INCLUDED
#define TOOLS_H_INCLUDED
#include <cmath>
#include <vector>
#include <initializer_list>
#include <stdexcept>


class Vec3 {
public:
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z): x(x), y(y), z(z) {}

    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vec3& operator*=(float t) {
        x *= t;
        y *= t;
        z *= t;
        return *this;
    }

    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    Vec3 operator*(float t) const {
        return Vec3(x * t, y * t, z * t);
    }

    Vec3 rotate(float a, float b, float c) const {
        return Vec3(cos(b) * cos(c) * x - cos(b) * sin(c) * y + sin(b) * z,
                    (sin(a) * sin(b) * cos(c) + cos(a) * sin(c)) * x + (cos(a) * cos(c) - sin(a) * sin(b) * sin(c)) * y - sin(a) * cos(b) * z,
                    (sin(a) * sin(c) - cos(a) * sin(b) * cos(c)) * x + (cos(a) * sin(b) * sin(c) + sin(a) * cos(c)) * y + cos(a) * cos(b) * z);
    }

    Vec3 rotate(const Vec3& w) const {
        return Vec3(cos(w.y) * cos(w.z) * x - cos(w.y) * sin(w.z) * y + sin(w.y) * z,
                    (sin(w.x) * sin(w.y) * cos(w.z) + cos(w.x) * sin(w.z)) * x + (cos(w.x) * cos(w.z) - sin(w.x) * sin(w.y) * sin(w.z)) * y - sin(w.x) * cos(w.y) * z,
                    (sin(w.x) * sin(w.z) - cos(w.x) * sin(w.y) * cos(w.z)) * x + (cos(w.x) * sin(w.y) * sin(w.z) + sin(w.x) * cos(w.z)) * y + cos(w.x) * cos(w.y) * z);
    }

    float dot(const Vec3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }
    
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    float norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vec3 normalized() const {
        float n = norm();
        if (n == 0) throw std::runtime_error("Zero length vector!");
        return Vec3(x / n, y / n, z / n);
    }
};


class RotationMat {
private:
    std::vector<float> mat;

public:
    RotationMat(): mat(9, 0) {}

    RotationMat(const Vec3& rotation_angles): mat(9) {
        const float a = rotation_angles.x;
        const float b = rotation_angles.y;
        const float c = rotation_angles.z;
        mat[0] = cos(b)*cos(c);
        mat[1] = -cos(b)*sin(c);
        mat[2] = sin(b);
        mat[3] = sin(a)*sin(b)*cos(c) + cos(a)*sin(c);
        mat[4] = cos(a)*cos(c) - sin(a)*sin(b)*sin(c);
        mat[5] = -sin(a)*cos(b);
        mat[6] = sin(a)*sin(c) - cos(a)*sin(b)*cos(c);
        mat[7] = sin(a)*cos(c) + cos(a)*sin(b)*sin(c);
        mat[8] = cos(a)*cos(b);
    }

    RotationMat(std::initializer_list<float> values) {
        if (values.size() != 9) {
            throw std::invalid_argument("Matrix must have 9 elements.");
        }
        mat.assign(values.begin(), values.end());
    }

    Vec3 operator*(const Vec3& v) const {
        return Vec3(mat[0]*v.x + mat[1]*v.y + mat[2]*v.z,
                    mat[3]*v.x + mat[4]*v.y + mat[5]*v.z,
                    mat[6]*v.x + mat[7]*v.y + mat[8]*v.z);
    }

    RotationMat operator*(const RotationMat& other) const {
        RotationMat result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.mat[i * 3 + j] = 0;
                for (int k = 0; k < 3; ++k) {
                    result.mat[i * 3 + j] += mat[i * 3 + k] * other.mat[k * 3 + j];
                }
            }
        }
        return result;
    }

    bool is_null() const {
        float sum = 0;
        for(float x : mat) {
            sum += std::abs(x);
        }
        return (sum == 0);
    }
};

#endif //TOOLS_H_INCLUDED