#ifndef CAMERA_AND_LIGHT_H_INCLUDED
#define CAMERA_AND_LIGHT_H_INCLUDED
#include "tools.h"
#include <stdexcept>
#include <iostream>
#include <cmath>


class Camera {
private:
    Vec3 position = {0, 0, -1};
    Vec3 direction = {0, 0, 1};
    float camera_distance;
    char* screen;
    const int width;
    const int height;
    const float aspect;
    const float pixel_aspect;

public:
    Camera(int width, int height, float pixel_aspect, float fov=90):
        camera_distance(1.0 / std::tan(fov * M_PI / 360.0)),
        screen(new char[width * height]),
        width(width), height(height),
        aspect(static_cast<float>(width) / height),
        pixel_aspect(pixel_aspect) {}

    Camera(int width, int height, float pixel_aspect, const Vec3& position, float fov=90):
        position(position),
        camera_distance(1.0 / std::tan(fov * M_PI / 360.0)),
        screen(new char[width * height]),
        width(width), height(height),
        aspect(static_cast<float>(width) / height),
        pixel_aspect(pixel_aspect) {}
    
    Vec3 get_screen_position() const {
        return position + direction.normalized() * camera_distance;
    }

    Vec3 get_dir_to_pixel(int i, int j) const {
        float y = static_cast<float>(i) / height * 2 - 1;
        float x = static_cast<float>(j) / width * 2 - 1;
        x *= aspect * pixel_aspect;

        Vec3 screen_center = get_screen_position();

        Vec3 right = direction.cross(Vec3(0, 1, 0)).normalized();
        Vec3 up = right.cross(direction).normalized();
        Vec3 pixel_point = screen_center + right * x + up * y;

        return (pixel_point - position).normalized();
    }

    void set_fov(float fov) {
        camera_distance = 1.0 / std::tan(fov * M_PI / 360.0);
    }

    void set_position(const Vec3& new_position) {
        position = new_position;
    }

    void set_direction(const Vec3& new_direction) {
        direction = new_direction;
    }

    void move(const Vec3& displacement) {
        position += displacement;
    }

    void rotate(const Vec3& rotation_angles) {
        RotationMat rotation_matrix = RotationMat(rotation_angles);
        direction = (rotation_matrix * direction).normalized();
    }

    void rotate_around_origin(const Vec3& rotation_angles) {
        RotationMat rotation_matrix = RotationMat(rotation_angles);
        position = (rotation_matrix * position).normalized() * position.norm();
        direction = (-position).normalized();
    }

    void rotate_around_point(const Vec3& point, const Vec3& rotation_angles) {
        Vec3 translated_position = position - point;
        RotationMat rotation_matrix = RotationMat(rotation_angles);
        translated_position = (rotation_matrix * translated_position).normalized() * translated_position.norm();
        position = translated_position + point;
        direction = (-position + point).normalized();
    }

    char& operator[](size_t index) {
        if (index >= static_cast<size_t>(width * height)) {
            throw std::out_of_range("Index out of range");
        }
        return screen[index];
    }

    const char* get_screen() const {
        return screen;
    }

    Vec3 get_position() const {
        return position;
    }

    Vec3 get_dir() const {
        return direction;
    }

    ~Camera() {
        delete[] screen;
    }
};



class Light {
private:
    Vec3 position = Vec3(0, -100, -100);
    float power = 1;

public:
    Light() {}
    Light(const Vec3& position, float power=1): position(position), power(power) {}

    void set_position(const Vec3& new_position) {
        position = new_position;
    }

    void set_power(float new_power) {
        power = new_power;
    }

    void move(const Vec3& displacement) {
        position += displacement;
    }

    void rotate_around_origin(const Vec3& rotation_angles) {
        RotationMat rotation_matrix = RotationMat(rotation_angles);
        position = (rotation_matrix * position).normalized() * position.norm();
    }

    void rotate_around_point(const Vec3& point, const Vec3& rotation_angles) {
        Vec3 translated_position = position - point;
        RotationMat rotation_matrix = RotationMat(rotation_angles);
        translated_position = (rotation_matrix * translated_position).normalized() * translated_position.norm();
        position = translated_position + point;
    }

    Vec3 get_position() const {
        return position;
    }

    float get_power() const {
        return power;
    }
};

#endif //CAMERA_AND_LIGHT_H_UNCLUDED