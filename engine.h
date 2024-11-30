#ifndef ENGINE_H_INCLUDED
#define ENGINE_H_INCLUDED
#include "scene.h"
#include "objects.h"
#include "camera_and_light.h"
#include <iostream>
#include <windows.h>
#include <wchar.h>


class RaytracingEngine {
private:
    const int width;
    const int height;
    const int num_reflections;

    HANDLE hConsole;
    DWORD dwBytesWritten;

    static constexpr char gradient[] = " .:!/r(l1Z4H9W8$@";
    static constexpr int gradient_size = sizeof(gradient) - 1;

public:
    Camera camera;
    Light light;
    Scene scene;

    RaytracingEngine(int width, int height, float pixel_aspect, int num_reflections=5):
        width(width), height(height), num_reflections(num_reflections), camera(width, height, pixel_aspect) {
            hConsole = CreateConsoleScreenBuffer(GENERIC_READ | GENERIC_WRITE, 0, NULL, CONSOLE_TEXTMODE_BUFFER, NULL);
            SetConsoleActiveScreenBuffer(hConsole);
            dwBytesWritten = 0;
    }

    void render_frame() {
        for(int i=0; i<height; ++i) {
            for(int j=0; j<width; ++j) {
                float max_intensity = 1;
                float light_intensity = 0;
                float cum_reflection_coeff = 1;
                Vec3 ray_point = camera.get_position();
                Vec3 ray_dir = camera.get_dir_to_pixel(i, j);
                Object* excluded_obj = nullptr;
                
                for(int k=0; k<num_reflections; ++k) {
                    auto intersection_and_norm = scene.get_nearest_intersection(ray_point, ray_dir, excluded_obj);

                    if (intersection_and_norm) {
                        Vec3 intersection = std::get<0>(intersection_and_norm.value());
                        Vec3 norm_dir = std::get<1>(intersection_and_norm.value());
                        Object* intersection_obj = std::get<2>(intersection_and_norm.value());

                        Vec3 dir_to_light = (light.get_position() - intersection).normalized();
                        float cos_angle = norm_dir.dot(dir_to_light);

                        cum_reflection_coeff *= intersection_obj->get_reflection_coeff(intersection);

                        if (cos_angle > 0 && !scene.is_shadow(intersection, dir_to_light, intersection_obj, (light.get_position() - intersection).norm())) {
                            light_intensity += cum_reflection_coeff*cos_angle*light.get_power();
                        }

                        ray_point = intersection;
                        ray_dir = (ray_dir - norm_dir*2*ray_dir.dot(norm_dir)).normalized();
                        excluded_obj = intersection_obj;
                    } else {
                        break;
                    }
                }

                int idx = std::min(static_cast<int>(light_intensity/max_intensity*gradient_size), gradient_size - 1);
                camera[i*width + j] = gradient[idx];   
            }
        }
		WriteConsoleOutputCharacter(hConsole, camera.get_screen(), width * height, { 0, 0 }, &dwBytesWritten);
    }
};

#endif //ENGINE_H_INCLUDED