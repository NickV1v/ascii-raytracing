#ifndef SCENE_H_INCLUDED
#define SCENE_H_INCLUDED
#include "tools.h"
#include "objects.h"
#include <vector>
#include <tuple>


class Scene {
private:
    std::vector<Object*> objects;

public:
    Scene() {}

    void add_object(Object* obj) {
        objects.push_back(obj);
    }

    std::optional<std::tuple<Vec3, Vec3, Object*>> get_nearest_intersection(const Vec3& line_point, const Vec3& line_dir, Object* excluded_obj = nullptr) const {
        float min_dist = INFINITY;
        Vec3 intersection;
        Vec3 norm_dir;
        Object* intersection_obj;
        std::optional<Vec3> curr_intersection;
        for(Object* obj : objects) {
            if (obj == excluded_obj) continue;
            curr_intersection = obj->intersection(line_point, line_dir);
            if (curr_intersection) {
                float curr_dist = (curr_intersection.value() - line_point).norm();
                if (curr_dist < min_dist) {
                    min_dist = curr_dist;
                    intersection = curr_intersection.value();
                    norm_dir = obj->norm_dir(intersection);
                    intersection_obj = obj;
                }
            }
        }

        if (min_dist == INFINITY) {
            return std::nullopt;
        }

        return std::make_tuple(intersection, norm_dir, intersection_obj);
    }

    bool is_shadow(const Vec3& line_point, const Vec3& line_dir, const Object* excluded_obj, float distance_to_light) const {
        for(Object* obj : objects) {
            if (obj == excluded_obj) continue;
            auto curr_intersection = obj->intersection(line_point, line_dir);
            if (curr_intersection) {
                float curr_dist = (curr_intersection.value() - line_point).norm();
                if (curr_dist < distance_to_light) {
                    return true;
                }
            }
        }
        return false;
    }

    ~Scene() {
        for(auto obj : objects) {
            delete obj;
        }
    }
};

#endif //SCENE_H_INCLUDED