#ifndef OBJECTS_H_INCLUDED
#define OBJECTS_H_INCLUDED
#include "tools.h"
#include <optional>


class Object {
public:
    virtual std::optional<Vec3> intersection(const Vec3&, const Vec3&) const = 0;
    virtual Vec3 norm_dir(const Vec3&) const = 0;
    virtual float get_reflection_coeff(const Vec3&) const = 0;
    virtual ~Object() {}
};


class Plane : public Object {
private:
    Vec3 point;
    Vec3 norm;
    float reflection_coeff;

public:
    Plane(const Vec3& point, float refl_coeff=0.5)
        : point(point), norm({0, -1, 0}), reflection_coeff(refl_coeff) {}

    Plane(const Vec3& point, const Vec3& norm, float refl_coeff=0.5)
        : point(point), norm(norm.normalized()), reflection_coeff(refl_coeff) {

        if (norm.norm() < 1e-6) {
            throw std::runtime_error("Norm cannot be a zero vector");
        }
    }

    std::optional<Vec3> intersection(const Vec3& line_point, const Vec3& line_dir) const override {
        if (fabs(norm.dot(line_dir)) < 1e-6) {
            return std::nullopt;
        }

        float t = norm.dot(point - line_point) / norm.dot(line_dir);

        if (t > 0) {
            return line_point + line_dir * t;
        } 

        return std::nullopt;
    }

    Vec3 norm_dir(const Vec3&) const override {
        return norm;
    }

    float get_reflection_coeff(const Vec3&) const override {
        return reflection_coeff;
    }
};


class ChessPlane : public Object {
private:
    Vec3 point;       
    Vec3 norm;         
    float square_size; 
    float reflection_coeff_black;
    float reflection_coeff_white;

public:
    ChessPlane(const Vec3& point, float square_size=0.5, float refl_coeff_black = 0.1, float refl_coeff_white = 0.3)
        : point(point), norm(0, -1, 0), square_size(square_size), reflection_coeff_black(refl_coeff_black), reflection_coeff_white(refl_coeff_white) {}

    ChessPlane(const Vec3& point, const Vec3& norm, float square_size=0.5, float refl_coeff_black = 0.1, float refl_coeff_white = 0.3)
        : point(point), norm(norm.normalized()), square_size(square_size), reflection_coeff_black(refl_coeff_black), reflection_coeff_white(refl_coeff_white) {

        if (norm.norm() < 1e-6) {
            throw std::runtime_error("Norm cannot be a zero vector");
        }
    }


    std::optional<Vec3> intersection(const Vec3& line_point, const Vec3& line_dir) const override {
        if (fabs(norm.dot(line_dir)) < 1e-6) {
            return std::nullopt;
        }

        float t = norm.dot(point - line_point) / norm.dot(line_dir);

        if (t > 0) {
            return line_point + line_dir * t;
        }

        return std::nullopt;
    }

    Vec3 norm_dir(const Vec3&) const override {
        return norm;
    }

    float get_reflection_coeff(const Vec3& intersection) const override {
        float x = intersection.x;
        float z = intersection.z;

        int x_index = static_cast<int>(floor(x / square_size));
        int z_index = static_cast<int>(floor(z / square_size));

        if ((x_index + z_index) % 2 == 0) {
            return reflection_coeff_black;
        } else {
            return reflection_coeff_white;
        }
    }
};


class Sphere : public Object {
private:
    Vec3 center;
    float radius;
    float reflection_coeff;

public:
    Sphere(const Vec3& center, float radius, float refl_coeff=0.5) 
        : center(center), radius(radius), reflection_coeff(refl_coeff) {}

    std::optional<Vec3> intersection(const Vec3& line_point, const Vec3& line_dir) const override {
        Vec3 oc = line_point - center;
        float a = line_dir.dot(line_dir);
        float b = 2.0 * oc.dot(line_dir);
        float c = oc.dot(oc) - radius * radius;

        float discriminant = b * b - 4 * a * c;

        if (discriminant > 0) {
            float t1 = (-b - sqrt(discriminant)) / (2.0 * a);
            float t2 = (-b + sqrt(discriminant)) / (2.0 * a);
            Vec3 intersection1 = line_point + line_dir * t1;
            Vec3 intersection2 = line_point + line_dir * t2;
            if (t1 > 0 && t2 > 0) {
                return t1 < t2 ? intersection1 : intersection2;
            } else if (t1 > 0) {
                return intersection1;
            } else if (t2 > 0) {
                return intersection2;
            }
        } else if (discriminant == 0) {
            float t = -b / (2.0 * a);
            if (t > 0) {
                return line_point + line_dir * t;
            }
        }

        return std::nullopt;
    }

    Vec3 norm_dir(const Vec3& point) const override {
        return (point - center).normalized();
    }

    float get_reflection_coeff(const Vec3&) const override {
        return reflection_coeff;
    }

};


class Rect : public Object {
private:
    Vec3 center;
    Vec3 norm;
    Vec3 width_dir;
    Vec3 height_dir;
    float width;
    float height;
    float reflection_coeff;

public:
    Rect() {}
    Rect(const Vec3& center, const Vec3& norm, const Vec3& width_dir, float width, float height, float reflection_coeff)
        : center(center), norm(norm.normalized()), width_dir(width_dir.normalized()), height_dir(norm.cross(width_dir).normalized()), width(width), height(height), reflection_coeff(reflection_coeff) {
        if (norm.norm() < 1e-6 || width_dir.norm() < 1e-6) {
            throw std::runtime_error("Norm or width_dir vector cannot be a zero vector");
        }
        if (norm.dot(width_dir) > 1e-6) {
            throw std::runtime_error("Norm and width_dir must be orthogonal");
        }
    }

    std::optional<Vec3> intersection(const Vec3& line_point, const Vec3& line_dir) const override {
       if (fabs(norm.dot(line_dir)) < 1e-6) {
            return std::nullopt;
        }

        float t = norm.dot(center - line_point) / norm.dot(line_dir);

        if (t > 0) {
            Vec3 possible_intersection = line_point + line_dir * t;
            Vec3 local_point = possible_intersection - center;
            if (fabs(width_dir.dot(local_point)) < width/2 && fabs(height_dir.dot(local_point)) < height/2) {
                return possible_intersection;
            }
        }

        return std::nullopt;
    }

    Vec3 norm_dir(const Vec3&) const override {
        return norm;
    }

    float get_reflection_coeff(const Vec3&) const override {
        return reflection_coeff;
    }
};


class RectPrism : public Object {
// this class doesn't work if height_dir or width_dir are not parallel to the axes
private:
    Vec3 base_center;
    Vec3 height_dir;
    Vec3 width_dir;
    Vec3 length_dir;
    float height;
    float width;
    float length;
    float reflection_coeff;

    Rect faces[6];

public:
    RectPrism(Vec3 base_center, Vec3 height_dir, Vec3 width_dir, float height, float width, float length, float reflection_coeff)
        : base_center(base_center), height_dir(height_dir.normalized()), width_dir(width_dir.normalized()), length_dir(height_dir.cross(width_dir).normalized()), height(height), width(width), length(length), reflection_coeff(reflection_coeff) {
        
        if (height_dir.norm() < 1e-6 || width_dir.norm() < 1e-6) {
            throw std::runtime_error("Height_dir or width_dir vector cannot be a zero vector");
        }

        if (height_dir.dot(width_dir) > 1e-6) {
            throw std::runtime_error("Height_dir and width_dir must be orthogonal");
        }

        faces[0] = Rect(base_center, -height_dir, width_dir, width, length, reflection_coeff); // bottom
        faces[1] = Rect(base_center + height_dir*height, height_dir, width_dir, width, length, reflection_coeff); // top
        faces[2] = Rect(base_center - length_dir*(length/2) + height_dir*(height/2), -length_dir, width_dir, width, height, reflection_coeff); // right
        faces[3] = Rect(base_center + length_dir*(length/2) + height_dir*(height/2), length_dir, width_dir, width, height, reflection_coeff); // left
        faces[4] = Rect(base_center - width_dir*(width/2) + height_dir*(height/2), -width_dir, length_dir, length, height, reflection_coeff); // front
        faces[5] = Rect(base_center + width_dir*(width/2) + height_dir*(height/2), width_dir, length_dir, length, height, reflection_coeff); // back
    }
    
    std::optional<Vec3> intersection(const Vec3& line_point, const Vec3& line_dir) const override {
        float closest = std::numeric_limits<float>::max();
        std::optional<Vec3> closest_intersection = std::nullopt;

        for (const auto& face : faces) {
            auto face_intersection = face.intersection(line_point, line_dir);
            if (face_intersection) {
                float t = (face_intersection.value() - line_point).norm();
                if (t < closest) {
                    closest = t;
                    closest_intersection = face_intersection;
                }
            }
        }

        return closest_intersection;
    }

    Vec3 norm_dir(const Vec3& point) const override {
        Vec3 min = base_center - width_dir*(width/2) - length_dir*(length/2);
        Vec3 max = base_center + width_dir*(width/2) + length_dir*(length/2) + height_dir*height;

        if (std::abs(point.dot(height_dir) - min.dot(height_dir)) < 1e-6) return -height_dir;
        if (std::abs(point.dot(height_dir)- max.dot(height_dir)) < 1e-6) return height_dir;
        if (std::abs(point.dot(width_dir) - min.dot(width_dir)) < 1e-6) return -width_dir;
        if (std::abs(point.dot(width_dir) - max.dot(width_dir)) < 1e-6) return width_dir;
        if (std::abs(point.dot(length_dir) - min.dot(length_dir)) < 1e-6) return -length_dir;
        return length_dir;
    }

    float get_reflection_coeff(const Vec3&) const override {
        return reflection_coeff;
    }
};


class Cube : public RectPrism {
public:
    Cube(const Vec3& base_center, const Vec3& height_dir, const Vec3& width_dir, float side, float reflection_coeff)
        : RectPrism(base_center, height_dir, width_dir, side, side, side, reflection_coeff) {}
};


class Cylinder : public Object {
private:
    Vec3 base_center;
    Vec3 axis_dir;
    float radius;
    float height;
    float reflection_coeff;

public:
    Cylinder(const Vec3& base_center, const Vec3& axis_dir, float radius, float height, float reflection_coeff)
        : base_center(base_center), axis_dir(axis_dir.normalized()), radius(radius), height(height), reflection_coeff(reflection_coeff) {}

    std::optional<Vec3> intersection_side(const Vec3& line_point, const Vec3& line_dir) const {
        Vec3 oc = line_point - base_center;
        Vec3 d = line_dir - axis_dir * line_dir.dot(axis_dir);
        Vec3 o = oc - axis_dir * oc.dot(axis_dir);

        float a = d.dot(d);
        float b = 2.0 * d.dot(o);
        float c = o.dot(o) - radius * radius;

        float discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return std::nullopt;
        }

        float t0 = (-b - sqrt(discriminant)) / (2 * a);
        float t1 = (-b + sqrt(discriminant)) / (2 * a);

        float t = (t0 > 0) ? t0 : t1;
        if (t < 0) {
            return std::nullopt;
        }

        Vec3 intersection = line_point + line_dir * t;
        float projection_length = (intersection - base_center).dot(axis_dir);

        if (projection_length < 0 || projection_length > height) {
            return std::nullopt;
        }

        return intersection;
    }

    std::optional<Vec3> base_intersection(const Vec3& line_point, const Vec3& line_dir, const Vec3& base_center) const {
        Vec3 base_norm = axis_dir;
        if (fabs(base_norm.dot(line_dir)) < 1e-6) {
            return std::nullopt;
        }

        float t = base_norm.dot(base_center - line_point) / base_norm.dot(line_dir);
        if (t < 0) {
            return std::nullopt;
        }

        Vec3 intersection = line_point + line_dir * t;
        if ((intersection - base_center).norm() <= radius) {
            return intersection;
        }

        return std::nullopt;
    }

    std::optional<Vec3> intersection(const Vec3& line_point, const Vec3& line_dir) const override {
        auto side_intersection = intersection_side(line_point, line_dir);

        auto bottom_intersection = base_intersection(line_point, line_dir, base_center);

        Vec3 top_center = base_center + axis_dir * height;
        auto top_intersection = base_intersection(line_point, line_dir, top_center);

        std::optional<Vec3> result = std::nullopt;
        float min_t = std::numeric_limits<float>::max();

        if (side_intersection) {
            float t = (side_intersection.value() - line_point).norm();
            if (t < min_t) {
                min_t = t;
                result = side_intersection;
            }
        }
        if (bottom_intersection) {
            float t = (bottom_intersection.value() - line_point).norm();
            if (t < min_t) {
                min_t = t;
                result = bottom_intersection;
            }
        }
        if (top_intersection) {
            float t = (top_intersection.value() - line_point).norm();
            if (t < min_t) {
                min_t = t;
                result = top_intersection;
            }
        }

        return result;
    }

    Vec3 norm_dir(const Vec3& point) const override {
        float projection_length = (point - base_center).dot(axis_dir);
        if (fabs(projection_length) < 1e-6) {
            return -axis_dir;
        }
        if (fabs(projection_length - height) < 1e-6) {
            return axis_dir;
        }

        Vec3 projection = base_center + axis_dir * projection_length;
        return (point - projection).normalized();
    }

    float get_reflection_coeff(const Vec3&) const override {
        return reflection_coeff;
    }
};


class Cone : public Object {
// this class doesn't work if axis is not parallel to one of the axes
private:
    Vec3 base_center;      
    Vec3 axis;             
    float radius;          
    float height;          
    float reflection_coeff; 

    Vec3 vertex;    

private:
    std::optional<Vec3> intersect_side(const Vec3& line_point, const Vec3& line_dir) const {
        Vec3 v = line_point - vertex;
        float cos2 = height * height / (height * height + radius * radius);

        float a = line_dir.dot(axis) * line_dir.dot(axis) - cos2 * line_dir.dot(line_dir);
        float b = 2 * (line_dir.dot(axis) * v.dot(axis) - cos2 * line_dir.dot(v));
        float c = v.dot(axis) * v.dot(axis) - cos2 * v.dot(v);

        float discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            return std::nullopt;
        }

        float t1 = (-b - sqrt(discriminant)) / (2 * a);
        float t2 = (-b + sqrt(discriminant)) / (2 * a);

        float t = std::min(t1, t2);
        if (t < 1e-6) {
            t = std::max(t1, t2);
        }

        if (t < 1e-6) {
            return std::nullopt;
        }

        Vec3 hit_point = line_point + line_dir * t;

        if ((hit_point - base_center).dot(axis) < 0 || (hit_point - vertex).dot(-axis) < 0) {
            return std::nullopt;
        }

        return hit_point;
    }

    std::optional<Vec3> intersect_base(const Vec3& line_point, const Vec3& line_dir) const {
        float denom = axis.dot(line_dir);
        if (fabs(denom) < 1e-6) {
            return std::nullopt;
        }

        float t = (base_center - line_point).dot(axis) / denom;
        if (t < 1e-6) {
            return std::nullopt;
        }

        Vec3 hit_point = line_point + line_dir * t;

        if ((hit_point - base_center).norm() > radius) {
            return std::nullopt;
        }

        return hit_point;
    }       

public:
    Cone(const Vec3& base_center, const Vec3& axis, float radius, float height, float refl_coeff = 0.5)
        : base_center(base_center), 
          axis(axis.normalized()), 
          radius(radius), 
          height(height), 
          reflection_coeff(refl_coeff),
          vertex(base_center + axis * height) {
        if (fabs(axis.norm()) < 1e-6) {
            throw std::runtime_error("Axis cannot be a zero vector");
        }
        if (radius <= 0 || height <= 0) {
            throw std::runtime_error("Radius and height must be positive values");
        }
    }

    std::optional<Vec3> intersection(const Vec3& line_point, const Vec3& line_dir) const override {
        std::optional<Vec3> side_hit = intersect_side(line_point, line_dir);

        std::optional<Vec3> base_hit = intersect_base(line_point, line_dir);

        if (side_hit && base_hit) {
            float side_dist = (*side_hit - line_point).norm();
            float base_dist = (*base_hit - line_point).norm();
            return (side_dist < base_dist) ? side_hit : base_hit;
        }

        return side_hit ? side_hit : base_hit;
    }

    Vec3 norm_dir(const Vec3& point) const override {
        if (fabs((point - base_center).dot(axis)) < 1e-6) {
            return -axis;
        }

        Vec3 v = point - vertex;
        Vec3 proj_on_axis = axis * v.dot(axis);
        Vec3 radial = v - proj_on_axis;
        return (radial - proj_on_axis * (radius / height)).normalized();
    }

    float get_reflection_coeff(const Vec3&) const override {
        return reflection_coeff;
    }
};


#endif //OBJECTS_H_INCLUDED