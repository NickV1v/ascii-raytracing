#include "engine.h"
// axes: X - left, Z - forward, Y - down

int main() {
    const int width = 274; // <- set your console window width
    const int height = 66; // <- set your console window height
    const float font_width = 6.0; // <- set your console font width (in pixels)
    const float font_height = 12.0; // <- set your console font height (in pixels)

    const float pixel_aspect = font_width / font_height;
    RaytracingEngine engine(width, height, pixel_aspect);
    
    engine.camera.set_position({0, -0.1, -0.6});
    engine.light.set_position({0, -100, -100});

    engine.scene.add_object(new ChessPlane({0, 0, 0}, 0.5, 0.1, 0.3));
    engine.scene.add_object(new Sphere({-0.5, -0.5, 0}, 0.5, 1));
    engine.scene.add_object(new Sphere({0.5, -0.5, 0}, 0.5, 1));

    Vec3 angular_velocity = {0, 0.025, 0};
    Vec3 camera_focus = {0, -0.5, 0};

    while (true) {
        engine.render_frame();
        engine.camera.rotate_around_point(camera_focus, angular_velocity);
        if (fabs((engine.camera.get_position() - camera_focus).dot({0, 0, 1})) < 0.5) {
            angular_velocity = -angular_velocity;
        }
    }
}