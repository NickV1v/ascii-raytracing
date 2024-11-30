#include "engine.h"
// axes: X - left, Z - forward, Y - down

int main() {
    const int width = 274; // <- set your console window width
    const int height = 66; // <- set your console window height
    const float font_width = 6.0; // <- set your console font width (in pixels)
    const float font_height = 12.0; // <- set your console font height (in pixels)

    const float pixel_aspect = font_width / font_height;
    RaytracingEngine engine(width, height, pixel_aspect);
    
    engine.camera.set_position({0, -1.2, -1.2});
    engine.light.set_position({0, -10, -10});

    engine.scene.add_object(new ChessPlane({0, 0, 0}, 0.5, 0.1, 0.3));
    engine.scene.add_object(new Sphere({-1, -0.5, 0}, 0.5, 1));
    engine.scene.add_object(new Cone({0, -0.75, -1}, {0, 1, 0}, 0.3, 0.75, 1));
    engine.scene.add_object(new RectPrism({1, 0, 0}, {0, -1, 0}, {0, 0, 1}, 1, 1, 0.5, 1));
    engine.scene.add_object(new Cylinder({0.1768, -0.5, 0.8232}, {-1, 0, 1}, 0.35, 0.5, 1));

    Vec3 angular_velocity = {0.023, 0.025, 0.025};

    while (true) {
        engine.render_frame();
        engine.camera.rotate_around_origin(angular_velocity);
    }
}