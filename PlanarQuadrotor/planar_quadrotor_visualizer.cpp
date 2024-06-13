#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;
    int x_box_size = 100;
    int y_box_size = 50;
    /* x, y, theta coordinates */
    q_x = state[0] ;
    q_y = state[1] ;
    q_theta = state[2];

    Sint16 body_x[4] = { q_x + (x_box_size * cos(-q_theta) - y_box_size * sin(-q_theta)),
        q_x - (x_box_size * cos(q_theta) - y_box_size * sin(q_theta)),
        q_x - (x_box_size * cos(-q_theta) - y_box_size * sin(-q_theta)),
        q_x + (x_box_size * cos(q_theta) - y_box_size * sin(q_theta)) };
    Sint16 body_y[4] = { q_y + (y_box_size * cos(-q_theta) + x_box_size * sin(-q_theta)),
        q_y + (y_box_size * cos(q_theta) + x_box_size * sin(q_theta)),
        q_y - (y_box_size * cos(-q_theta) + x_box_size * sin(-q_theta)),
        q_y - (y_box_size * cos(q_theta) + x_box_size * sin(q_theta)) };

    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF); // AA RR GG BB
    filledPolygonColor(gRenderer.get(), body_x, body_y, 4, 0x55555555);
    //filledCircleColor(gRenderer.get(), q_x, q_y, 30, 0xFF5500FF); // 0xRRGGBBAA 
}
