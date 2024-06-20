#include "planar_quadrotor_visualizer.h"
#include <cmath>

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers
 */  
    int color1 = 0x99000000;
    int color2 = 0x22000000;
    int pom;
    float prev_xdot = 0;
    float prev_ydot = 0;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;
    const int x_offset = SCREEN_WIDTH / 2;
    const int y_offset = SCREEN_HEIGHT / 2;
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {

    Eigen::VectorXf state = quadrotor_ptr->GetState();

    Eigen::VectorXf goal = quadrotor_ptr->GetState() - quadrotor_ptr->GetControlState();
    float q_x, q_y, q_theta, xdot, ydot;
    int x_box_size = 50;
    int y_box_size = 15;
    int x_arm_size = 5;
    int y_arm_size = 10;
    int x_prop_size = 15;
    int y_prop_size = 7;
    int x_extra_prop_size = 25;
    /* x, y, theta coordinates */
    q_x = state[0] + x_offset;
    q_y = state[1] + y_offset;
    q_theta = state[2];
    xdot = state[3];
    ydot = state[4];

    Sint16 body_x[4] = { q_x + (x_box_size * cos(-q_theta) - y_box_size * sin(-q_theta)),
        q_x - (x_box_size * cos(q_theta) - y_box_size * sin(q_theta)),
        q_x - (x_box_size * cos(-q_theta) - y_box_size * sin(-q_theta)),
        q_x + (x_box_size * cos(q_theta) - y_box_size * sin(q_theta)) };
    Sint16 body_y[4] = { q_y + (y_box_size * cos(-q_theta) + x_box_size * sin(-q_theta)),
        q_y + (y_box_size * cos(q_theta) + x_box_size * sin(q_theta)),
        q_y - (y_box_size * cos(-q_theta) + x_box_size * sin(-q_theta)),
        q_y - (y_box_size * cos(q_theta) + x_box_size * sin(q_theta)) };

    Sint16 l_body_x[4] = { body_x[2] + (x_arm_size * cos(-q_theta) - y_arm_size * sin(-q_theta)),
        body_x[2] - (x_arm_size * cos(q_theta) - y_arm_size * sin(q_theta)),
        body_x[2] - (x_arm_size * cos(-q_theta) - y_arm_size * sin(-q_theta)),
        body_x[2] + (x_arm_size * cos(q_theta) - y_arm_size * sin(q_theta)) };
    Sint16 l_body_y[4] = { body_y[2] + (y_arm_size * cos(-q_theta) + x_arm_size * sin(-q_theta)),
        body_y[2] + (y_arm_size * cos(q_theta) + x_arm_size * sin(q_theta)),
        body_y[2] - (y_arm_size * cos(-q_theta) + x_arm_size * sin(-q_theta)),
        body_y[2] - (y_arm_size * cos(q_theta) + x_arm_size * sin(q_theta)) };
    Sint16 r_body_x[4] = { body_x[3] + (x_arm_size * cos(-q_theta) - y_arm_size * sin(-q_theta)),
        body_x[3] - (x_arm_size * cos(q_theta) - y_arm_size * sin(q_theta)),
        body_x[3] - (x_arm_size * cos(-q_theta) - y_arm_size * sin(-q_theta)),
        body_x[3] + (x_arm_size * cos(q_theta) - y_arm_size * sin(q_theta)) };
    Sint16 r_body_y[4] = { body_y[3] + (y_arm_size * cos(-q_theta) + x_arm_size * sin(-q_theta)),
        body_y[3] + (y_arm_size * cos(q_theta) + x_arm_size * sin(q_theta)),
        body_y[3] - (y_arm_size * cos(-q_theta) + x_arm_size * sin(-q_theta)),
        body_y[3] - (y_arm_size * cos(q_theta) + x_arm_size * sin(q_theta)) };

    Sint16 r_prop_x[6] = {
        r_body_x[3] - (x_prop_size * cos(-q_theta) - y_prop_size * sin(-q_theta)),
        r_body_x[3] + (x_prop_size * cos(-q_theta) - y_prop_size * sin(-q_theta)),
        r_body_x[3] + (x_extra_prop_size * cos(-q_theta)),
        r_body_x[3] + (x_prop_size * cos(q_theta) - y_prop_size * sin(q_theta)),
        r_body_x[3] - (x_prop_size * cos(q_theta) - y_prop_size * sin(q_theta)),
        r_body_x[3] - (x_extra_prop_size * cos(-q_theta))
    };
    Sint16 r_prop_y[6] = {
        r_body_y[3] - (y_prop_size * cos(-q_theta) + x_prop_size * sin(-q_theta)),
        r_body_y[3] + (y_prop_size * cos(-q_theta) + x_prop_size * sin(-q_theta)),
        r_body_y[3] + (x_prop_size * sin(-q_theta)),
        r_body_y[3] - (y_prop_size * cos(q_theta) + x_prop_size * sin(q_theta)),
        r_body_y[3] + (y_prop_size * cos(q_theta) + x_prop_size * sin(q_theta)),
        r_body_y[3] - (x_prop_size * sin(-q_theta))
    };
    Sint16 l_prop_x[6] = {
        l_body_x[2] - (x_prop_size * cos(-q_theta) - y_prop_size * sin(-q_theta)),
        l_body_x[2] + (x_prop_size * cos(-q_theta) - y_prop_size * sin(-q_theta)),
        l_body_x[2] + (x_extra_prop_size * cos(-q_theta)),
        l_body_x[2] + (x_prop_size * cos(q_theta) - y_prop_size * sin(q_theta)),
        l_body_x[2] - (x_prop_size * cos(q_theta) - y_prop_size * sin(q_theta)),
        l_body_x[2] - (x_extra_prop_size * cos(-q_theta))
    };
    Sint16 l_prop_y[6] = {
        l_body_y[2] - (y_prop_size * cos(-q_theta) + x_prop_size * sin(-q_theta)),
        l_body_y[2] + (y_prop_size * cos(-q_theta) + x_prop_size * sin(-q_theta)),
        l_body_y[2] + (x_prop_size * sin(-q_theta)),
        l_body_y[2] - (y_prop_size * cos(q_theta) + x_prop_size * sin(q_theta)),
        l_body_y[2] + (y_prop_size * cos(q_theta) + x_prop_size * sin(q_theta)),
        l_body_y[2] - (x_prop_size * sin(-q_theta))
    };

    if (sqrt(pow(xdot,2 )+pow(ydot,2)) > sqrt(pow(prev_xdot, 2) + pow(prev_ydot, 2))) {
        if (color1!= 0x99000000 && color1 != 0x22000000) {
            color1 = 0x99000000;
            color2 = 0x22000000;
        }
            pom = color1;
            color1 = color2;;
            color2 = pom;
    }
    else{
        if(color1!=0xFF005530 && color1!=0xFF550055){
            color1 = 0xFF005530;
            color2 = 0xFF550055;
        }
       pom = color1;
       color1 = color2;;
       color2 = pom;
    };
    //std::cout << " xdot = " << sqrt(pow(xdot, 2) + pow(ydot, 2)) << " xprevdot= " << sqrt(pow(prev_xdot, 2) + pow(prev_ydot, 2)) << std::endl;
    prev_xdot = xdot;
    prev_ydot = ydot;


    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF); // AA RR GG BB
    filledPolygonColor(gRenderer.get(), body_x, body_y, 4, 0xFF555555);
    filledPolygonColor(gRenderer.get(), l_body_x, l_body_y, 4, 0xFF555555);
    filledPolygonColor(gRenderer.get(), r_body_x, r_body_y, 4, 0xFF555555);
    filledPolygonColor(gRenderer.get(), r_prop_x, r_prop_y, 6, color1);
    filledPolygonColor(gRenderer.get(), l_prop_x, l_prop_y, 6, color2);
    filledCircleColor(gRenderer.get(), goal[0] + x_offset, goal[1] + y_offset, 2, 0xFF0000FF); // 0xRRGGBBAA chyba to zle bo jest 0xAABBGGRR
}
