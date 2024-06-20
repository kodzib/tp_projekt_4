/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include <matplot/matplot.h>
#include <thread>
#include <iostream>
#include <windows.h>
#include <mmsystem.h>
#include <mciapi.h>
#pragma comment(lib, "winmm.lib")

Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 0.004, 0.004, 400, 0.005, 0.045, 2 / 2 / M_PI;
    R.row(0) << 30, 7;
    R.row(1) << 7, 30;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

//plotowanie
void plot(const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& theta, std::atomic<bool>& plot_active) {
    plot_active = false;
    matplot::plot3(x, y, theta);
    //matplot::plot(x, y);
    matplot::xlabel("X");
    matplot::ylabel("Y");
    matplot::zlabel("Theta");
    matplot::show();
}

//musialem dac dwa rozne audio to mozna usuanc t¹ funckje lub dac tam jakeis variable co sie nazwe pliku bedzie wpisywac
void play_audio() {
    PlaySound("sound.wav", GetModuleHandle(NULL), SND_FILENAME | SND_ASYNC | SND_LOOP);
    std::cout << std::endl << "audio" << std::endl;
}

void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    /**
     * TODO: Extend simulation
     * 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
     *    [x, y, 0, 0, 0, 0]
     * 2. Update PlanarQuadrotor from simulation when goal is changed
    */
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    initial_state << 0, 0, 0, 0, 0, 0;
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.001;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    /**
     * TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * 2. Plot trajectory using matplot++ when key 'p' is clicked
    */
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        float x_p, y_p;
        bool audio_active1 = false;
        bool audio_active2 = false;
        float prev_xdot = 0;
        float prev_ydot = 0;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);
        while (!quit)
        {
            static std::atomic<bool> plot_active = false;
            //events
            while (SDL_PollEvent(&e) != 0)
            {

                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEMOTION)
                {
                    SDL_GetMouseState(&x, &y);
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN) {
                    goal_state << x - (SCREEN_WIDTH / 2), y - (SCREEN_HEIGHT / 2), 0, 0, 0, 0;
                    quadrotor.SetGoal(goal_state);
                }
                else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_p && plot_active == false) {
                    plot_active = true;
                    std::thread plot_thread(plot, x_history, y_history, theta_history, std::ref(plot_active));
                    plot_thread.detach();
                    plot_active = false;
                }


            }
            SDL_Delay((int)dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);



            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);

            // Update trajectory history
            x_history.push_back(quadrotor.GetState()[0]);
            y_history.push_back(quadrotor.GetState()[1]);
            theta_history.push_back(quadrotor.GetState()[2]);

            if (sqrt(pow(quadrotor.GetState()[3], 2) + pow(quadrotor.GetState()[4], 2)) > sqrt(pow(prev_xdot, 2) + pow(prev_ydot, 2)) && audio_active1 == false) {
                PlaySound("sound125.wav", GetModuleHandle(NULL), SND_FILENAME | SND_ASYNC | SND_LOOP);
                audio_active1 = true;
                audio_active2 = true;
            }
            else if (sqrt(pow(quadrotor.GetState()[3], 2) + pow(quadrotor.GetState()[4], 2)) <= sqrt(pow(prev_xdot, 2) + pow(prev_ydot, 2)) && audio_active2 == true) {
                PlaySound("sound.wav", GetModuleHandle(NULL), SND_FILENAME | SND_ASYNC | SND_LOOP);
                audio_active2 = false;
                audio_active1 = false;
            };
            prev_xdot = quadrotor.GetState()[3];
            prev_ydot = quadrotor.GetState()[4];


        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}
