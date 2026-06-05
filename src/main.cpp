#include <iostream>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "cnc_controller.h"

volatile bool g_running = true;

void signalHandler(int sig) { g_running = false; }

int getKeyPress() {
    struct termios oldt, newt;
    int ch;
    int oldf;
    
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    
    ch = getchar();
    
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    
    if(ch != EOF) {
        switch(ch) {
            case 'w': case 'W': return BUTTON_UP;
            case 's': case 'S': return BUTTON_DOWN;
            case 'a': case 'A': return BUTTON_LEFT;
            case 'd': case 'D': return BUTTON_RIGHT;
            case ' ': case '\n': return BUTTON_ENTER;
            case 27: return BUTTON_ESC;
        }
    }
    return 0;
}

int main() {
    signal(SIGINT, signalHandler);
    
    std::cout << "CNC Controller with bcm2835" << std::endl;
    std::cout << "Controls: W=Up, S=Down, A=Left, D=Right, Space=Enter, ESC=Exit" << std::endl;
    
    CNCController cnc;
    if(!cnc.initialize()) {
        std::cerr << "Failed to initialize CNC controller!" << std::endl;
        return 1;
    }
    
    std::cout << "CNC Controller Ready" << std::endl;
    std::cout << "Position: X=0 Y=0 Z=0 A=0" << std::endl;
    
    while(g_running) {
        int key = getKeyPress();
        if(key) {
            switch(key) {
                case BUTTON_UP: cnc.moveRelative(0, 10, 0, 0, 500); break;
                case BUTTON_DOWN: cnc.moveRelative(0, -10, 0, 0, 500); break;
                case BUTTON_LEFT: cnc.moveRelative(-10, 0, 0, 0, 500); break;
                case BUTTON_RIGHT: cnc.moveRelative(10, 0, 0, 0, 500); break;
                case BUTTON_ENTER: cnc.homeAll(); break;
                case BUTTON_ESC: g_running = false; break;
            }
            Position pos = cnc.getCurrentPosition();
            std::cout << "\rPos: X=" << pos.x << " Y=" << pos.y << " Z=" << pos.z << " A=" << pos.a << "   " << std::flush;
        }
        usleep(50000);
    }
    
    cnc.shutdown();
    std::cout << "\nShutting down..." << std::endl;
    return 0;
}
