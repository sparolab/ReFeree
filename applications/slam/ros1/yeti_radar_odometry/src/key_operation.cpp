#include "key_operation.hpp"

void getch(bool& signal) {
    char buf = 0;
    struct termios old = {0};
    if(tcgetattr(0, &old) < 0)
        perror("tcgetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if(tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");

    while (!finished.load()) {
        if(read(0, &buf, 1) < 0)
            perror("read()");
        if (buf == ' ') {
            signal = !signal;
        } else if (buf == 'q' || buf == 'Q') {
            finished.store(true);
        }
    }

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if(tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
}