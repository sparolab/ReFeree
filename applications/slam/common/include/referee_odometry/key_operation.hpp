#include <iostream>
#include <thread>
#include <pthread.h>
#include <atomic>
#include <unistd.h>
#include <termios.h>

std::atomic<bool> finished(false); // 프로그램 종료 신호

// 터미널 설정을 변경하지 않고 키 입력을 받는 함수
void getch(bool& signal);