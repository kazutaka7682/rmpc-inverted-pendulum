#pragma once

#include <chrono>
#include <iostream>

// 時間計測用
std::chrono::system_clock::time_point start, end;
std::time_t time_stamp;

void tic() {
    start = std::chrono::system_clock::now(); // 計測開始時間
}
double toc_us() {
    end = std::chrono::system_clock::now(); // 計測終了時間
    auto time = end - start;
    auto usec =
      std::chrono::duration_cast<std::chrono::microseconds>(time).count();
    return usec;
}



//int rightNumber = 0;
//for (int i = (int)Js.size() - 1; i >= 0; i--) {
//    if (i == (int)Js.size() - 1) {
//        break;
//    }
//    if (Js[i] > Js[i+1]) {
//        rightNumber = i + 1;
//    }
//}
