// Inverted pendulum test

#include "InvPend.h"
#include "Timer.h"
#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <random>

double randomizedMPC(const InvertedPendulum &pendu_t);
double get_urand();


int main() {
    const double dT = 0.0001;
    const double g = 9.8;
    const double m = 2.0;
    const double M = 10.0;
    const double L = 1.00;
    const double initialPos = 0.0;
    const double initialAngle = 0.1;

    InvertedPendulum initPendu(dT, L, g, m, M, initialPos, initialAngle, 0.0, 0.0);
    const int simN = 50000; // simulation steps

    // ******* original speed *********
    std::cout << "Original speed..." << std::endl;
    InvertedPendulum orgPend(initPendu);
    tic(); // 計測開始

    double inputF1 = -3.0;
    int timeLimit = 1 / dT;
    for (int i = 0; i < simN; i++)  {
        if (i == timeLimit - 1) {
            inputF1 = 0.0;
        }
        orgPend.UpdateState(inputF1);
        //orgPend.ShowState(dT*i); // ここのコメントアウトを消して状態出力
    }
    //orgPend.ShowState(dT*simN); // ここのコメントアウトを消して状態出力

    // 計測終了
    std::cout << "comp time : " << toc_us() << "us" << std::endl;
    std::cout << std::endl;

    // ******* improved speed *********
    std::cout << "Improved speed..." << std::endl;
    InvertedPendulum impPend(initPendu);
    tic(); // 計測開始


    double inputF2 = -3.0;
    timeLimit = 1 / dT;
    for (int i = 0; i < simN; i++)  {
        if (i == timeLimit - 1) {
            inputF2 = 0.0;
        }
        impPend.UpdateStateFast(inputF2);
        //impPend.ShowState(dT*i); // ここのコメントアウトを消して状態出力
    }
    //impPend.ShowState(dT*simN); // ここのコメントアウトを消して状態出力

    // 計測終了
    std::cout << "comp time : " << toc_us() << "us" << std::endl;
    std::cout << std::endl;

    // ******* rmpc *********
    std::cout << "Rmpc..." << std::endl;
    InvertedPendulum rmpcPend(0.01, L, g, m, M, initialPos, initialAngle, 0, 0);
    tic(); // 計測開始

    double inputF3 = 0.0;
    int counter = 0;
    for (int i = 0; i < simN; i++) {
        inputF3 = randomizedMPC(rmpcPend);
        rmpcPend.UpdateStateFast(inputF3);
        //rmpcPend.ShowState(rmpcPend.getdT()*i); // ここのコメントアウトを消して状態出力
        if (rmpcPend.getAngle() > -0.01 && rmpcPend.getAngle() < 0.01 && rmpcPend.getDerivativeAngle() > -0.01 && rmpcPend.getDerivativeAngle() < 0.01) { // this is for checking stability
            counter++;
        } else {
            counter = 0;
        }
        if (counter > (int)(3.0/rmpcPend.getdT())) { // this is for checking stability (3秒間要件値を満たしたときsuccessを出力)
            std::cout << "succcess" << std::endl;
            counter = 0;
        }
//        if (counter == 0) {
//            std::cout << "failed" << std::endl;
//        } else {
//            std::cout << "success" << std::endl;
//            c++;
//        }
    }
    //rmpcPend.ShowState(rmpcPend.getdT()*simN); // ここのコメントアウトを消して状態出力
//    std::cout << "success : " << c << std::endl;
//    std::cout << "failed : " << abs(simN - c) << std::endl;
    // 計測終了
    std::cout << "comp time:" << toc_us() << "us" << std::endl;
    std::cout << std::endl;

}

// Random from -1 to 1
double get_urand() {
    return (2* (( rand() / (double)RAND_MAX )) -1);
}


double randomizedMPC(const InvertedPendulum &pendu_t) {
    int Ns = 100; // Ns of input series
    std::vector<double> Js;  // storage for J
    std::vector<double> Fs;  // storage for u(0)
    Js.reserve(Ns);
    Fs.reserve(Ns);
    int N = 5;
    double currentMin = 1e+100;
    // Ns of series are sampled
    int rightNumber = 0;
    for (int i = 0; i < Ns; i++) {
        InvertedPendulum sim(pendu_t);  // copy pendulum for simulation of i th input series
        // simulation in RMPC process
        double u = 14.01*get_urand(); // replitでは14.55
        Fs.push_back(u);
        Js.push_back(0.0);
        for (int j = 0; j < N; j++) {
            sim.UpdateStateFast(u);
            u = 14.01*get_urand();
            Js[i] += (0.0 - sim.getAngle())*(0.0 - sim.getAngle());
        }
        // find minimum J
        if (currentMin > Js[i]) {
            currentMin = Js[i];
            rightNumber = i;
        }
    }
    // return its F (minimum J)
    double opt_F = 0.0;
    opt_F = Fs[rightNumber];
    return opt_F;
}
