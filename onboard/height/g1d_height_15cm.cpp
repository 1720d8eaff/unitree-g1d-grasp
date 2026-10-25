#include <unitree/robot/g1/agv/g1_agv_client.hpp>
#include <iostream>
#include <unistd.h>
#include <signal.h>

using namespace std;

static bool running = true;

void signalHandler(int)
{
    running = false;
}

int main()
{
    signal(SIGINT, signalHandler);

    unitree::robot::ChannelFactory::Instance()->Init(0);

    unitree::robot::g1::AgvClient ac;
    ac.SetTimeout(3.0f);
    ac.Init();

    cout << "开始上升3秒，然后自动停止" << endl;

    // ========================
    // 第一阶段：上升3秒
    // ========================
    float up_cmd = 1.0f;   // 最大高度目标
    int hz = 200;
    int total = 1.5 * hz;

    for(int i=0;i<total && running;i++)
    {
        ac.HeightAdjust(up_cmd);
        usleep(1000000 / hz);
    }

    // ========================
    // 第二阶段：发送停止命令
    // ========================
    cout << "时间到，发送停止命令" << endl;

    float stop_cmd = 0.0f;

    for(int i=0;i<20;i++)
    {
        ac.HeightAdjust(stop_cmd);
        usleep(20000);
    }

    cout << "Height adjustment finished." << endl;

    return 0;
}
