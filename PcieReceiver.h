#ifndef PCIERECEIVER_H
#define PCIERECEIVER_H

#include "DataReadRule.h"
#include "ThreadSafeQueue.h"
#include "doamseestimatoroptimized.h"
#include "dbscan.h"
//#include "PDW_STRUCT.h"
#include "UAVIdentifier.h"
#include <vector>
#include <string>
#include <functional>
#include <thread>
#include <map>
#include <atomic>
#include <stdint.h>


class PcieReceiver {
public:
    using DataCallback = std::function<void(int interruptId, const std::string& label, const std::vector<char>& data)>;

    PcieReceiver(const std::vector<std::string>& eventDevs,
        const std::string& c2hDev);

    ~PcieReceiver();

    void setReadRules(const std::map<int, DataReadRule>& rules); // 设置中断ID→读取规则映射
    void setDataCallback(DataCallback cb);                       // 设置数据处理回调

    int init_xdma_user();                                       //初始化xdma的user空间

    void reset_fpga(int x);                                     //复位与解复位FPGA

    int get_data_length(int i);                                      //获取数据长度

    std::vector<char> extractFirstFrame(const std::vector<char>& buffer);// 提取第一个帧头到第二个帧头之间的数据

    

    void start();
    void stop();

    ThreadSafeQueue<std::vector<char>> queue0, queue1;

//     UAVIdentifier uav_shibie;                                       //创建识别器
// 
//     std::vector<pdw_data_g> pdw_kjc;

private:
    std::vector<std::string> eventDevs;
    std::string c2hDev;
    size_t mmapSize;

    std::map<int, DataReadRule> readRules;
    DataCallback dataCallback;

    std::vector<std::thread> threads;
    std::atomic<bool> running;

    void* mmapPtr = nullptr;
    int c2hFd = -1;
    int c2hFd_1 = -1;

    volatile uint32_t* mmap_buff_user = nullptr;

    UAVIdentifier uav_shibie;       //创型号建识别器

    DBSCAN  dbscan_julei;           //创建无人机PDW数据聚类器

    void handleInterrupt(int interruptId, const std::string& eventDev);

    void processQueue_AD(ThreadSafeQueue<std::vector<char>>& queue);

    void processQueue_PDW(ThreadSafeQueue<std::vector<char>>& queue);
};

#endif // PCIERECEIVER_H
