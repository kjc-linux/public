#include "PcieReceiver.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <iostream>
#include <cstring>
#include <iomanip>


PcieReceiver::PcieReceiver(const std::vector<std::string>& eventDevs,
    const std::string& c2hDev)
    : eventDevs(eventDevs), c2hDev(c2hDev),  running(false)  , dbscan_julei(5.0, 5) {}

PcieReceiver::~PcieReceiver() {
    stop();
}

void PcieReceiver::setReadRules(const std::map<int, DataReadRule>& rules) {
    readRules = rules;
}

void PcieReceiver::setDataCallback(DataCallback cb) {
    dataCallback = cb;
}

void PcieReceiver::start() {
    running = true;

    // 从JSON文件加载参数
    if (!uav_shibie.loadParametersFromFile("uav_parameters.json")) {
        std::cerr << "无法加载无人机参数，请检查配置文件" << std::endl;
        return ;
    }

    std::cout << "已加载 " << uav_shibie.getParameterCount() << " 种无人机参数" << std::endl;

    
    c2hFd = open(c2hDev.c_str(), O_RDONLY);
    if (c2hFd < 0) {
        perror("Open xdma c2h failed");
        return;
    }

    c2hFd_1 = open("/dev/xdma0_c2h_1", O_RDONLY);
    if (c2hFd_1 < 0) {
        perror("Open xdma c2h_1 failed");
        return;
    }


    int ret = init_xdma_user();
    if (ret == EXIT_FAILURE)
    {
        perror("init error");
        return;
    }

    for (size_t i = 0; i < eventDevs.size(); ++i) {
        threads.emplace_back(&PcieReceiver::handleInterrupt, this, int(i), eventDevs[i]);
    }

     threads.emplace_back(&PcieReceiver::processQueue_AD,this, std::ref(queue0));
 
     threads.emplace_back(&PcieReceiver::processQueue_PDW, this, std::ref(queue1));



}

void PcieReceiver::stop() {
    running = false;

    for (auto& t : threads) {
        if (t.joinable()) t.join();
    }

    threads.clear();

    if (c2hFd >= 0) {
        close(c2hFd);
        c2hFd = -1;
    }
}

void PcieReceiver::handleInterrupt(int interruptId, const std::string& eventDev) {
    int eventFd = open(eventDev.c_str(), O_RDONLY);
    if (eventFd < 0) {
        perror(("Open " + eventDev + " failed").c_str());
        return;
    }

    int ret = 0;

    int length_1 = 0,length_0=0;
    uint32_t dummy = 0;

    while (running) {


            read(eventFd, &dummy, sizeof(dummy)); 

            auto it = readRules.find(interruptId);
            if (it != readRules.end()) {
                const auto& rule = it->second;

                if (rule.label == "INT1_AD_DATA")
                {
                    //0729,考虑对数据的处理，读取之后是否交给另一个线程去处理，在那个线程里进行调制样式识别和型号识别

                    length_1 = get_data_length(0);                       //获取数据长度，从XDMA_user中获取

                    std::vector<char> buffer_1(length_1);

                    ret = read(c2hFd, buffer_1.data(), length_1);
                    if (ret > 0)
                    {
                        queue1.push(buffer_1);
                    }

                }
                if (rule.label == "INT0_PDW")
                {
                    //0901，对于PDW数据的读取
                    length_0 = get_data_length(1);

                    std::vector<char> buffer_0(length_0);

                    ret = read(c2hFd_1, buffer_0.data(), length_0);
                    if (ret > 0)
                    {
                        queue0.push(buffer_0);
                    }

                }

            }
            else {
                std::cerr << "[PCIE] No read rule defined for interrupt: " << interruptId << std::endl;
            }
    }

    close(eventFd);
}


int PcieReceiver::init_xdma_user()
{
    
    int fd_user = open("/dev/xdma0_user", O_RDWR);
    if (fd_user < 0) {
        perror("open failed");
        return EXIT_FAILURE;
    }

    mmap_buff_user = (volatile uint32_t*)mmap(NULL, 1024, PROT_READ | PROT_WRITE, MAP_SHARED, fd_user, 0);
    if (mmap_buff_user == MAP_FAILED) {
        perror("mmap failed");
        close(fd_user);
        return EXIT_FAILURE;
    }

    return 0;

}


void PcieReceiver::reset_fpga(int x)
{

    if (x == 1)
    {
        mmap_buff_user[0] = 0x00000001;
    }
    else if (x == 0)
    {
        mmap_buff_user[0] = 0x00000000;
    }
    else
    {
        return ;
    }

    msync((void*)mmap_buff_user, 4, MS_SYNC);

}


int PcieReceiver::get_data_length(int i)
{
    int length = 0;

    if (i == 0)
    {
        length = mmap_buff_user[8];
    }
    if (i == 1)
    {
        length = mmap_buff_user[7];
    }


    return length;
}


// 提取第一个帧头到第二个帧头之间的数据
std::vector<char> PcieReceiver::extractFirstFrame(const std::vector<char>& buffer) {
    const uint8_t FRAME_HEADER[4] = { 0xF0, 0x7F, 0x01, 0x80 };
    const size_t HEADER_SIZE = sizeof(FRAME_HEADER);

    std::vector<size_t> headerPositions;

    // ---- Step 1: 查找所有帧头位置 ----
    for (size_t i = 0; i + HEADER_SIZE <= buffer.size(); ++i) {
        if (static_cast<uint8_t>(buffer[i]) == FRAME_HEADER[0] &&
            static_cast<uint8_t>(buffer[i + 1]) == FRAME_HEADER[1] &&
            static_cast<uint8_t>(buffer[i + 2]) == FRAME_HEADER[2] &&
            static_cast<uint8_t>(buffer[i + 3]) == FRAME_HEADER[3]) {
            headerPositions.push_back(i);
            if (headerPositions.size() == 2) break; // 找到前两个帧头就够了
        }
    }

    // ---- Step 2: 没找到两个帧头的情况 ----
    if (headerPositions.size() < 2) {
        std::cerr << "Warning: 帧头数量不足两个，无法提取完整帧\n";
        return {};
    }

    // ---- Step 3: 提取第一个帧头到第二个帧头之间的数据 ----
    size_t start = headerPositions[0];
    size_t end = headerPositions[1];
    std::vector<char> frame(buffer.begin() + start, buffer.begin() + end);
    return frame;
}



void PcieReceiver::processQueue_AD(ThreadSafeQueue<std::vector<char>>& queue) {
    size_t length = 0;
    while (running) {
        std::vector<char> data = queue.wait_and_pop();
        length = data.size();
        //补充对数据的处理 此处为对AD中频数据进行调制样式识别




    }
}

void PcieReceiver::processQueue_PDW(ThreadSafeQueue<std::vector<char>>& queue) {

    DOAMSEEstimator doasmse;

    std::vector<std::vector<pdw_data_g>> pdw_clu;

    double fc_p = 0;
    std::vector<double> phy_p(3);
    while (running) {
        std::vector<char> data = queue.wait_and_pop();

        pdw_clu.clear();

        //补充对数据的处理 此处为对PDW数据进行无人机型号识别

        //另一种处理方式，不拷贝，更快但是有危险
        //std::vector<pdw_data_g>& pdw_kjc = reinterpret_cast<std::vector<pdw_data_g>&>(data);
        //int length_t = pdw_kjc.size();

        // 检查数据大小是否合法
        if (data.size() % sizeof(pdw_data_g) != 0) {
            // 处理错误：数据大小不是结构体的整数倍
            throw std::runtime_error("Invalid data size");
        }

        // 计算元素数量
        size_t num_elements = data.size() / sizeof(pdw_data_g);

        // 将字节数据复制到 pdw_data_g 向量中
        std::vector<pdw_data_g> pdw_kjc(num_elements);
        memcpy(pdw_kjc.data(), data.data(), data.size());

        // 现在可以安全使用 pdw_kjc
        size_t length_t = pdw_kjc.size();  // 注意：size() 是成员函数，需要括号！

        //计算来波方向DOA
        for (size_t i_p = 0; i_p < length_t; i_p++)
        {

            fc_p = pdw_kjc[i_p].freq * 146484.375;    //中心频率的单位换算

            phy_p[0] = pdw_kjc[i_p].Phase1 * (9.58738e-5);
            phy_p[1] = pdw_kjc[i_p].Phase2 * (9.58738e-5);
            phy_p[2] = pdw_kjc[i_p].Phase3 * (9.58738e-5);         //相位差的单位换算

            pdw_kjc[i_p].doa = doasmse.estimateDOA_MSE(fc_p, phy_p);    //计算每个PDW脉冲的doa

        }

        pdw_clu = dbscan_julei.fit(pdw_kjc);

        for (int x = 0; x < pdw_clu.size(); x++)
        {

            // 执行无人机型号识别
            std::string detectedType;
            if (uav_shibie.match(pdw_clu[x], detectedType)) {
                std::cout << "识别到无人机: " << detectedType << std::endl;
            }
            else {
                std::cout << "未识别到任何无人机" << std::endl;
            }
        }


    }
}

