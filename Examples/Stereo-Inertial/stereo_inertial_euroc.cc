/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
        vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

int main(int argc, char **argv)
{
// Step1. 参数数量检查
    if (argc < 5)
    {
        cerr << endl
             << "Usage: "
             << "./stereo_inertial_euroc "   // 运行的示例程序，在这里就是：ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc
             << "path_to_vocabulary "        // 字典路径，可以使用自己训练的字典，自带的字典路径：ORB_SLAM3/Vocabulary/ORBvoc.txt
             << "path_to_settings "          // 配置文件路径，在这里就是：ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml
             << "path_to_sequence_folder_1 " // 数据集路径，例如：MH_01_easy/、MH_02_easy/ 等
             << "path_to_times_file_1 "      // 时间戳文件路径，例如：ORB_SLAM3/Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt
             // 可以一次性顺序运行多个数据集
             << "(path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) "
             << endl;
        return 1;
    }

    // argc - 3 剩下的就是数据集数量的 2 倍（一个数据集路径，一个是时间戳文件路径）
    const int num_seq = (argc - 3) / 2;
    cout << "num_seq = " << num_seq << endl;
    // 参数说明中没有写，其实可以在参数列表最后面加上一个字符串，指定输出的轨迹文件的文件名（后缀）
    bool bFileName = (((argc - 3) % 2) == 1); // 检查用户是否指定输出的轨迹文件的文件名（后缀）
    string file_name; // 这个对象就在下面的 if 中用了一下
    if (bFileName)
    {
        file_name = string(argv[argc - 1]);
        cout << "file name: " << file_name << endl;
    }

// Step2. 读取所有数据集的图像帧和 IMU 测量数据
    // Load all sequences:
    int seq; // 这个变量就是为了下面的两个 for 循环，没其他用处
    vector<vector<string> > vstrImageLeft;    // 第一层 vector 保存所有数据集，第二层保存一个数据集中所有左图文件路径
    vector<vector<string> > vstrImageRight;   // 第一层 vector 保存所有数据集，第二层保存一个数据集中所有右图文件路径
    vector<vector<double> > vTimestampsCam;   // 相机时间戳，来自时间戳文件
    vector<vector<cv::Point3f> > vAcc, vGyro; // IMU 测量数据
    vector<vector<double> > vTimestampsImu;   // IMU 时间戳，来自数据集
    vector<int> nImages;                      // 每个数据集中的图像帧数量（左右图像是一帧）
    vector<int> nImu;                         // 每个数据集中的 IMU 测量数据数量
    vector<int> first_imu(num_seq, 0);        // 第一个符合图像帧时间戳的 IMU 测量数据在本数据集 vector 的索引 vTimestampsImu[seq]

    vstrImageLeft.resize(num_seq);
    vstrImageRight.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0; // 图像帧总数
    for (seq = 0; seq < num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        // 准备路径
        string pathSeq(argv[(2 * seq) + 3]);
        string pathTimeStamps(argv[(2 * seq) + 4]);
        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathCam1 = pathSeq + "/mav0/cam1/data";
        string pathImu = pathSeq + "/mav0/imu0/data.csv";

        // 加载图像帧路径
        LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        // 加载 IMU 测量数据
        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if ((nImages[seq] <= 0) || (nImu[seq] <= 0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // 假设 IMU 测量数据比图像帧来的早，寻找第一个符合图像帧时间戳的 IMU 测量数据
        // Find first imu to be considered, supposing imu measurements start first
        while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered
        // first_imu[seq] 是本数据集中第一个时间戳上大于等于本数据集图像帧时间戳的 IMU 测量数据在 vTimestampsImu[seq] 中的索引
    } // for 加载所有数据集

// Step3. 读取配置文件
    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack; // 为了统计追踪用时
    vTimesTrack.resize(tot_images); // 这里开的空间太大了，只需要开辟所有数据集中最多的图像帧数量就可以

    cout << endl
         << "-------" << endl;
    cout.precision(17); // 设置输出值的有效位数为 17 位

// Step4. 启动 SLAM 系统
    // 创建 SLAM 系统对象（初始化所有系统线程并准备好处理图像帧和 IMU 测量数据）
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(
        argv[1],                       // 配置文件路径
        argv[2],                       // 字典路径
        ORB_SLAM3::System::IMU_STEREO, // 输入传感器类型
        // false                       // 是否开启可视化
        true
    );

    cv::Mat imLeft, imRight;
    for (seq = 0; seq < num_seq; seq++)
    {
        // Seq loop
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        double t_rect = 0.f;
        double t_resize = 0.f;
        double t_track = 0.f;
        // 下面两个对象没用到
        // int num_rect = 0;
        // int proccIm = 0;
        // for (int ni = 0; ni < nImages[seq]; ni++, proccIm++)
        for (int ni = 0; ni < nImages[seq]; ni++)
        {
            // Read left and right images from file
            imLeft = cv::imread(vstrImageLeft[seq][ni], cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[seq][ni], cv::IMREAD_UNCHANGED);

            // if (imLeft.empty()) // 合到一起看着舒服
            // {
            //     cerr << endl
            //          << "Failed to load image at: "
            //          << string(vstrImageLeft[seq][ni]) << endl;
            //     return 1;
            // }

            // if (imRight.empty())
            if (imRight.empty() || imLeft.empty())
            {
                cerr << endl
                     << "Failed to load image at: "
                     << string(vstrImageRight[seq][ni]) << endl;
                return 1;
            }

            double tframe = vTimestampsCam[seq][ni];

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if (ni > 0)
            {
                while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][ni]) // while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
                {
                    vImuMeas.push_back(
                        ORB_SLAM3::IMU::Point(
                            vAcc[seq][first_imu[seq]].x,
                            vAcc[seq][first_imu[seq]].y,
                            vAcc[seq][first_imu[seq]].z,
                            vGyro[seq][first_imu[seq]].x,
                            vGyro[seq][first_imu[seq]].y,
                            vGyro[seq][first_imu[seq]].z,
                            vTimestampsImu[seq][first_imu[seq]]
                        )
                    );
                    first_imu[seq]++;
                }
            }

// #ifdef COMPILEDWITHC11 // 编译器是否支持 C++11 标准，C++11 标准用 steady_clock 代替了 monotonic_clock，这里是为了兼容
//             std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
// #else
//             std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
// #endif

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            // Pass the images to the SLAM system
            SLAM.TrackStereo(imLeft, imRight, tframe, vImuMeas);
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

// #ifdef COMPILEDWITHC11
//             std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
// #else
//             std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
// #endif

// #ifdef REGISTER_TIMES // 作者说是为了统计执行的统计时间，但作者未完成，该功能用不了
//             t_track = t_rect + t_resize + std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
//             SLAM.InsertTrackTime(t_track);
// #endif

            // ttrack 是为了统计里程计每次追踪所需时间
            double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            vTimesTrack[ni] = ttrack;

            // 这里就是为了尽可能按照真实的时间戳向 SLAM 系统输入（Wait to load the next frame）
            double T = 0;
            if (ni < nImages[seq] - 1)
                T = vTimestampsCam[seq][ni + 1] - tframe; // 下一图像帧时间戳 - 当前帧时间戳
            else if (ni > 0) // 最后一次处理后为什么还要睡眠？感觉最后一次睡眠没意义
                T = tframe - vTimestampsCam[seq][ni - 1]; // 当前帧时间戳 - 上一针时间戳

            if (ttrack < T)
                // usleep(微秒为单位) 1(s) = 1e6(μs)
                usleep((T - ttrack) * 1e6); // 1e6
        } // for 处理本数据集

        if (seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;
            SLAM.ChangeDataset();
        }
    } // for 顺序处理每个数据集


// Step 5. SLAM 系统关机并保存轨迹
    // Stop all threads
    SLAM.Shutdown();

    // 保存轨迹，用户可指定文件名后缀（Save camera trajectory）
    if (bFileName)
    {
        const string kf_file = "kf_" + string(argv[argc - 1]) + ".txt";
        const string f_file = "f_" + string(argv[argc - 1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t / 1e9);
        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while (!fImu.eof())
    {
        string s;
        getline(fImu, s);

        // 第一行不要
        // #timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
        if (s[0] == '#')
            continue;

        if (!s.empty())
        {
            string item;
            size_t pos = 0;
            // 1403636579758555392,-0.099134701513277898,0.14730578886832138,0.02722713633111154,8.1476917083333333,-0.37592158333333331,-2.4026292499999999
            double data[7];
            int count = 0;
            // string::npos 等于 string::size_type 的最大值，用来表示一个不存在的地址
            // typedef std::size_t std::string::size_type
            while ((pos = s.find(',')) != string::npos)
            {
                item = s.substr(0, pos);    // substr(起始位置, 字符数量)
                data[count++] = stod(item); // string -> double
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0] / 1e9);
            vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
            vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
        }
    } // while
} // LoadIMU
