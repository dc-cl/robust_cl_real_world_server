#include <ros/ros.h>
#include "nlink_parser/LinktrackNodeframe2.h"
#include "cla_monitor/position.h"
#include <vector>
#include <string>
#include <cmath>
#include <thread>

ros::Publisher pub;
std::vector<float> v_pos0_x, v_pos2_x, v_pos0_y, v_pos2_y;
std::vector<uint> v_ts0, v_ts2;
std::mutex mutex0, mutex2;
int fir = -1;

void callback0(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg) {
    auto pos = msg->pos_3d;
    std::lock_guard<std::mutex> lock(mutex0);
    v_ts0.push_back(msg->system_time);
    v_pos0_x.push_back(pos[0]);
    v_pos0_y.push_back(pos[1]);
}

void callback2(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg) {
    auto pos = msg->pos_3d;
    std::lock_guard<std::mutex> lock(mutex2);
    v_ts2.push_back(msg->system_time);
    v_pos2_x.push_back(pos[0]);
    v_pos2_y.push_back(pos[1]);
}

void interp1(uint& ts_fir_0, uint& ts_fir_1, uint& ts_0, float& pos_fir_x_0, float& pos_fir_y_0, float& pos_fir_x_1, float& pos_fir_y_1, float& pos_0_x, float& pos_0_y) {
    uint t_up = ts_0 - ts_fir_0;
    uint t_down = ts_fir_1 - ts_fir_0;
    pos_0_x = pos_fir_x_0 + t_up * (pos_fir_x_1 - pos_fir_x_0) / t_down;
    pos_0_y = pos_fir_y_0 + t_up * (pos_fir_y_1 - pos_fir_y_0) / t_down;
}

void process() {
    std::lock_guard<std::mutex> lock0(mutex0);
    std::lock_guard<std::mutex> lock2(mutex2);
    if (fir < 0 && !v_ts0.empty() && !v_ts2.empty()) {
        if (v_ts0[0] < v_ts2[0])
            fir = 0;
        else
            fir = 2;
    }
    float pos0[2], pos2[2];
    if (!fir && v_pos0_x.size() > 1 && !v_pos2_x.empty()) {
        while (v_pos0_x.size() > 1 && v_ts0[1] < v_ts2[0]) {
            v_pos0_x.pop_back();
            v_pos0_y.pop_back();
            v_ts0.pop_back();
        }
        if (v_pos0_x.size() > 1) {
            interp1(v_ts0[0], v_ts0[1], v_ts2[0], v_pos0_x[0], v_pos0_y[0], v_pos0_x[1], v_pos0_y[1], pos0[0], pos0[1]);
            pos2[0] = v_pos2_x[0];
            pos2[1] = v_pos2_y[0];

            v_pos0_x.pop_back();
            v_pos0_y.pop_back();
            v_ts0.pop_back();
            v_pos2_x.pop_back();
            v_pos2_y.pop_back();
            v_ts2.pop_back();
        }
    } else if (fir == 2 && v_pos2_x.size() > 1 && !v_pos0_x.empty()) {
        while (v_pos2_x.size() > 1 && v_ts2[1] < v_ts0[0]) {
            v_pos2_x.pop_back();
            v_pos2_y.pop_back();
            v_ts2.pop_back();
        }
        if (v_pos2_x.size() > 1) {
            interp1(v_ts2[0], v_ts2[1], v_ts0[0], v_pos2_x[0], v_pos2_y[0], v_pos2_x[1], v_pos2_y[1], pos2[0], pos2[1]);
            pos0[0] = v_pos0_x[0];
            pos0[1] = v_pos0_y[0];

            v_pos0_x.pop_back();
            v_pos0_y.pop_back();
            v_ts0.pop_back();
            v_pos2_x.pop_back();
            v_pos2_y.pop_back();
            v_ts2.pop_back();
        }
    }
    cla_monitor::position msg;
    // TODO not just simply in the **middle**
    msg.pos_3d = {(pos0[0] + pos2[0]) / 2, // X
                  (pos0[1] + pos2[1]) / 2, // Y
                  atan2(pos2[1] - pos0[1], pos2[0] - pos0[0])}; // theta
    pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "middle");
    ros::NodeHandle nh;

    const std::string topic_base = "/nlink_linktrack_nodeframe2_";
    nh.subscribe(topic_base + "0", 1000, callback0);
    nh.subscribe(topic_base + "2", 1000, callback2);

    pub = nh.advertise<cla_monitor::position>("/real_pos", 1000);

    // Create a new thread to execute the process() function
    std::thread process_thread([&]() {
        ros::Rate rate(50); // Adjust the rate as per your requirement
        while (ros::ok()) {
            process();
            rate.sleep();
        }
    });

    ros::spin();

    // Wait for the process thread to finish
    process_thread.join();

    return 0;
}
