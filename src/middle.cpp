#include <ros/ros.h>
#include <ros/rate.h> // Add this line
#include "nlink_parser/LinktrackNodeframe2.h"
#include "cla_monitor/position.h"
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

class TopicPairProcessor {
public:
    TopicPairProcessor(ros::NodeHandle& nh, const std::string& topic0, const std::string& topic2, int id)
    : fir(-1), id_(id), pub_(nh.advertise<cla_monitor::position>("/real_pos_" + std::to_string(id_), 1000)) {
        sub0_ = nh.subscribe(topic0, 1000, &TopicPairProcessor::callback0, this);
        sub2_ = nh.subscribe(topic2, 1000, &TopicPairProcessor::callback2, this);
    }

    void callback0(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg) {
        auto pos = msg->pos_3d;
        std::lock_guard<std::mutex> lock(mutex0_);
        v_ts0_.emplace_back(msg->system_time);
        v_pos0_x_.emplace_back(pos[0]);
        v_pos0_y_.emplace_back(pos[1]);
    }

    void callback2(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg) {
        auto pos = msg->pos_3d;
        std::lock_guard<std::mutex> lock(mutex2_);
        v_ts2_.emplace_back(msg->system_time);
        v_pos2_x_.emplace_back(pos[0]);
        v_pos2_y_.emplace_back(pos[1]);
    }

    void interp1(uint& ts_fir_0, uint& ts_fir_1, uint& ts_0, float& pos_fir_x_0, float& pos_fir_y_0, float& pos_fir_x_1, float& pos_fir_y_1, float& pos_0_x, float& pos_0_y) {
        uint t_up = ts_0 - ts_fir_0;
        uint t_down = ts_fir_1 - ts_fir_0;
        pos_0_x = pos_fir_x_0 + t_up * (pos_fir_x_1 - pos_fir_x_0) / t_down;
        pos_0_y = pos_fir_y_0 + t_up * (pos_fir_y_1 - pos_fir_y_0) / t_down;
    }

    void process() {
        std::lock_guard<std::mutex> lock0(mutex0_);
        std::lock_guard<std::mutex> lock2(mutex2_);
        if (fir < 0 && !v_ts0_.empty() && !v_ts2_.empty()) {
            if(v_ts0_[0] < v_ts2_[0]) fir = 0;
            else fir = 2;
        }
        float pos0[2], pos2[2];
        if (!fir && !v_pos0_x_.size() > 1 && !v_pos2_x_.empty()) {
            while(!v_pos0_x_.size() > 1 && v_ts0_[1] < v_ts2_[0]) {
                v_pos0_x_.pop_back();
                v_pos0_y_.pop_back();
                v_ts0_.pop_back();
            }
            if (v_pos0_x_.size() > 1) {
                interp1(v_ts0_[0], v_ts0_[1], v_ts2_[0], v_pos0_x_[0], v_pos0_y_[0], v_pos0_x_[1], v_pos0_y_[1], pos0[0], pos0[1]);
                pos2[0] = v_pos2_x_[0];
                pos2[1] = v_pos2_y_[0];

                v_pos0_x_.pop_back();
                v_pos0_y_.pop_back();
                v_ts0_.pop_back();
                v_pos2_x_.pop_back();
                v_pos2_y_.pop_back();
                v_ts2_.pop_back();
            }
        }
        else if (fir == 2 && !v_pos2_x_.size() > 1 && !v_pos0_x_.empty()) {
            while(!v_pos2_x_.size() > 1 && v_ts2_[1] < v_ts0_[0]) {
                v_pos2_x_.pop_back();
                v_pos2_y_.pop_back();
                v_ts2_.pop_back();
            }
            if (v_pos2_x_.size() > 1) {
                interp1(v_ts2_[0], v_ts2_[1], v_ts0_[0], v_pos2_x_[0], v_pos2_y_[0], v_pos2_x_[1], v_pos2_y_[1], pos2[0], pos2[1]);
                pos0[0] = v_pos0_x_[0];
                pos0[1] = v_pos0_y_[0];

                v_pos0_x_.pop_back();
                v_pos0_y_.pop_back();
                v_ts0_.pop_back();
                v_pos2_x_.pop_back();
                v_pos2_y_.pop_back();
                v_ts2_.pop_back();
            }
        }
        cla_monitor::position msg;
        // TODO not just simply in the **middle**
        msg.pos_3d = {(pos0[0] + pos2[0]) / 2, // X
            (pos0[1] + pos2[1]) / 2, // Y
            atan2(pos2[1] - pos0[1], pos2[0] - pos0[0])}; // theta
        pub_.publish(msg);
    }

private:
    int id_;
    int fir; // Judge which uwb sensor comes data first
    ros::Subscriber sub0_, sub2_;
    ros::Publisher pub_;
    std::vector<float> v_pos0_x_, v_pos2_x_, v_pos0_y_, v_pos2_y_;
    std::vector<uint> v_ts0_, v_ts2_;
    std::mutex mutex0_, mutex2_; // Mutexes for the vectors

};

const double epsilon = std::numeric_limits<double>::epsilon();

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "middle");
    ros::NodeHandle nh;
    if(!nh.hasParam("num_robots")) {
        ROS_ERROR("Need to specify the number of robots as an argument.");
        return -1;
    }
    int i_num_robots;
    nh.getParam("num_robots", i_num_robots);

    const std::string topic_base = "/nlink_linktrack_nodeframe2_";
    // Create instances for each pair of topics
    std::vector<std::unique_ptr<TopicPairProcessor>> processors;
    for(int id = 0; id < i_num_robots; id++) {
        int number = 3 * id;
        processors.push_back(std::make_unique<TopicPairProcessor>(nh, topic_base + std::to_string(number), 
            topic_base + std::to_string(number + 2), id));
    }

    // Create a new thread to execute the process() function
    std::thread process_thread([&]() {
        ros::Rate rate(50); // Adjust the rate as per your requirement
        while (ros::ok()) {
            for (auto& processor : processors) {
                processor->process();
            }
            rate.sleep();
        }
    });

    ros::spin();

    // Wait for the process thread to finish
    process_thread.join();

    return 0;
}