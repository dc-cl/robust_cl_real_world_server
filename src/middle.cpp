#include <ros/ros.h>
#include <ros/rate.h> // Add this line
#include "nlink_parser/LinktrackNodeframe2.h"
#include "cla_monitor/position.h"
#include <vector>
#include <queue>
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
        // ROS_INFO("Callback0: %f %f %f", pos[0], pos[1], pos[2]);
        std::lock_guard<std::mutex> lock(mutex0_);
        q_ts0_.emplace(msg->system_time);
        q_pos0_x_.emplace(pos[0]);
        q_pos0_y_.emplace(pos[1]);
    }

    void callback2(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg) {
        auto pos = msg->pos_3d;
        std::lock_guard<std::mutex> lock(mutex2_);
        q_ts2_.emplace(msg->system_time);
        q_pos2_x_.emplace(pos[0]);
        q_pos2_y_.emplace(pos[1]);
    }

    // void interp1(uint& ts_fir_0, uint& ts_fir_1, uint& ts_0, float& pos_fir_x_0, float& pos_fir_y_0, float& pos_fir_x_1, float& pos_fir_y_1, float& pos_0_x, float& pos_0_y) {
    //     if (abs(ts_fir_0 - ts_0) < epsilon) {
    //         pos_0_x = pos_fir_x_0;
    //         pos_0_y = pos_fir_y_0;
    //     }
    //     else if (abs(ts_fir_1 - ts_0) < epsilon) {
    //         pos_0_x = pos_fir_x_1;
    //         pos_0_y = pos_fir_y_1;
    //     }
    //     else {
    //         uint t_up = ts_0 - ts_fir_0;
    //         uint t_down = ts_fir_1 - ts_fir_0;
    //         pos_0_x = pos_fir_x_0 + t_up * (pos_fir_x_1 - pos_fir_x_0) / t_down;
    //         pos_0_y = pos_fir_y_0 + t_up * (pos_fir_y_1 - pos_fir_y_0) / t_down;
    //     }
    // }

    void process() {
        std::lock_guard<std::mutex> lock0(mutex0_);
        std::lock_guard<std::mutex> lock2(mutex2_);
        if (fir < 0 && !q_ts0_.empty() && !q_ts2_.empty()) {
            if(q_ts0_.front() < q_ts2_.front()) fir = 0;
            else if(q_ts0_.front() > q_ts2_.front()) fir = 2;
        }
        if (q_ts0_.empty() || q_ts2_.empty()) return; // No valid data to process
        float pos0[2], pos2[2];
        if (!fir && !q_pos0_x_.empty() && !q_pos2_x_.empty()) {
            while(!q_pos0_x_.empty() && q_ts0_.front() < q_ts2_.front()) {
                q_pos0_x_.pop();
                q_pos0_y_.pop();
                q_ts0_.pop();
            }
            if(q_pos0_x_.empty()) return;
        }
        else if (fir == 2 && !q_pos2_x_.empty() && !q_pos0_x_.empty()) {
            while(!q_pos2_x_.empty() && q_ts2_.front() < q_ts0_.front()) {
                q_pos2_x_.pop();
                q_pos2_y_.pop();
                q_ts2_.pop();
            }
            if(q_pos2_x_.empty()) return;
        }
        fir = -1;

        pos0[0] = q_pos0_x_.front();
        pos0[1] = q_pos0_y_.front();
        pos2[0] = q_pos2_x_.front();
        pos2[1] = q_pos2_y_.front();

        q_pos0_x_.pop();
        q_pos0_y_.pop();
        q_ts0_.pop();
        q_pos2_x_.pop();
        q_pos2_y_.pop();
        q_ts2_.pop();

        // if (!fir && !v_pos0_x_.size() > 1 && !v_pos2_x_.empty()) {
        //     while(v_pos0_x_.size() > 1 && v_ts0_[1] < v_ts2_[0]) {
        //         v_pos0_x_.pop();
        //         v_pos0_y_.pop();
        //         v_ts0_.pop();
        //     }
        //     if (v_pos0_x_.size() > 1) {
        //         interp1(v_ts0_[0], v_ts0_[1], v_ts2_[0], v_pos0_x_[0], v_pos0_y_[0], v_pos0_x_[1], v_pos0_y_[1], pos0[0], pos0[1]);
        //         pos2[0] = v_pos2_x_[0];
        //         pos2[1] = v_pos2_y_[0];

        //         v_pos0_x_.pop();
        //         v_pos0_y_.pop();
        //         v_ts0_.pop();
        //         v_pos2_x_.pop();
        //         v_pos2_y_.pop();
        //         v_ts2_.pop();
        //     }
        // }
        // else if (fir == 2 && !v_pos2_x_.size() > 1 && !v_pos0_x_.empty()) {
        //     while(v_pos2_x_.size() > 1 && v_ts2_[1] < v_ts0_[0]) {
        //         v_pos2_x_.pop();
        //         v_pos2_y_.pop();
        //         v_ts2_.pop();
        //     }
        //     if (v_pos2_x_.size() > 1) {
        //         interp1(v_ts2_[0], v_ts2_[1], v_ts0_[0], v_pos2_x_[0], v_pos2_y_[0], v_pos2_x_[1], v_pos2_y_[1], pos2[0], pos2[1]);
        //         pos0[0] = v_pos0_x_[0];
        //         pos0[1] = v_pos0_y_[0];

        //         v_pos0_x_.pop();
        //         v_pos0_y_.pop();
        //         v_ts0_.pop();
        //         v_pos2_x_.pop();
        //         v_pos2_y_.pop();
        //         v_ts2_.pop();
        //     }
        // }
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
    std::queue<float> q_pos0_x_, q_pos2_x_, q_pos0_y_, q_pos2_y_;
    std::queue<uint> q_ts0_, q_ts2_;
    std::mutex mutex0_, mutex2_; // Mutexes for the queues

};

const double epsilon = std::numeric_limits<double>::epsilon();

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "middle");
    ros::NodeHandle nh;
    if(!nh.hasParam("num_robots")) {
        ROS_ERROR("Need to specify the number of robots as an argument.");
        return -1;
    }
    int i_num_robots = 3;
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