#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include <cstdlib>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <simple_camera_transformer/TransformerConfig.h>
#include <vector>

// Save the config state
simple_camera_transformer::TransformerConfig config_state;

// Publisher
image_transport::Publisher pub;

dynamic_reconfigure::Server<simple_camera_transformer::TransformerConfig> *server = nullptr;

void configCallback(simple_camera_transformer::TransformerConfig& config_, uint32_t level)
{
    config_state = config_;
}

void applyTransform(cv::Mat& frame, const simple_camera_transformer::TransformerConfig& config)
{
    // Resize
    cv::resize(frame, frame, cv::Size(), config.resize, config.resize, cv::INTER_AREA);

    // Color correction (brightness & contrast)
    if(config.enable_color_correct)
    {
        frame = frame*config.cc_alpha + config.cc_beta;
    }

    // Sharpen
    if(config.enable_sharpen)
    {
        cv::Mat out;
        cv::GaussianBlur(frame, out, cv::Size(0, 0), config.sharp_kernel*2+1);
        cv::addWeighted(frame, 1.0+config.sharp_weight, out, -1.0*config.sharp_weight, 0, out);
        out.copyTo(frame);
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr &input)
{
    // Get the OpenCV image from the ROS message
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        // Something went wrong
        ROS_ERROR("cv_bridge exceptions: %s", e.what());
        return;
    }

    sensor_msgs::ImagePtr msg;
    cv::Mat frame = cv_ptr->image;

    // Apply transformations
    applyTransform(frame, config_state);

    // Convert it back to a message and publish
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    if(pub.getNumSubscribers() > 0)
    {
        pub.publish(msg);
    }

    cv::waitKey(3);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cam_transformer");

    ros::NodeHandle nh{"~"};
    image_transport::ImageTransport it(nh);

    dynamic_reconfigure::Server<simple_camera_transformer::TransformerConfig> server_;

    server = &server_;

    std::string pub_topic;
    std::string sub_topic;
    if (!nh.getParam("pub_topic", pub_topic) || !nh.getParam("sub_topic", sub_topic))
    {
        ROS_ERROR_STREAM("Missing 'pub_topic' or 'sub_topic' rosparams");
        return 0;
    }

    pub = it.advertise(pub_topic, 1);
    image_transport::Subscriber img_input = it.subscribe(sub_topic, 1, &imageCallback, 0);

    server_.setCallback(boost::bind(&configCallback, _1, _2));
    server_.getConfigDefault(config_state);

    ros::spin();
    }
