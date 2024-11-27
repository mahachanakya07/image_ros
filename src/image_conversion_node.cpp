#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageConversionNode : public rclcpp::Node
{
public:
    ImageConversionNode() : Node("image_conversion_node"), mode_(1)
    {
        this->declare_parameter<std::string>("input_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("output_topic", "/image_converted");

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();

        image_sub_ = image_transport::create_subscription(
            this, input_topic_,
            std::bind(&ImageConversionNode::imageCallback, this, std::placeholders::_1),
            "raw");

        image_pub_ = image_transport::create_publisher(this, output_topic_);

        service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_mode", std::bind(&ImageConversionNode::setMode, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        try
        {
            // Log the image encoding to diagnose potential mismatches
            RCLCPP_INFO(this->get_logger(), "Image encoding: %s", msg->encoding.c_str());

            // Use the actual encoding from the message for conversion
            cv::Mat image = cv_bridge::toCvCopy(msg, msg->encoding)->image;

            // Check the mode and apply conversion or pass through
            if (mode_ == 1)
            {
                RCLCPP_INFO(this->get_logger(), "Converting image to grayscale");
                cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
                sensor_msgs::msg::Image::SharedPtr gray_msg = cv_bridge::CvImage(
                    msg->header, sensor_msgs::image_encodings::MONO8, image).toImageMsg();
                image_pub_.publish(gray_msg);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Publishing original color image");
                image_pub_.publish(msg);
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        }
    }

    void setMode(const std_srvs::srv::SetBool::Request::SharedPtr request,
                 std_srvs::srv::SetBool::Response::SharedPtr response)
    {
        mode_ = request->data ? 1 : 2;
        RCLCPP_INFO(this->get_logger(), "Service called. Mode set to: %s", mode_ == 1 ? "Grayscale" : "Color");
        response->success = true;
        response->message = mode_ == 1 ? "Mode set to Grayscale" : "Mode set to Color";
    }

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    std::string input_topic_, output_topic_;
    int mode_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConversionNode>());
    rclcpp::shutdown();
    return 0;
}


