#include "alliance_ros_auto_aim/event_bus.hpp"
#include "alliance_ros_auto_aim/include/sync_data_processor.hpp"
#include "alliance_ros_auto_aim/parameters/params_system_v1.hpp"
#include "alliance_ros_auto_aim/system_factory.hpp"
#include "hikcamera/image_capturer.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/detail/u_int8_multi_array__struct.hpp>

class AutoAimTest : public rclcpp::Node {
public:
    AutoAimTest()
        : rclcpp::Node("/test") {
        std::thread{[]() {
            hikcamera::ImageCapturer image_capturer_;
            while (true) {
                world_exe::core::EventBus::Publish(
                    world_exe::parameters::ParamsForSystemV1::raw_image_event,
                    image_capturer_.read());
            }
        }}.detach();
    };

private:
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sync_data_ =
        create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/alliance_auto_aim/sync", 1, [](const std_msgs::msg::UInt8MultiArray::SharedPtr data) {
                world_exe::ros::SyncData_Feb_TimeCameraGimbal_8byteAlignas copy_data;
                std::memcpy(&copy_data, &data->data, sizeof(copy_data));
                world_exe::core::EventBus::Publish(
                    world_exe::parameters::ParamsForSystemV1::camera_capture_transforms,
                    world_exe::ros::sync_data_process(copy_data));
            });
};

int main(int argc, char** argv) {
    world_exe::parameters::ParamsForSystemV1::set_szu_model_path(
        ament_index_cpp::get_package_share_directory("alliance_auto_aim")
        + "/models/szu_identify_model.onnx");
    world_exe::core::SystemFactory::Build(world_exe::enumeration::SystemVersion::V1_TraceFlow);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoAimTest>());
    rclcpp::shutdown();
}