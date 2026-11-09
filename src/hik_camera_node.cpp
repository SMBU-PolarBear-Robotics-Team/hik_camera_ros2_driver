#include <stdexcept>
#include <string>

#include "MvCameraControl.h"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"

namespace hik_camera_ros2_driver
{
class HikCameraRos2DriverNode : public rclcpp::Node
{
public:
  explicit HikCameraRos2DriverNode(const rclcpp::NodeOptions & options)
  : Node("hik_camera_ros2_driver", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraRos2DriverNode!");

    if (!initializeCamera()) {
      throw std::runtime_error("Failed to initialize Hik camera");
    }
    declareParameters();
    startCamera();

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraRos2DriverNode::dynamicParametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread(&HikCameraRos2DriverNode::captureLoop, this);
  }

  ~HikCameraRos2DriverNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraRos2DriverNode destroyed!");
  }

private:
  bool initializeCamera()
  {
    MV_CC_DEVICE_INFO_LIST device_list;

    // enum device
    while (rclcpp::ok()) {
      n_ret_ = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
      if (n_ret_ != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enumerate devices, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
      } else if (device_list.nDeviceNum == 0) {
        RCLCPP_ERROR(this->get_logger(), "No camera found, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
      } else {
        RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);
        break;
      }
    }

    if (!rclcpp::ok()) {
      return false;
    }

    n_ret_ = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
    if (n_ret_ != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create camera handle!");
      return false;
    }

    n_ret_ = MV_CC_OpenDevice(camera_handle_);
    if (n_ret_ != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera device!");
      return false;
    }

    // Get camera information
    n_ret_ = MV_CC_GetImageInfo(camera_handle_, &img_info_);
    if (n_ret_ != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get camera image info!");
      return false;
    }

    // Init convert param
    image_msg_.data.resize(img_info_.nHeightMax * img_info_.nWidthMax * 3);
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    return true;
  }

  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // Acquisition frame rate
    param_desc.description = "Acquisition frame rate in Hz";
    MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double acquisition_frame_rate =
      this->declare_parameter("acquisition_frame_rate", 165.0, param_desc);
    MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
    MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", acquisition_frame_rate);
    RCLCPP_INFO(this->get_logger(), "Acquisition frame rate: %f", acquisition_frame_rate);

    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

    int status;

    // ADC Bit Depth
    param_desc.description = "ADC Bit Depth";
    param_desc.additional_constraints = "Supported values: Bits_8, Bits_12";
    std::string adc_bit_depth = this->declare_parameter("adc_bit_depth", "Bits_8", param_desc);
    status = MV_CC_SetEnumValueByString(camera_handle_, "ADCBitDepth", adc_bit_depth.c_str());
    if (status == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "ADC Bit Depth set to %s", adc_bit_depth.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set ADC Bit Depth, status = %d", status);
    }

    // Pixel format
    param_desc.description = "Pixel Format";
    std::string pixel_format = this->declare_parameter("pixel_format", "RGB8Packed", param_desc);
    status = MV_CC_SetEnumValueByString(camera_handle_, "PixelFormat", pixel_format.c_str());
    if (status == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Pixel Format set to %s", pixel_format.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set Pixel Format, status = %d", status);
    }
  }

  void startCamera()
  {
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    camera_name_ = this->declare_parameter("camera_name", "camera");
    frame_id_ = this->declare_parameter("frame_id", camera_name_ + "_optical_frame");
    camera_topic_ = this->declare_parameter("camera_topic", camera_name_ + "/image");

    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, camera_topic_, qos);

    MV_CC_StartGrabbing(camera_handle_);

    // Load camera info
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url = this->declare_parameter(
      "camera_info_url", "package://hik_camera_ros2_driver/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }
  }

  void captureLoop()
  {
    MV_FRAME_OUT out_frame;
    RCLCPP_INFO(this->get_logger(), "Publishing image!");

    image_msg_.header.frame_id = frame_id_;
    image_msg_.encoding = "rgb8";

    while (rclcpp::ok()) {
      n_ret_ = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
      if (MV_OK == n_ret_) {
        auto image_size = static_cast<std::size_t>(out_frame.stFrameInfo.nWidth) *
                          out_frame.stFrameInfo.nHeight * 3;
        if (image_msg_.data.size() != image_size) {
          image_msg_.data.resize(image_size);
        }

        convert_param_.pDstBuffer = image_msg_.data.data();
        convert_param_.nDstBufferSize = image_msg_.data.size();
        convert_param_.pSrcData = out_frame.pBufAddr;
        convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
        convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

        MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

        image_msg_.header.stamp = this->now();
        image_msg_.height = out_frame.stFrameInfo.nHeight;
        image_msg_.width = out_frame.stFrameInfo.nWidth;
        image_msg_.step = out_frame.stFrameInfo.nWidth * 3;

        camera_info_msg_.header = image_msg_.header;
        camera_pub_.publish(image_msg_, camera_info_msg_);

        MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
        fail_count_ = 0;

        static auto last_log_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 3) {
          MVCC_FLOATVALUE f_value;
          MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &f_value);
          RCLCPP_DEBUG(this->get_logger(), "ResultingFrameRate: %f Hz", f_value.fCurValue);
          last_log_time = now;
        }

      } else {
        fail_count_++;
        RCLCPP_WARN(
          this->get_logger(), "Get buffer failed! nRet: [%x], consecutive failures: %d", n_ret_,
          fail_count_);
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_StartGrabbing(camera_handle_);
      }

      if (fail_count_ > 5) {
        RCLCPP_FATAL(this->get_logger(), "Camera failed!");
        rclcpp::shutdown();
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      const auto & type = param.get_type();
      const auto & name = param.get_name();
      int status = MV_OK;

      if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (name == "gain") {
          status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        } else {
          result.successful = false;
          result.reason = "Unknown parameter: " + name;
          continue;
        }
      } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        if (name == "exposure_time") {
          status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        } else {
          result.successful = false;
          result.reason = "Unknown parameter: " + name;
          continue;
        }
      } else {
        result.successful = false;
        result.reason = "Unsupported parameter type for: " + name;
        continue;
      }

      if (status != MV_OK) {
        result.successful = false;
        result.reason = "Failed to set " + name + ", status = " + std::to_string(status);
      }
    }

    return result;
  }

  void * camera_handle_ = nullptr;
  int n_ret_ = MV_OK;
  MV_IMAGE_BASIC_INFO img_info_;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  image_transport::CameraPublisher camera_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  std::string camera_name_;
  std::string frame_id_;
  std::string camera_topic_;

  std::thread capture_thread_;
  int fail_count_ = 0;
};
}  // namespace hik_camera_ros2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera_ros2_driver::HikCameraRos2DriverNode)
