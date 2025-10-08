#include <chrono>
#include <functional>
#include <sensor_msgs/image_encodings.hpp>
#include <map>
#include <tuple>
#include <string>
#include <utility>
using namespace std::literals::chrono_literals;

#include <rclcpp/rclcpp.hpp>
namespace rcl = rclcpp;
/// other ROS2-related
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
// #include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
/// MVS SDK
#include "MvCameraControl.h"
#include "MvErrorDefine.h"
typedef int MV_RET_T;

typedef char * C_STR;
typedef const char * const NAME;
template<class K_T, class V_T>
using FIND_TABLE = std::map<K_T, V_T>;

namespace camera_node{
/*	There are 3 categories of params (as for params' delivering path): 
	Node => this; (e.g. frame rate. Already set by xxx.launch.py)
	Node => this => hardware; (e.g. exposure time, gain. Already set by xxx.launch.py)
	hardware => this => Node. (e.g. width, height)
	All managed by this, so here are the lists for traversing Node's params and the camera hardware's params.
*/
/// Default parameters that need to be set are as the keys of rcl_Parameter_2_setter and std_string_2_MV_CC_GetXxxValue_2_rcl_ParameterValue shows.
/// The main product of this cpp. Will use  as much as possible in HikCameraNode in order to strengthen clarity
class HikCameraNode : public rcl::Node{
	/// Logging utilities
	#define glg get_logger()
	#define INFO(...) RCLCPP_INFO(glg, __VA_ARGS__);
	#define WARN(...) RCLCPP_WARN(glg, __VA_ARGS__);
	#define ERR(...) RCLCPP_ERROR(glg, __VA_ARGS__);
	#define CHECK_EXISTING_RET(_upstream_ret, msg, err_act) \
		switch(_upstream_ret){ \
			case MV_OK: break; \
			default: \
				ERR(msg ". Error code (HEX): %x", _upstream_ret); \
				err_act; \
				return _upstream_ret; \
		}
	#define CHECK_RET(_upstream, msg, err_act) \
		switch(const auto _upstream_ret = _upstream){ \
			case MV_OK: break; \
			default: \
				ERR(msg ". Error code (HEX): %x", _upstream_ret); \
				err_act; \
				return _upstream_ret; \
		}
	#define CHECK_NO_RET(_upstream, msg, err_act) \
		switch(const auto _upstream_ret = _upstream){ \
			case MV_OK: break; \
			default: \
				ERR(msg ". Error code (HEX): %x", _upstream_ret); \
				err_act; \
		}
	
public:
	/// NEED to declare_parameter before use, even though they are written in xxx.launch.py. But this autonomously does it! :-D
    static rclcpp::NodeOptions inline modify_options(rclcpp::NodeOptions &options){
        options.automatically_declare_parameters_from_overrides(true);
        return options;
    }
	inline HikCameraNode(rclcpp::NodeOptions options) : rcl::Node("hik_camera_node", modify_options(options)){
		INFO("Going to start HIK camera");
		
		/// Timing
		start_ = std::chrono::system_clock::now();
		
		/// Initialize the hardware, and fetch params in TABLE from camera to Node
		init_SDK_and_camera();
		INFO("Hardware-side initialized");
		
		MV_CC_GetXxxValue_2_rcl_Parameter_batch();
		INFO("Params fetched from camera to Node");
		
		/// Initialize this-side
		init_this();
		INFO("hw_DeviceSerialNumber: \"%s\"; image_topic: \"%s\"", camera_serial_.c_str(), image_topic_.c_str());
		// camera_name_ = declare_parameter("camera_name", "HIK VISION Test");
		// camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
		image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos_);
		// camera_pub_ = image_transport::create_camera_publisher(this, image_topic_, rmw_qos_profile_sensor_data);
		
		/// Timers: Check camera connection every 1ms; grab image every 1/FPS (already set in parameter init)
		// reconnect_timer_ = create_wall_timer(500ms, 
		// 	bind(&HikCameraNode::check_and_repair_connection, this)
		// );
		get_dynamic_params_timer_ = create_wall_timer(2s, 
			bind(&HikCameraNode::INFO_params_runtime_batch, this)
		);
		
		/// Fetch params in TABLE from Node to camera
		if(const auto _upstream_ret = apply_param_to_camera_batch(); !_upstream_ret.successful){
			ERR("Upstream error in `apply_param_to_camera_batch` in `HikCameraNode()`. Error reason: %s", _upstream_ret.reason.c_str());
		}
		set_parameters_callback = add_on_set_parameters_callback(
			bind(&HikCameraNode::apply_param_to_camera_batch<std::vector>, this, std::placeholders::_1)
		);
		INFO("Params fetched from Node to camera");
		
		INFO("Hik camera node started. Topic: %s", image_topic_.c_str());
	}

	inline ~HikCameraNode(){
		if(capture_thread_.joinable()){
			capture_thread_.join();
		}
		else{
			ERR("Cannot join `capture_thread_`");
		}
		INFO("Going to destroy HikCameraNode with device serial (HEX): %s", camera_serial_.c_str());
		finalize_camera_and_SDK();
		INFO("Hik camera node deconstructed");
	}
	
protected:
	std::chrono::system_clock::time_point start_;
	long inline get_ms_(){
		return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_).count();
	}
	
	void *camera_handle_ = nullptr;	// 相机句柄
	bool is_connected_ = false;		// 相机连接状态
	const static uint max_retries = 4;
	uint retries = 0;
	std::string camera_serial_;		// 相机序列号
	// std::string camera_name_;		// 相机序列号
	uint capture_timeout_msec; /// Time out time in capturing image
	MV_CC_PIXEL_CONVERT_PARAM_EX params_MV_conv;
	
	std::string image_topic_;		// 图像发布Topic
	rcl::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;	// 图像发布者
	// image_transport::CameraPublisher camera_pub_;
	sensor_msgs::msg::Image msg;
	/// Queue size: 10, QoS: BEST_EFFORT
	uint queue_size_ = 10;
	rcl::QoS qos_ = rcl::QoS(rcl::KeepLast(queue_size_), rmw_qos_profile_sensor_data);
	
	std::thread capture_thread_;
	
	rcl::TimerBase::SharedPtr 
		reconnect_timer_,	// 断线重连定时器
		capture_timer_,		// 图像采集定时器
		get_dynamic_params_timer_;
	OnSetParametersCallbackHandle::SharedPtr set_parameters_callback;
	// std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
	// sensor_msgs::msg::CameraInfo camera_info_msg_;
	
	/// ---------------- Dictionaries / Maps ----------------
	
	const FIND_TABLE<
		std::string,
		MvGvspPixelType
	> image_encodings_2_MvGvspPixelType = {
		{
			sensor_msgs::image_encodings::RGB8,
			PixelType_Gvsp_RGB8_Packed
		},
		{
			sensor_msgs::image_encodings::BGR8,
			PixelType_Gvsp_BGR8_Packed
		},
	};
	
	/// Interface for setting parameters from this to hardware. names in rcl => set corresponding name in MV. May be locally managed.
	
	const FIND_TABLE<
		std::string, 
		std::function<MV_RET_T(const rcl::Parameter &param)>
	> rcl_Parameter_2_setter = {
		{
			"capture_timeout_msec",
			[this](const rcl::Parameter &param)->MV_RET_T{
				INFO("Fetched `capture_timeout_msec`: %ld", param.as_int());
				capture_timeout_msec = param.as_int();
				return MV_OK;
			}
		},
		{
			"hw_DeviceSerialNumber",
			[this](const rcl::Parameter &param)->MV_RET_T{
				INFO("Fetched `hw_DeviceSerialNumber`: %s", param.as_string().c_str());
				camera_serial_ = param.as_string();
				return MV_OK;
			}
		},
		{
			"exposure_time", 
			[this](const rcl::Parameter &param)->MV_RET_T{
				INFO("Fetched `exposure_time`: %lf (microsec)", param.as_double());
				return MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_double());
			}
		},
		{
			"gain", 
			[this](const rcl::Parameter &param)->MV_RET_T{
				INFO("Fetched `gain`: %lf (dB)", param.as_double());
				return MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
			}
		},
		/// Locally managed
		{
			"frame_rate", 
			[this](const rcl::Parameter &param)->MV_RET_T{
				// INFO("Fetched `frame_rate`: %ld (FPS)", param.as_int());
				// if(capture_timer_ != nullptr && !capture_timer_->is_canceled()){
				// 	INFO("Going to cancel the timer controlling frame rate");
				// 	capture_timer_->cancel();
				// }
				// capture_timer_ = create_wall_timer(
				// 	1.0s / param.as_int(),
				// 	std::bind(&HikCameraNode::capture_and_send, this)
				// );
				// INFO("Timer controlling frame rate reset");
				// return MV_OK;
				if(capture_thread_.joinable()){
					capture_thread_.join();
				}
				capture_thread_ = std::thread{[this]()->void{
						std::this_thread::sleep_for(1s); /// Avoid unset changes in other lambda
						for(;;){
							capture_and_send();
						}
					}
				};
				return MV_OK;
			}
		},
		/// Cannot set using string! And cannot set at hardware-layer at all!
		// {
		// 	"pixel_format", 
		// 	[this](const rcl::Parameter &param){
		// 		INFO("Fetched `pixel_format`: %s", param.as_string().c_str());
		// 		return MV_CC_SetEnumValueByString(camera_handle_, "PixelFormat", param.as_string().c_str());
		// 	}
		// },
		// {
		// 	"hw_PixelFormat", 
		// 	[this](const rcl::Parameter &param){
		// 		INFO("Fetched `pixel_format` (HEX): %lx", param.as_int());
		// 		return MV_CC_SetEnumValue(camera_handle_, "PixelFormat", param.as_int());
		// 	}
		// },
		{
			"pixel_format",
			[this](const rcl::Parameter &param)->MV_RET_T{
				INFO("Fetched `pixel_format` (HEX): %s", param.as_string().c_str());
				
				const auto tmp = image_encodings_2_MvGvspPixelType.find(param.as_string());
				if(tmp == image_encodings_2_MvGvspPixelType.end()){
					ERR("PixelType not implemented in `image_encodings_2_MvGvspPixelType` in lambda `pixel_format`");
					return MV_E_SUPPORT;
				}
				
				params_MV_conv.enDstPixelType = tmp->second;
				msg.encoding = param.as_string();
				return MV_OK;
			}
		},
	};
	MV_RET_T inline apply_param_to_camera(const rcl::Parameter &param){
		const auto tmp_iter = rcl_Parameter_2_setter.find(param.get_name());
		if(tmp_iter == rcl_Parameter_2_setter.end()){
			ERR("Parameter type of \"%s\" is not implemented in `rcl_Parameter_2_setter` in `apply_param_to_camera`", param.get_name().c_str());
			return MV_E_SUPPORT;
		}
		return tmp_iter->second(param);
	}
	/// Multiple param setter from Node to camera
	template<template<class...> class container>
	rcl_interfaces::msg::SetParametersResult inline apply_param_to_camera_batch(const container<rcl::Parameter> &params){
		rcl_interfaces::msg::SetParametersResult result; /// Default unsuccessful
		for(const rcl::Parameter & param : params){
			switch(const auto _upstream_ret = apply_param_to_camera(param)){
				case MV_OK: break;
				default:
					ERR("Upstream error in `apply_param_to_camera` in `apply_param_to_camera_batch`. Error code (HEX): %x", _upstream_ret);
					result.successful = false;
					result.reason = "Upstream error";
					return result;
			}
		}
		result.successful = true;
		return result;
	}
	/// Overload version: Auto-obtain params in Node
	rcl_interfaces::msg::SetParametersResult inline apply_param_to_camera_batch(){
		rcl_interfaces::msg::SetParametersResult result;
		for(const auto &[k, v] : rcl_Parameter_2_setter){
			INFO("Going to: get_parameter(\"%s\")", k.c_str());
			rcl::Parameter tmp;
			get_parameter(k, tmp);
			CHECK_NO_RET(v(tmp),
				"Upstream error in `apply_param_to_camera` in `apply_param_to_camera_batch`",
				{result.successful = false;
				result.reason = "Upstream error";
				return result;}
			)
		}
		result.successful = true;
		return result;
	}

	/// Interface collector: Get. Begin with hw_ (hardware) to distinguish from those manually set values
	const FIND_TABLE<
		std::string, 
		std::function<std::tuple<MV_RET_T, rcl::ParameterValue>()>
	> std_string_2_MV_CC_GetXxxValue_2_rcl_ParameterValue_static = {
		{
			"hw_DeviceSerialNumber",
			[this]()->std::tuple<MV_RET_T, rcl::ParameterValue>{
				MVCC_STRINGVALUE tmp;
				const MV_RET_T _ret = MV_CC_GetStringValue(camera_handle_, "DeviceSerialNumber", &tmp);
				INFO("Got \"DeviceSerialNumber\": %s", tmp.chCurValue);
				return {_ret, rcl::ParameterValue(tmp.chCurValue)};
			}
		},
		/// Got an error when getting: 
		/*
		[hik_camera_node-1] [INFO] [1759748315.107330406] [hik_camera_node]: Got "DeviceID": P���X
		[hik_camera_node-1] [ERROR] [1759748315.107602146] [hik_camera_node]: Upstream error in lambda expr in `MV_CC_GetXxxValue_2_rcl_Parameter_batch`. Error code (HEX): 80000100
		*/
		// {
		// 	"hw_DeviceID",
		// 	[this]()->std::tuple<MV_RET_T, rcl::ParameterValue>{
		// 		MVCC_STRINGVALUE tmp;
		// 		const MV_RET_T _ret = MV_CC_GetStringValue(camera_handle_, "DeviceID", &tmp);
		// 		INFO("Got \"DeviceID\": %s", tmp.chCurValue);
		// 		return {_ret, rcl::ParameterValue(tmp.chCurValue)};
		// 	}
		// },
		{
			"hw_Width",
			[this]()->std::tuple<MV_RET_T, rcl::ParameterValue>{
				MVCC_INTVALUE_EX tmp;
				const MV_RET_T _ret = MV_CC_GetIntValueEx(camera_handle_, "Width", &tmp);
				INFO("Got \"Width\": %ld", tmp.nCurValue);
				return {_ret, rcl::ParameterValue(tmp.nCurValue)};
			}
		},
		{
			"hw_Height",
			[this]()->std::tuple<MV_RET_T, rcl::ParameterValue>{
				MVCC_INTVALUE_EX tmp;
				const MV_RET_T _ret = MV_CC_GetIntValueEx(camera_handle_, "Height", &tmp);
				INFO("Got \"Height\": %ld", tmp.nCurValue);
				return {_ret, rcl::ParameterValue(tmp.nCurValue)};
			}
		},
		/// Get int type of the enum. TODO: Change on runtime by restarting device
		// {
		// 	"hw_PixelFormat",
		// 	[this]()->std::tuple<MV_RET_T, rcl::ParameterValue>{
		// 		MVCC_ENUMVALUE_EX tmp;
		// 		const MV_RET_T _ret = MV_CC_GetEnumValueEx(camera_handle_, "PixelFormat", &tmp);
		// 		INFO("Got \"PixelFormat\" (HEX): %x", tmp.nCurValue);
		// 		return {_ret, rcl::ParameterValue(static_cast<int64_t>(tmp.nCurValue))};
		// 	}
		// },
	};
	const FIND_TABLE<
		std::string, 
		std::function<std::tuple<MV_RET_T, rcl::ParameterValue>()>
	> std_string_2_MV_CC_GetXxxValue_2_rcl_ParameterValue_dynamic = {
		{
			"hw_ResultingFrameRate",
			[this]()->std::tuple<MV_RET_T, rcl::ParameterValue>{
				MVCC_FLOATVALUE tmp;
				const MV_RET_T _ret = MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &tmp);
				INFO("Runtime echo: \"ResultingFrameRate\": %lf", tmp.fCurValue);
				return {_ret, rcl::ParameterValue(tmp.fCurValue)};
			}
		},
		// {
		// 	"hw_ResultingLineRate",
		// 	[this]()->std::tuple<MV_RET_T, rcl::ParameterValue>{
		// 		MVCC_FLOATVALUE tmp;
		// 		const MV_RET_T _ret = MV_CC_GetFloatValue(camera_handle_, "ResultingLineRate", &tmp);
		// 		INFO("Runtime echo: \"ResultingLineRate\": %lf", tmp.fCurValue);
		// 		return {_ret, rcl::ParameterValue(tmp.fCurValue)};
		// 	}
		// },
	};
	/// Single param getter: Forward params from MV_CC funcs to this program
	std::tuple<MV_RET_T, rcl::ParameterValue> inline MV_CC_GetXxxValue_2_rcl_Parameter(const std::string &name, const FIND_TABLE<
		std::string, 
		std::function<std::tuple<MV_RET_T, rcl::ParameterValue>()>
	> &table){
		const auto tmp_iter = table.find(name);
		if(tmp_iter == table.end()){
			ERR("Parameter type of \"%s\" is not implemented in in `MV_CC_GetXxxValue_2_rcl_Parameter`", name.c_str());
			return {MV_E_SUPPORT, rcl::ParameterValue()};
		}
		return tmp_iter->second();
	}
	/// Batch fetching (Only overloaded version, considering actual usage)
	MV_RET_T inline MV_CC_GetXxxValue_2_rcl_Parameter_batch(){
		for(const auto &[k, v] : std_string_2_MV_CC_GetXxxValue_2_rcl_ParameterValue_static){
			const auto [_upstream_ret, param_v] = v();
			CHECK_EXISTING_RET(_upstream_ret, "Upstream error in a lambda expr in `MV_CC_GetXxxValue_2_rcl_Parameter_batch`",)
			declare_parameter(k, param_v);
		}
		return MV_OK;
	}
	/// Print these dynamic params to INFO channel during runtime. (Should not set_parameter, which will trigger parameter change.)
	MV_RET_T inline INFO_params_runtime_batch(){
		INFO("Going to runtime echo:")
		for(const auto &[k, v] : std_string_2_MV_CC_GetXxxValue_2_rcl_ParameterValue_dynamic){
			const auto [_upstream_ret, param_v] = v();
			CHECK_EXISTING_RET(_upstream_ret, "Upstream error in a lambda expr in `INFO_params_runtime_batch`",)
		}
		return MV_OK;
	}
	
	
	/// ---------------- Data-related definitions end ----------------
	
	/// ---------------- SDK-side sealing 0: Stack building and destroying ----------------
	
	/// Layer 0 initialization
	MV_RET_T inline init_SDK(){
		return MV_CC_Initialize();
	}
	/// Prepare for Layer 1 initialization
	std::tuple<MV_RET_T, MV_CC_DEVICE_INFO *> inline find_proper_camera(){
		MV_CC_DEVICE_INFO_LIST device_list;
		__restart_finding_device:
		if(retries >= max_retries){
			ERR("max_retries reached in `find_proper_camera`. return");
			return {MV_E_UNKNOW, nullptr};
		}
		
		switch(const auto _ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list)){ /// &: lvalue ref
			case MV_OK: break;
			default:
				ERR("Failed to enumerate cameras in `MV_CC_EnumDevices` in `find_proper_camera`. Error code (HEX): %x", _ret);
				return {_ret, nullptr};
		}
		
		if(device_list.nDeviceNum == 0){
			WARN("No HIK camera found in `find_proper_camera`");
			std::this_thread::sleep_for(1s); /// Avoid overwhelming messages
			retries++;
			goto __restart_finding_device;
		}
		else{
			INFO("Found %u HIK camera(s) in `find_proper_camera`", device_list.nDeviceNum);
			retries = 0;
		}

		/// Select camera. 1st by default.
		MV_CC_DEVICE_INFO* selected_device = nullptr;
		if(camera_serial_.empty()){ /// string == "",
			selected_device = device_list.pDeviceInfo[0]; /// then the 1st should be selected
			INFO("Using first camera in `find_proper_camera`. hw_DeviceSerialNumber: \"%s\"", selected_device->SpecialInfo.stUsb3VInfo.chSerialNumber);
		}
		else{
			for(uint i = 0; i < device_list.nDeviceNum; i++){
				INFO("Comparing in `find_proper_camera`: \"%s\" vs \"%s\"", 
					reinterpret_cast<const char *>(device_list.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber), 
					camera_serial_.c_str()
				);
				if(!strcmp(
					reinterpret_cast<const char *>(device_list.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber), 
					camera_serial_.c_str()
				)){ /// C style for efficiency
					selected_device = device_list.pDeviceInfo[i]; /// Only copy pointer
					break;
				}
			}
			if(selected_device == nullptr){
				ERR("Camera not found with hw_DeviceSerialNumber \"%s\" in `find_proper_camera`", camera_serial_.c_str());
				return{MV_E_HANDLE, nullptr};
			}
		}
		return{MV_OK, selected_device};
	}
	/// Layer 1 initialization
	MV_RET_T inline create_camera_handle(){
		const auto &[_upstream_ret, selected_device] = find_proper_camera();
		CHECK_EXISTING_RET(_upstream_ret, "Upstream error in `find_proper_camera` in `create_camera_handle`",)
		
		CHECK_RET(MV_CC_CreateHandle(&camera_handle_, selected_device),
			"Error in `MV_CC_CreateHandle` in `create_camera_handle`",
			MV_CC_DestroyHandle(&camera_handle_) /// Because here is Layer 1
		)
		return MV_OK;
	}
	/// Layer 2
	MV_RET_T inline open_camera(){
		CHECK_RET(MV_CC_OpenDevice(camera_handle_),
			"Error in `MV_CC_OpenDevice` in `open_camera`",
			{MV_CC_DestroyHandle(&camera_handle_);
			camera_handle_ = nullptr;}
		)
		return MV_OK;
	}
	
	
	/// ---------------- SDK-side sealing 1: Can be directly used ----------------
	
	MV_RET_T inline init_SDK_and_camera(){
		/// Layer 0
		CHECK_RET(init_SDK(), "Error in `init_SDK` in `init_camera`",)
		return init_camera();
	}
	MV_RET_T inline init_camera(){
		/// Layer 1 (including preparation)
		CHECK_RET(create_camera_handle(), "Error in `create_camera_handle` in `init_camera`",)
		/// Layer 2
		CHECK_RET(open_camera(), "Error in `open_camera` in `init_camera`",)
		CHECK_NO_RET(MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_RGB8_Packed), "Error setting PixelFormat",);
		/// On Layer 2, start capturing
		CHECK_RET(MV_CC_StartGrabbing(camera_handle_), "Error in `MV_CC_StartGrabbing` in `init_camera`",)
		is_connected_ = true;
		INFO("Camera connected successfully.");
		return MV_OK;
	}
	MV_RET_T inline finalize_camera_and_SDK(){
		if(camera_handle_ == nullptr){
			WARN("Try to finalize with a nullptr in `finalize_camera`");
			return MV_E_HANDLE;
		}
		MV_CC_StopGrabbing(camera_handle_);
		MV_CC_CloseDevice(camera_handle_);
		MV_CC_DestroyHandle(camera_handle_);
		MV_CC_Finalize();
		INFO("Camera node closed.");
		return MV_OK;
	}

	MV_RET_T inline check_and_repair_connection(){
		if(!(is_connected_ && MV_CC_IsDeviceConnected(camera_handle_))){
			if(retries > max_retries){
				ERR("retries > max_retries. return");
				return MV_E_HANDLE;
			}
			WARN("Bad connection detected in `check_and_repair_connection`. Reconnecting to camera...");
			finalize_camera_and_SDK();
			switch(const auto _ret = init_camera()){
				case MV_OK:
					is_connected_ = true;
					break;
				default:
					return _ret;
			}
			retries++;
		}
		else{
			retries = 0;
		}
		return MV_OK;
	}
	
	
	/// ---- Manage this
	void inline init_this(){
		// get_parameter("hw_DeviceSerialNumber", camera_serial_); /// Should not modify in this Node, because reconnecting uses it
		get_parameter("image_topic", image_topic_);
		INFO("Got `hw_DeviceSerialNumber`, `image_topic` in `init_this`: \"%s\", \"%s\"", camera_serial_.c_str(), image_topic_.c_str());
		
		get_parameter("hw_Width", params_MV_conv.nWidth);
		get_parameter("hw_Height", params_MV_conv.nHeight);
		{
		int64_t tmp;
		get_parameter("hw_PixelFormat", tmp);
		params_MV_conv.enSrcPixelType = static_cast<MvGvspPixelType>(tmp);
		}
		INFO("Got `hw_Width`, `hw_Height`, `hw_PixelFormat` in `init_this`: \"%d\", \"%d\", %lx", 
			params_MV_conv.nWidth, params_MV_conv.nHeight, static_cast<int64_t>(params_MV_conv.enSrcPixelType));
	}

	// ---------------- After all initializations ----------------
	void inline capture_and_send(){
		INFO("#t| Enter: %ld ms", get_ms_());
		CHECK_NO_RET(check_and_repair_connection(), "`check_and_repair_connection` failed. return", return);
		
		INFO("#t| Checked connection: %ld ms", get_ms_());
		/// Extract from SDK
		static MV_FRAME_OUT frame; /// MV_FRAME_OUT stores image address and info only
		CHECK_NO_RET(MV_CC_GetImageBuffer(camera_handle_, &frame, capture_timeout_msec),
			"Failed to get image buffer in `capture_and_send`. Disconnected",
			{is_connected_ = false; retries++; return;}
		)
		
		INFO("#t| Done MV_CC_GetImageBuffer: %ld ms", get_ms_());
		/// Prepare message
		msg.width = frame.stFrameInfo.nExtendWidth;
		msg.step = msg.width * 3;
		msg.height = frame.stFrameInfo.nExtendHeight;
		msg.data.resize(msg.step * msg.height);
		
		INFO("#t| Done resize: %ld ms", get_ms_());
		/* LOW EFFICIENCY: MV_CC_ConvertPixelTypeEx
		params_MV_conv.nWidth = frame.stFrameInfo.nExtendWidth;
		params_MV_conv.nHeight = frame.stFrameInfo.nExtendHeight;
		params_MV_conv.pSrcData = frame.pBufAddr;
		params_MV_conv.nSrcDataLen = frame.stFrameInfo.nFrameLen;
		params_MV_conv.enSrcPixelType = frame.stFrameInfo.enPixelType;
		// INFO("Converting: Source image PixelType (HEX): %lx", params_MV_conv.enSrcPixelType);
		params_MV_conv.pDstBuffer = msg.data.data();
		// params_MV_conv.nDstLen = msg.data.size();
		params_MV_conv.nDstBufferSize = msg.data.size();
		CHECK_NO_RET(MV_CC_ConvertPixelTypeEx(camera_handle_, &params_MV_conv),
			"Error in `MV_CC_ConvertPixelTypeEx` in `capture_and_send`",
		);
		
		INFO("#t| Done MV_CC_ConvertPixelTypeEx: %ld ms", get_ms_());
		*/
		memcpy(msg.data.data(), frame.pBufAddr, msg.data.size());
		INFO("#t| Done memcpy: %ld ms", get_ms_());
		
		msg.header.set__stamp(now());
		image_pub_->publish(msg);
		// camera_info_msg_.header = msg.header;
		// camera_pub_.publish(msg, camera_info_msg_);
		
		INFO("#t| Done publish: %ld ms", get_ms_());
		/// Release things SDK produced
		CHECK_NO_RET(MV_CC_FreeImageBuffer(camera_handle_, &frame),
			"Error in `MV_CC_FreeImageBuffer` in `capture_and_send`",
		);
		INFO("#t| Done all: %ld ms", get_ms_());
	}
	// void inline capture_and_send_continuously(){
	// 	for(;;){
	// 		capture_and_send();
	// 	}
	// }
	
#undef CHECK_RET
#undef INFO
#undef WARN
#undef ERR
#undef glg
};
};

RCLCPP_COMPONENTS_REGISTER_NODE(camera_node::HikCameraNode)
