// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

// Author: Nikola Banovic

#ifndef beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_
#define beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_

#include <string>
#include <vector>
#include <limits>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ads/AdsLib.h"
#include "ads/AdsDevice.h"
namespace beckhoff_ads_hardware_interface
{

enum class PLCType
{
    // Using enum instead of raw strings for faster iterations in read/write
    UNKNOWN, BOOL, LREAL, REAL, UDINT, DINT, INT, UINT, SINT, USINT, BYTE, STRING,
};

// Describes each PLC item (array or single variable) for polling via sum commands.
struct ADSDataLayout {
    // Configured from yaml
    std::string plc_name_symbolic;     // e.g., "MAIN.Joint_Pos_State". Used to get the handle.
    PLCType     plc_type;
    uint32_t    ads_handle;          // PLC Handle for the symbolic name. Not using AdsHandle, as we don't need a shared ptr, just a value to paste in the message

    size_t      num_elements;        // 6 for LREAL[6], 1 for single LREAL/BOOL etc.
    size_t      plc_element_byte_size; // byte size of ONE element on PLC (e.g., 8 for LREAL, 1 for BOOL).

    // for mapping to ros2_control
    size_t      offset_in_ros2_control;    // Starting index in hw_states_ or hw_commands_

    // If we have an identically named command and state interface, in case there are no new commands to be sent to the robot, we want to use the value read in the state interface for the next request.
    size_t      offset_in_ros2_control_state_ = std::numeric_limits<size_t>::max();

    // for unpacking sum read response  [Err1_ULONG,...,ErrN_ULONG | Data1_bytes,...,DataN_bytes]
    size_t      offset_in_read_response_error;    // Byte offset where this item's ULONG error code starts.
    size_t      offset_in_read_response_data;     // Byte offset where this item's data starts.

    // for packing sum write response [ADS_ITEM_REQ_HEADER_1,...,ADS_ITEM_REQ_HEADER_N | Data1_bytes,...,DataN_bytes]
    size_t      offset_in_write_request_data;     // Byte offset where this item's data starts.
};

// Packed header for sum read/write request item headers
typedef struct {
    uint32_t indexGroup;  // ADSIGRP_SYM_VALBYHND
    uint32_t indexOffset; // The ADS Handle
    uint32_t NumBytesData;      // total num of bytes in this data section
} ADS_ITEM_REQ_HEADER;

class BeckhoffADSHardwareInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentParams & params);

    hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    rclcpp::Logger getLogger() { return rclcpp::get_logger("BeckhoffADSHardwareInterface"); }
    std::shared_ptr<rclcpp::Clock> logging_throttle_clock_;

    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;

    // ========= PLC ==============================

    // PLC Type and Size Helpers
    PLCType strToPlcType(const std::string& type_str);
    size_t plcTypeByteSize(PLCType type_enum);

    // ADS Communication objects
    std::unique_ptr<AdsDevice> ads_device_; // Manages the route/connection to the PLC
    bool configure_ads_device();

    // Metadata (populated in on interface export)
    // Describes each variable on the PLC
    std::vector<ADSDataLayout> ads_item_layouts_read_;
    std::vector<ADSDataLayout> ads_item_layouts_write_;
    bool build_sum_read_buffers();
    bool build_sum_write_buffers();

    // ADS Sum Command Buffers
    // SENT: List of ADS_ITEM_REQ_HEADER structs
    // RECEIVED: List of [Err1_ULONG,...,ErrN_ULONG | Data1_bytes,...,DataN_bytes]
    std::vector<uint8_t> ads_buffer_sum_read_request_;
    std::vector<uint8_t> ads_buffer_sum_read_response_;
    size_t num_items_read_ = 0;

    // SENT: List of [ADS_ITEM_REQ_HEADER_1,...,ADS_ITEM_REQ_HEADER_N | Data1_bytes,...,DataN_bytes]
    // RECEIVED: List of ErrCode_ULONGs
    std::vector<uint8_t> ads_buffer_sum_write_request_;
    std::vector<uint8_t> ads_buffer_sum_write_response_;
    size_t num_items_write_ = 0;
  };

}  // namespace beckhoff_ads_hardware_interface

#endif  // beckhoff_ads_hardware_interface__BECKHOFF_SYSTEM_HPP_
