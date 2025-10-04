#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <math.h>

#include "mros2.h"
#include "mros2-platform.h"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "message/motorBoard.hpp"
#include "message/powerboard.hpp"

#define DEG_TO_RAD(deg)  ((deg) / 180.0 * M_PI)  // 度からラジアンへの変換


static const char TAG[] = "main";
twai_node_handle_t handle = NULL;
void send_can(ID id, uint8_t* data, size_t size);

void userCallback(std_msgs::msg::UInt16 *msg)
{
  PowerBoard_format sendmsg = {0};
  sendmsg.id.format.to_BoardType = Board_Type::PowerBoard;
  sendmsg.id.format.to_BoardID = 0;
  sendmsg.id.format.message_type = Message_Type::Target;
  sendmsg.data.target.ON_OFF = msg->data;

  send_can(sendmsg.id, (uint8_t*)sendmsg.data.data, 32);
}

void userCallback2(geometry_msgs::msg::Twist *msg)
{
  MROS2_INFO("cmd_vel msg: '%f'", msg->linear.x);
  MotorBoard_format sendmsg = {0};
  sendmsg.id.format.to_BoardType = Board_Type::MotorBoard;
  sendmsg.id.format.to_BoardID = 0;
  sendmsg.id.format.message_type = Message_Type::Target;

  sendmsg.data.target.modem = ControlMode::PWM_Mode;
  sendmsg.data.target.target[0] = static_cast<int>(sinf(DEG_TO_RAD(msg->linear.x         ))*msg->angular.z*msg->linear.y);
  sendmsg.data.target.target[1] = static_cast<int>(sinf(DEG_TO_RAD(msg->linear.x -  90.0f))*msg->angular.z*msg->linear.y);
  sendmsg.data.target.target[2] = static_cast<int>(sinf(DEG_TO_RAD(msg->linear.x - 180.0f))*msg->angular.z*msg->linear.y);
  sendmsg.data.target.target[3] = static_cast<int>(sinf(DEG_TO_RAD(msg->linear.x - 270.0f))*msg->angular.z*msg->linear.y);

  send_can(sendmsg.id, (uint8_t*)sendmsg.data.data, sizeof(MotorBoard_Target));
}
 
static bool IRAM_ATTR nmea_on_received(
    twai_node_handle_t handle,
    const twai_rx_done_event_data_t *edata,
    void *user_ctx
){
    MROS2_INFO("subscribed msg");
    return true;
}

static esp_err_t nmea_init(twai_node_handle_t *handle) {
    twai_onchip_node_config_t node_config = {};
    node_config.io_cfg.tx = GPIO_NUM_10;
    node_config.io_cfg.rx = GPIO_NUM_9;
    node_config.bit_timing.bitrate = 1000000;
    node_config.data_timing.bitrate = 2000000;
    node_config.tx_queue_depth = 100,
    twai_new_node_onchip(&node_config, handle);
    
    twai_event_callbacks_t callbacks = {};
    callbacks.on_rx_done = nmea_on_received;

    twai_node_register_event_callbacks(*handle, &callbacks,NULL);

    twai_node_enable(*handle);
    return ESP_OK;
} 


void send_can(ID id, uint8_t* data, size_t size) {
  id.format.from_BoardType = Board_Type::Master_Board;
  id.format.from_BoardID = 0;

  twai_frame_t tx_msg={};
  tx_msg.header.id = id.id;           // Message ID
  tx_msg.header.ide = false;         // Use 29-bit extended ID format
  tx_msg.header.fdf = true;
  tx_msg.header.brs = true;
  tx_msg.header.rtr = false;
  tx_msg.buffer = data;
  tx_msg.buffer_len = size;
  ESP_ERROR_CHECK(twai_node_transmit(handle, &tx_msg, 10));
  osDelay(10);
}

extern "C" void app_main(void)
{
  ESP_ERROR_CHECK(nmea_init(&handle));

  /* connect to the network */
  if (mros2_platform_network_connect())
  {
    MROS2_INFO("successfully connect and setup network\r\n---");
  }
  else
  {
    MROS2_ERROR("failed to connect and setup network! aborting,,,");
    return;
  }
 
  MROS2_INFO("mbed mros2 start!");
  MROS2_INFO("app name: sub_uint16");
 
  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed");
 
  mros2::Node node = mros2::Node::create_node("mros2_node");
  mros2::Subscriber sub = node.create_subscription<std_msgs::msg::UInt16>("turtle1/poweron", 10, userCallback);
  mros2::Subscriber sub2 = node.create_subscription<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10, userCallback2);
  osDelay(100);
  mros2::spin();
}
 