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

#include "messageFormat/motorBoard.hpp"
#include "messageFormat/powerboard.hpp"

#define DEG_TO_RAD(deg) ((deg) / 180.0 * M_PI) // 度からラジアンへの変換

static const char TAG[] = "main";
void send_can(ID_Format id, uint8_t *data, size_t size);

void userCallback(std_msgs::msg::UInt16 *msg)
{
  PowerBoard_Target sendmsg = {0};
  ID_Format id;
  id.format.to_BoardType = Board_Type::PowerBoard;
  id.format.to_BoardID = 0;
  id.format.message_type = Message_Type::Target;
  sendmsg.ON_OFF = !msg->data;
  MROS2_INFO("power %s", msg->data ? "on" : "off");

  send_can(id, reinterpret_cast<uint8_t *>(&sendmsg), sizeof(PowerBoard_Target));
}

void userCallback2(geometry_msgs::msg::Twist *msg)
{
  ID_Format id;
  id.format.to_BoardType = Board_Type::MotorBoard;
  id.format.to_BoardID = 0;
  id.format.message_type = Message_Type::Target;

  MotorBoard_Target sendmsg;
  sendmsg.mode = ControlMode::PWM_Mode;
  sendmsg.target[0] = static_cast<int>(sinf(DEG_TO_RAD(msg->linear.x)) * msg->angular.z * msg->linear.y);
  sendmsg.target[1] = static_cast<int>(sinf(DEG_TO_RAD(msg->linear.x - 90.0f)) * msg->angular.z * msg->linear.y);
  sendmsg.target[2] = static_cast<int>(sinf(DEG_TO_RAD(msg->linear.x - 180.0f)) * msg->angular.z * msg->linear.y);
  sendmsg.target[3] = static_cast<int>(sinf(DEG_TO_RAD(msg->linear.x - 270.0f)) * msg->angular.z * msg->linear.y);
  send_can(id, reinterpret_cast<uint8_t *>(&sendmsg), sizeof(MotorBoard_Target));

  id.format.to_BoardID = 1;
  sendmsg.mode = ControlMode::PWM_Mode;
  sendmsg.target[3] = static_cast<int>(msg->angular.x * 100.0f);
  send_can(id, reinterpret_cast<uint8_t *>(&sendmsg), sizeof(MotorBoard_Target));
}
static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t recv_buff[64];
    twai_frame_t rx_frame = {0};
    rx_frame.buffer = recv_buff;
    rx_frame.buffer_len = sizeof(recv_buff);
    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
        // receive ok, do something here
        MROS2_INFO("subscribed msg %ld",  rx_frame.header.id);
    }
    return false;
}

static esp_err_t nmea_init(twai_node_handle_t *handle)
{
  twai_onchip_node_config_t node_config = {};
  node_config.io_cfg.tx = GPIO_NUM_10;
  node_config.io_cfg.rx = GPIO_NUM_9;
  node_config.bit_timing.bitrate = 1000000;
  node_config.data_timing.bitrate = 2000000;
  node_config.tx_queue_depth = 100,
  twai_new_node_onchip(&node_config, handle);

  twai_event_callbacks_t callbacks = {};
  callbacks.on_rx_done = twai_rx_cb;

  twai_node_register_event_callbacks(*handle, &callbacks, NULL);

  twai_node_enable(*handle);
  return ESP_OK;
}

twai_node_handle_t handle = NULL;

void send_can(ID_Format id, uint8_t *data, size_t size)
{
  id.format.from_BoardType = Board_Type::Master_Board;
  id.format.from_BoardID = 0;

  twai_frame_t tx_msg = {};
  tx_msg.header.id = id.id;  // Message ID
  tx_msg.header.ide = false; // Use 29-bit extended ID format
  tx_msg.header.fdf = true;
  tx_msg.header.brs = true;
  tx_msg.header.rtr = false;
  tx_msg.buffer = data;
  tx_msg.buffer_len = size;
  ESP_ERROR_CHECK(twai_node_transmit(handle, &tx_msg, 0));
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
