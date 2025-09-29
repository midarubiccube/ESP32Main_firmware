#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "mros2.h"
#include "mros2-platform.h"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "message.hpp"

static const char TAG[] = "main";
twai_node_handle_t handle = NULL;
void send_can(Message_format msg);

void userCallback(std_msgs::msg::UInt16 *msg)
{
  MROS2_INFO("subscribed msg: '%d'", msg->data);
  Message_format send = {0};
  send.id.format.to_id1 = 0;
  send.id.format.to_id2 = 0;
  send.id.format.to_type = 1;
  send.data.power_rsv.ON_OFF = msg->data;
  send_can(send);


}

void userCallback2(geometry_msgs::msg::Twist *msg)
{
  MROS2_INFO("cmd_vel msg: '%f'", msg->linear.x);
  Message_format send2 = {0};
  send2.id.format.to_id1 = 0;
  send2.id.format.to_id2 = 0;
  send2.id.format.to_type = 2;
  send2.is_remote = false;
  
  send2.data.motor_rsv.target = (int)((msg->linear.y + msg->linear.x) * 100.0);
  send_can(send2);
  send2.data.motor_rsv.target = (int)((msg->linear.y - msg->linear.x) * 100.0);
  send2.id.format.to_id2 = 1;
  send_can(send2);
  send2.data.motor_rsv.target = (int)((msg->linear.y + msg->linear.x) * -100.0);
  send2.id.format.to_id2 = 2;
  send_can(send2);
  send2.data.motor_rsv.target = (int)((msg->linear.y - msg->linear.x) * -100.0);
  send2.id.format.to_id2 = 3;
  send_can(send2);

}
 
static bool IRAM_ATTR nmea_on_received(
    twai_node_handle_t handle,
    const twai_rx_done_event_data_t *edata,
    void *user_ctx
){
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


void send_can(Message_format msg) {
  msg.id.format.from_id1 = 0;
  msg.id.format.from_id2 = 0;
  msg.id.format.from_type = 0;

  twai_frame_t tx_msg={};
  tx_msg.header.id = msg.id.id;           // Message ID
  tx_msg.header.ide = true;         // Use 29-bit extended ID format
  tx_msg.header.fdf = true;
  tx_msg.header.brs = true;
  tx_msg.header.rtr = msg.is_remote;
  tx_msg.buffer = (uint8_t *)msg.data.data;        // Pointer to data to transmit
  tx_msg.buffer_len = 32;
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
 