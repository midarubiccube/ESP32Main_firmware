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

static const char TAG[] = "main";
twai_node_handle_t handle = NULL;
void send_can(int data);

void userCallback(std_msgs::msg::UInt16 *msg)
{
  MROS2_INFO("subscribed msg: '%d'", msg->data);
  send_can(msg->data);
}

void userCallback2(geometry_msgs::msg::Twist *msg)
{
  int send_data  = msg->linear.x < 0 ? ((int)msg->linear.x)*100 : 0
  MROS2_INFO("subscribed msg: '%f'",);
}
 

static QueueHandle_t rx_queue = NULL;

static bool IRAM_ATTR twai_listener_on_error_callback(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    ESP_LOGI(TAG, "bus error: arb_lost:%d bit_err:%d form_err:%d ", edata->err_flags.arb_lost, edata->err_flags.bit_err, edata->err_flags.form_err);
    return false;
}

// Node state
static bool IRAM_ATTR twai_listener_on_state_change_callback(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *user_ctx)
{
    const char *twai_state_name[] = {"error_active", "error_warning", "error_passive", "bus_off"};
    ESP_LOGI(TAG, "state changed: %s -> %s", twai_state_name[edata->old_sta], twai_state_name[edata->new_sta]);
    return false;
}

static bool IRAM_ATTR nmea_on_received(
    twai_node_handle_t handle,
    const twai_rx_done_event_data_t *edata,
    void *user_ctx
){
    return true;
}

static esp_err_t nmea_init(twai_node_handle_t *handle) {
    static twai_onchip_node_config_t node_config = {};
    node_config.io_cfg.tx = GPIO_NUM_10;
    node_config.io_cfg.rx = GPIO_NUM_9;
    node_config.bit_timing.bitrate = 1000000;
    node_config.data_timing.bitrate = 2000000;
    node_config.tx_queue_depth = 5,
    twai_new_node_onchip(&node_config, handle);
    
    twai_node_enable(*handle);
    return ESP_OK;
} 


void send_can(int data) {

    ESP_LOGI(TAG, "init");    
    ESP_LOGI(TAG, "init done");

    uint8_t send_buff[32] = {0};
    send_buff[0] = data;
    twai_frame_t tx_msg={};
    tx_msg.header.id = 15;           // Message ID
    tx_msg.header.ide = false;         // Use 29-bit extended ID format
    tx_msg.header.fdf = true;
    tx_msg.header.brs = true;
    tx_msg.buffer = send_buff;        // Pointer to data to transmit
    tx_msg.header.dlc = 13;
    ESP_ERROR_CHECK(twai_node_transmit(handle, &tx_msg, 0));
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
 