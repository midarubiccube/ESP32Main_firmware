#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <math.h>

#include "mros2.h"
#include "mros2-platform.h"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "messageFormat/motorBoard.hpp"
#include "messageFormat/powerboard.hpp"

struct canfd_frame
{
  uint16_t identifier;
  uint8_t data[64];
  size_t length;
  bool is_remote;
};

#define DEG_TO_RAD(deg) ((deg) / 180.0 * M_PI) // 度からラジアンへの変換

static QueueHandle_t rx_queue = NULL;

static const char TAG[] = "main";
void send_can(ID_Format id, uint8_t *data, size_t size, bool is_remote);

void userCallback(std_msgs::msg::UInt16 *msg)
{
  PowerBoard_Target sendmsg = {0};
  ID_Format id;
  id.format.to_BoardType = Board_Type::PowerBoard;
  id.format.to_BoardID = 0;
  id.format.message_type = Message_Type::Target;
  sendmsg.ON_OFF = msg->data;
  MROS2_INFO("power %s", msg->data ? "on" : "off");

  send_can(id, reinterpret_cast<uint8_t *>(&sendmsg), sizeof(PowerBoard_Target), false);
}

void userCallback2(geometry_msgs::msg::Twist *msg)
{
  ID_Format id;
  id.format.to_BoardType = Board_Type::MotorBoard;
  id.format.to_BoardID = 0;
  id.format.message_type = Message_Type::Target;

  MotorBoard_Target sendmsg;
  sendmsg.mode = ControlMode::PWM_Mode;
  sendmsg.target[0] = static_cast<int>(sinf(DEG_TO_RAD(msg->linear.x)) * msg->linear.y + msg->linear.z);
  sendmsg.target[1] = static_cast<int>(sinf(DEG_TO_RAD(msg->linear.x - 90.0f)) *  msg->linear.y + msg->linear.z);
  sendmsg.target[2] = static_cast<int>(sinf(DEG_TO_RAD(msg->linear.x - 180.0f)) * msg->linear.y + msg->linear.z);
  sendmsg.target[3] = static_cast<int>(sinf(DEG_TO_RAD(msg->linear.x - 270.0f)) * msg->linear.y + msg->linear.z);
  send_can(id, reinterpret_cast<uint8_t *>(&sendmsg), sizeof(MotorBoard_Target), false);

  id.format.to_BoardID = 1;
  sendmsg.mode = ControlMode::PWM_Mode;
  sendmsg.target[3] = static_cast<int>(msg->angular.x * 500.0f);
  send_can(id, reinterpret_cast<uint8_t *>(&sendmsg), sizeof(MotorBoard_Target), false);

  id.format.to_BoardID = 2;
  sendmsg.mode = ControlMode::PWM_Mode;
  sendmsg.target[3] = static_cast<int>(msg->angular.y * 500.0f);
  sendmsg.target[1] = static_cast<int>(msg->angular.z);
  send_can(id, reinterpret_cast<uint8_t *>(&sendmsg), sizeof(MotorBoard_Target), false);
  
}

static esp_err_t queues_init()
{
  rx_queue = xQueueCreate(100, sizeof(canfd_frame));

  if (rx_queue == NULL)
  {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

static bool canfd_on_received(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
  canfd_frame frame = {0};
  BaseType_t woken = pdFALSE;

  twai_frame_t rx_frame;
  rx_frame.buffer = frame.data,
  rx_frame.buffer_len = sizeof(frame.data) / sizeof(uint8_t);

  if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK)
  {
    frame.identifier = rx_frame.header.id;
    frame.length = rx_frame.header.dlc;
    xQueueSendFromISR(rx_queue, &frame, &woken);
  }
  return woken == pdTRUE;
}

static esp_err_t canfd_init(twai_node_handle_t *handle)
{
  twai_onchip_node_config_t node_config = {};
  node_config.io_cfg.tx = GPIO_NUM_10;
  node_config.io_cfg.rx = GPIO_NUM_9;
  node_config.bit_timing.bitrate = 1000000;
  node_config.data_timing.bitrate = 2000000;
  node_config.tx_queue_depth = 100,
  twai_new_node_onchip(&node_config, handle);

  twai_event_callbacks_t callbacks = {};
  callbacks.on_rx_done = canfd_on_received;

  twai_node_register_event_callbacks(*handle, &callbacks, NULL);

  twai_node_enable(*handle);
  return ESP_OK;
}

twai_node_handle_t handle = NULL;

void send_can(ID_Format id, uint8_t *data, size_t size, bool is_remote)
{
  id.format.from_BoardType = Board_Type::Master_Board;
  id.format.from_BoardID = 0;

  twai_frame_t tx_msg = {};
  tx_msg.header.id = id.id; // Message ID
  tx_msg.header.ide = true; // Use 29-bit extended ID format
  tx_msg.header.fdf = true;
  tx_msg.header.brs = true;
  tx_msg.header.rtr = is_remote;
  tx_msg.buffer = data;
  tx_msg.buffer_len = size;
  ESP_ERROR_CHECK(twai_node_transmit(handle, &tx_msg, 0));
  osDelay(10);
}

TaskHandle_t rsvHandle = NULL;

extern "C" void app_main(void)
{
  ESP_ERROR_CHECK(canfd_init(&handle));
  ESP_ERROR_CHECK(queues_init());

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
  mros2::Publisher pub = node.create_publisher<std_msgs::msg::Float32>("to_linux", 10);
  mros2::Subscriber sub2 = node.create_subscription<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10, userCallback2);
  osDelay(100);
  osThreadAttr_t attributes;

  canfd_frame received_frame = {0};
  for (;;)
  {
    BaseType_t ret = xQueueReceive(rx_queue, &received_frame, portMAX_DELAY);

    if (ret == pdTRUE)
    {
      ID_Format id;
      id.id = received_frame.identifier;
      if (id.format.to_BoardType == Board_Type::Master_Board && id.format.from_BoardType == Board_Type::PowerBoard)
      {
        auto status = reinterpret_cast<PowerBoard_Status *>(received_frame.data);
        std_msgs::msg::Float32 msg;
        msg.data = status->Current;
          pub.publish(msg);

        printf("Current %f\n", status->Current);
      }
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}
