#include <Arduino.h>
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include "pico/util/queue.h"

// ==========================================
// 硬件配置与全局变量
// ==========================================

// 定义 PIO USB 引脚 (连接物理鼠标)
// D+ 接 GPIO 0, D- 接 GPIO 1
#define PIN_USB_HOST_DP 0 

// USB Host 实例 (Core 1 用)
Adafruit_USBH_Host USBHost;

// USB Device 实例 (发往电脑, Core 0 用)
// 默认包含标准的鼠标和键盘报告描述符
uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_MOUSE()
};
Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_MOUSE, 2, false);

// 双核通信队列：存储鼠标数据的结构体
typedef struct {
  uint8_t buttons; // 按键状态掩码
  int8_t x;        // X轴相对移动量
  int8_t y;        // Y轴相对移动量
  int8_t wheel;    // 滚轮滚动量
  int8_t pan;      // 横向滚轮(部分鼠标支持)
} mouse_data_t;

// 定义跨核安全队列
queue_t mouse_queue;


// ==========================================
// Core 0: 主逻辑与 USB Device (发往电脑)
// ==========================================

// --- 函数映射逻辑 (留空待后续填充) ---
void map_mouse_data(mouse_data_t *data) {
  // [示例代码] 此处可以修改传入的 data 指针中的数据
  // 
  // 1. 灵敏度加倍:
  //    data->x = data->x * 2;
  //    data->y = data->y * 2;
  // 
  // 2. 按键映射 (如: 按下鼠标中键(0x04)时，模拟左键(0x01)):
  //    if (data->buttons & 0x04) {
  //      data->buttons |= 0x01; 
  //    }
 
  // TODO: 在这里填充你的映射逻辑...
}

void setup() {
  Serial.begin(115200);

  // 初始化双核通信队列，最大缓存 20 个鼠标事件 (足够处理极高回报率)
  queue_init(&mouse_queue, sizeof(mouse_data_t), 20);

  // 初始化 USB Device (作为鼠标连入电脑)
  usb_hid.begin();

  // 等待 USB 设备准备就绪 (可选)
  while( !TinyUSBDevice.mounted() ) delay(1);
}

void loop() {
  mouse_data_t incoming_data;

  // 非阻塞尝试从 Core 1 获取最新的鼠标数据
  if (queue_try_remove(&mouse_queue, &incoming_data)) {
   
    // 1. 进行信号映射与处理
    map_mouse_data(&incoming_data);

    // 2. 检查设备状态并发送给电脑
    if (usb_hid.ready()) {
      // 这里的 0 是 report_id (对于纯鼠标描述符默认0即可)
      usb_hid.mouseReport(0, 
                          incoming_data.buttons, 
                          incoming_data.x, 
                          incoming_data.y, 
                          incoming_data.wheel, 
                          incoming_data.pan);
    }
  }
}


// ==========================================
// Core 1: USB Host (读取物理鼠标)
// ==========================================

void setup1() {
  // 延迟启动确保 Core 0 初始化队列完毕
  delay(10); 

  // 配置 PIO USB 为 Host 模式
  static pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_USB_HOST_DP;

  // 在 Core 1 上初始化 PIO USB 和 Host 控制器
  USBHost.configure_pio_usb(1, &pio_cfg);
  USBHost.begin(1);
}

void loop1() {
  // 持续运行 TinyUSB Host 任务 (不可阻塞，不能使用 delay)
  USBHost.task();
}

// ==========================================
// TinyUSB Host 回调函数 (运行在 Core 1 上下文)
// ==========================================
extern "C" {

  // 当 USB 设备(鼠标)接入并被挂载时触发
  void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len) {
    // 发起第一次读取请求
    tuh_hid_receive_report(dev_addr, instance);
  }

  // 当接收到 USB 设备的报文时触发
  void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    // 检查是否是鼠标设备
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
   
    if (itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
      // 将原始数据转换为标准鼠标报文结构
      // 注意：标准Boot鼠标报文通常前4-5字节结构一致
      hid_mouse_report_t const * report_data = (hid_mouse_report_t const *) report;

      mouse_data_t new_data;
      new_data.buttons = report_data->buttons;
      new_data.x       = report_data->x;
      new_data.y       = report_data->y;
      new_data.wheel   = report_data->wheel;
      new_data.pan     = report_data->pan;

      // 将解析后的数据推入跨核安全队列 (如果队列满了，尝试非阻塞丢弃/跳过)
      queue_try_add(&mouse_queue, &new_data);
    }

    // 必须请求下一个报文，否则无法接收后续移动
    tuh_hid_receive_report(dev_addr, instance);
  }

}