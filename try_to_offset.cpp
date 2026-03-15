#include <Arduino.h>
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include "pico/util/queue.h"
#include <math.h>

// ==========================================
// 硬件配置与 USB 描述符
// ==========================================

// 定义 PIO USB 引脚 (连接物理鼠标)
// D+ 接 GPIO 0, D- 接 GPIO 1
#define PIN_USB_HOST_DP 0 

// USB Host 实例 (Core 1 用，读取物理鼠标)
Adafruit_USBH_Host USBHost;

// USB Device 实例 (发往电脑, Core 0 用)
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
  int8_t pan;      // 横向滚轮
} mouse_data_t;

// 跨核安全队列
queue_t mouse_queue;

// ==========================================
// 反加速映射核心数据与状态变量
// ==========================================

// 反加速映射表 (LUT)，存储 1/P(v) 的结果
float reverse_accel_lut[128];

// 你的物理鼠标实际 DPI
const float MOUSE_DPI = 1200.0f;

// 状态缓存变量 (跨帧保留数据)
static float remainder_x = 0.0f;
static float remainder_y = 0.0f;
static float smoothed_v = 0.0f;

// 初始化 libinput 抵消映射表
void init_anti_accel_lut() {
    // libinput 默认将所有鼠标归一化到 1000 DPI
    float dpi_scale = 1000.0f / MOUSE_DPI;
    
    // libinput 默认参数 (对应系统滑块 0.0)
    float threshold = 0.4f;
    float incline = 1.1f;
    float accel_max = 2.0f;

    for (int i = 0; i < 128; i++) {
        // 假设 1000Hz 回报率，i 代表单次报告的物理位移(像素)，即速度 units/ms
        float v_norm = (float)i * dpi_scale;
        float p_v;

        if (v_norm < 0.07f) {
            // 低速减速区 (补偿粘滞感)
            p_v = 10.0f * v_norm + 0.3f;
        } else if (v_norm < threshold) {
            // 1:1 线性区
            p_v = 1.0f;
        } else {
            // 加速爬坡区
            p_v = incline * (v_norm - threshold) + 1.0f;
        }

        // 封顶限制
        if (p_v > accel_max) {
            p_v = accel_max;
        }

        // 存入倒数 1/P(v)
        reverse_accel_lut[i] = 1.0f / p_v;
    }
    
    // 强制修正 0 速度点的倍率，防止除 0 或异常
    reverse_accel_lut[0] = 1.0f;
}

// 核心映射逻辑
void map_mouse_data(mouse_data_t *data) {
  // 如果没有任何移动，仅衰减历史速度并返回
  if (data->x == 0 && data->y == 0) {
    smoothed_v = smoothed_v * 0.7f;
    return;
  }

  // 1. 计算当前原始位移的模长 (瞬时物理速度)
  float v_raw = sqrtf((float)data->x * data->x + (float)data->y * data->y);

  // 2. 速度平滑滤波 (模拟 libinput 内部的历史追踪器)
  smoothed_v = smoothed_v * 0.7f + v_raw * 0.3f;

  // 3. 查表获取抵消系数 1/P(v)
  int index = (int)smoothed_v;
  if (index > 127) index = 127; 
  if (index < 0) index = 0;
  float reverse_factor = reverse_accel_lut[index];

  // 4. 计算理想浮点位移
  float ideal_x = (float)data->x * reverse_factor + remainder_x;
  float ideal_y = (float)data->y * reverse_factor + remainder_y;

  // 5. 截断为整数以通过 USB 发送
  int8_t out_x = (int8_t)ideal_x;
  int8_t out_y = (int8_t)ideal_y;

  // 6. 将丢失的小数部分存入余数池
  remainder_x = ideal_x - (float)out_x;
  remainder_y = ideal_y - (float)out_y;

  // 7. 回写处理后的数据
  data->x = out_x;
  data->y = out_y;
}

// ==========================================
// Core 0: 主逻辑与 USB Device (发往电脑)
// ==========================================

void setup() {
  Serial.begin(115200);

  // 初始化反加速曲线 LUT
  init_anti_accel_lut();

  // 初始化双核通信队列，最大缓存 20 个鼠标事件
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
    
    // 1. 进行信号映射与处理 (反加速)
    map_mouse_data(&incoming_data);

    // 2. 检查设备状态并发送给电脑
    if (usb_hid.ready()) {
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
  // 持续运行 TinyUSB Host 任务 (不可阻塞)
  USBHost.task();
}

// ==========================================
// TinyUSB Host 回调函数 (运行在 Core 1 上下文)
// ==========================================
extern "C" {

  void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len) {
    // 发起第一次读取请求
    tuh_hid_receive_report(dev_addr, instance);
  }

  void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    
    if (itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
      hid_mouse_report_t const * report_data = (hid_mouse_report_t const *) report;

      mouse_data_t new_data;
      new_data.buttons = report_data->buttons;
      new_data.x       = report_data->x;
      new_data.y       = report_data->y;
      new_data.wheel   = report_data->wheel;
      new_data.pan     = report_data->pan;

      // 推入队列交由 Core 0 处理
      queue_try_add(&mouse_queue, &new_data);
    }

    // 继续请求下一个报文
    tuh_hid_receive_report(dev_addr, instance);
  }

}
