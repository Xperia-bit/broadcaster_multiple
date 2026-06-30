#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include "print.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <nrf_sys_event.h>


int broadcaster_multiple(void);
int system_init(void);
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

// 使能引脚
#define SENSOR_NODE DT_ALIAS(sensorsw)
static const struct gpio_dt_spec sensor_enable = GPIO_DT_SPEC_GET(SENSOR_NODE, gpios);

// pwr-key
#define PWR_NODE	DT_ALIAS(pwrkey) 
static const struct gpio_dt_spec pwr_key = GPIO_DT_SPEC_GET(PWR_NODE, gpios);
static struct gpio_callback pwr_key_cb_data;		//按键事件回调
static uint32_t last_button_press_ms;				//消抖
#define BUTTON_DEBOUNCE_MS	20

int main(void)
{
	df_print_init();
	int err;
	k_msleep(4000);

	df_print("Starting Multiple Broadcaster Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		df_print("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	(void)broadcaster_multiple();

	df_print("Exiting %s thread.\n", __func__);
	return 0;
}

int system_init(void)
{
	// 初始化使能引脚
	gpio_is_ready_dt(&sensor_enable);
	gpio_pin_configure_dt(&sensor_enable, GPIO_OUTPUT_INACTIVE);
	// 初始化pwr-key
	device_is_ready(pwr_key.port);
	gpio_pin_configure_dt(&pwr_key, GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_interrupt_configure_dt(&pwr_key, GPIO_INT_EDGE_TO_ACTIVE);		//设置button的中断模式->按下激活时触发
	gpio_init_callback(&pwr_key_cb_data, button_pressed, BIT(pwr_key.pin)); 	
	gpio_add_callback(pwr_key.port, &pwr_key_cb_data);
	df_print("system init ok\n");
	// 使能传感器电源
	// gpio_pin_set_dt(&sensor_enable, 1);
	// k_msleep(1000);
	return 1;
}

// 把中断执行的任务放在另一个队列中执行，不占用中断
void button_work(struct k_work *work)
{
	df_print("button press!\n");
};
K_WORK_DEFINE(button_wk,button_work);
// 按键中断回调函数
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	uint32_t now = k_uptime_get_32();

	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if ((now - last_button_press_ms) < BUTTON_DEBOUNCE_MS) {
		return;
	}

	last_button_press_ms = now;
	k_work_submit(&button_wk);
}
