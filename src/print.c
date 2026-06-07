#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <stdarg.h>

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart21));

int df_print_init(void)
{
    // 初始化串口
	if (!device_is_ready(uart)){
	printk("UART device not ready\r\n");
	return 1 ;
	}
    else
    {
        return  0;
    }
}

int df_print(const char *fmt, ...)
{
    char buf[128];
    va_list args;

    va_start(args, fmt);
    int len = vsnprintk(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len < 0) {
        return len;
    }

    if (len >= sizeof(buf)) {
        len = sizeof(buf) - 1;
    }

    for (int i = 0; i < len; i++) {
        uart_poll_out(uart, buf[i]);
    }

    return len;
}