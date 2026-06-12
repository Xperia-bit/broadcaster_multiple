/*
 * broadcaster_multiple.c
 *
 * 功能概述：
 * 1. 创建两个 BLE 扩展广播集（Extended Advertising Set）：
 *    - beacon_adv：不可连接广播，用来持续发送 Beacon 信息。
 *    - config_adv：可连接广播，用来让手机连接后通过 GATT 修改 Beacon 参数。
 *
 * 2. Beacon 广播内容中包含 Manufacturer Data：
 *    - Company ID
 *    - Frame Type
 *    - 当前 Beacon 广播间隔 interval_ms
 *    - 当前实际设置成功的发射功率 tx_power_dbm
 *
 * 3. 手机连接 config_adv 后，可以通过自定义 GATT Service 读写两个特征值：
 *    - Beacon 广播间隔，单位 ms，uint16_t，小端格式
 *    - Beacon 发射功率，单位 dBm，int8_t
 *
 * 4. 当手机写入新的广播间隔或发射功率后，代码会重新应用 Beacon 广播参数，
 *    并刷新 Beacon 广播包里的 Manufacturer Data。
 *
 * 注意：
 * 该代码依赖 Zephyr 蓝牙协议栈，并且使用了 HCI Vendor Extension 来设置广播发射功率。
 */

#include <zephyr/kernel.h>          /* Zephyr 内核基础接口，例如 k_work、K_WORK_DEFINE 等 */
#include <zephyr/sys/byteorder.h>   /* 字节序转换接口，例如 sys_put_le16、sys_get_le16 */
#include <zephyr/sys/printk.h>      /* Zephyr printk 调试输出 */

#include <zephyr/bluetooth/bluetooth.h> /* 蓝牙核心功能 */
#include <zephyr/bluetooth/conn.h>      /* BLE 连接相关接口和回调 */
#include <zephyr/bluetooth/gatt.h>      /* GATT 服务、特征值读写相关接口 */
#include <zephyr/bluetooth/hci.h>       /* HCI 命令和错误码相关接口 */
#include <zephyr/bluetooth/hci_vs.h>    /* Zephyr / Nordic 等平台的 HCI Vendor Specific 扩展 */
#include <zephyr/bluetooth/uuid.h>      /* UUID 定义和初始化相关接口 */

#include "print.h"                     /* 项目自定义打印接口 df_print() */

/*
 * 编译期检查：必须启用 CONFIG_BT_HAS_HCI_VS。
 *
 * 本文件中的 adv_tx_power_set() 会调用 HCI Vendor Specific 命令：
 * BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL。
 * 如果工程配置里没有打开 HCI Vendor Extension，编译时直接报错，避免运行时才发现不支持。
 */
BUILD_ASSERT(IS_ENABLED(CONFIG_BT_HAS_HCI_VS),
	     "This application requires Zephyr HCI vendor extensions");

/* Beacon 不可连接广播的间隔范围，单位 ms。
 * BLE 广播间隔本质单位是 0.625 ms，这里对外用 ms，便于手机端配置。
 */
#define BEACON_ADV_INTERVAL_MS_MIN      20U
#define BEACON_ADV_INTERVAL_MS_MAX      10240U
#define BEACON_ADV_INTERVAL_MS_DEFAULT  200U

/* config_adv 可连接广播的间隔范围，单位 ms。
 * 这个广播用于让手机发现并连接设备，所以设置为 100~150 ms，连接体验会更快。
 */
#define CONFIG_ADV_INTERVAL_MS_MIN      100U
#define CONFIG_ADV_INTERVAL_MS_MAX      150U

/* Beacon 广播名称。会放进 beacon_ad 里作为 Complete Local Name。 */
#define BEACON_NAME                     "Beacon Test"

/* Manufacturer Data 中的公司 ID。
 * 0xFFFF 通常用于测试或自定义场景，正式产品建议申请/使用合法 Company ID。
 */
#define BEACON_MFG_COMPANY_ID           0xFFFF

/* Manufacturer Data 中的帧类型，用于区分自定义数据格式。 */
#define BEACON_MFG_FRAME_TYPE           0x01

/*
 * 自定义 GATT 服务 UUID。
 *
 * 注意 Zephyr 的 BT_UUID_INIT_128() 使用的是小端字节数组写法。
 * 也就是说，这里看到的字节顺序和手机 App 上显示的标准 UUID 字符串顺序可能是反过来的。
 */
#define BEACON_CFG_SERVICE_UUID_BYTES   \
	0xF0, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, \
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12

/*
 * 自定义 GATT 特征值 UUID：Beacon 广播间隔。
 * 手机可以读/写该特征值，写入格式是 uint16_t，小端，单位 ms。
 */
#define BEACON_CFG_INTERVAL_UUID_BYTES  \
	0xF1, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, \
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12

/*
 * 自定义 GATT 特征值 UUID：Beacon 发射功率。
 * 手机可以读/写该特征值，写入格式是 int8_t，单位 dBm。
 */
#define BEACON_CFG_TX_POWER_UUID_BYTES  \
	0xF2, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, \
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12

/*
 * 运行时 Beacon 配置结构体。
 * interval_ms：Beacon 广播间隔，单位 ms。
 * tx_power_dbm：Beacon 广播发射功率，单位 dBm。
 */
struct beacon_runtime_cfg {
	uint16_t interval_ms;
	int8_t tx_power_dbm;
};

/*
 * beacon_adv：不可连接 Beacon 广播集句柄。
 * config_adv：可连接配置广播集句柄。
 * config_conn：当前手机连接对象，用于引用计数管理。
 */
static struct bt_le_ext_adv *beacon_adv;
static struct bt_le_ext_adv *config_adv;
static struct bt_conn *config_conn;

/* Beacon 默认运行参数。设备启动后默认 200 ms 广播一次，发射功率先设为 0 dBm。 */
static struct beacon_runtime_cfg beacon_cfg = {
	.interval_ms = BEACON_ADV_INTERVAL_MS_DEFAULT,
	.tx_power_dbm = 0,
};

/* config_adv 重启工作函数的前置声明。
 * 断开连接后会提交这个 work，让可连接广播重新启动。
 */
static void config_adv_restart_work_handler(struct k_work *work);

/* 定义一个 Zephyr work item。
 * 好处是：断开连接回调中不直接做复杂操作，而是把重启广播动作放到系统 workqueue 中执行。
 */
static K_WORK_DEFINE(config_adv_restart_work, config_adv_restart_work_handler);

/* 把 128-bit UUID 字节数组初始化为 Zephyr UUID 对象，后面注册 GATT 服务和特征值会用到。 */
static struct bt_uuid_128 beacon_cfg_service_uuid =
	BT_UUID_INIT_128(BEACON_CFG_SERVICE_UUID_BYTES);
static struct bt_uuid_128 beacon_cfg_interval_uuid =
	BT_UUID_INIT_128(BEACON_CFG_INTERVAL_UUID_BYTES);
static struct bt_uuid_128 beacon_cfg_tx_power_uuid =
	BT_UUID_INIT_128(BEACON_CFG_TX_POWER_UUID_BYTES);

/*
 * Beacon Manufacturer Data 自定义数据格式：
 *
 * byte[0..1]  Company ID，小端格式，例如 0xFFFF 会存成 FF FF
 * byte[2]     Frame type，用于标识当前帧类型
 * byte[3..4]  Beacon interval，单位 ms，小端 uint16_t
 * byte[5]     Applied TX power，实际应用成功的发射功率，单位 dBm，int8_t
 */
static uint8_t beacon_mfg_data[] = {
	(BEACON_MFG_COMPANY_ID & 0xFF),          /* Company ID 低字节 */
	(BEACON_MFG_COMPANY_ID >> 8),            /* Company ID 高字节 */
	BEACON_MFG_FRAME_TYPE,                   /* 自定义帧类型 */
	(BEACON_ADV_INTERVAL_MS_DEFAULT & 0xFF), /* 默认广播间隔低字节 */
	(BEACON_ADV_INTERVAL_MS_DEFAULT >> 8),   /* 默认广播间隔高字节 */
	0,                                       /* 默认发射功率，后续会刷新 */
};

/*
 * Beacon 广播数据。
 *
 * 这是不可连接广播 beacon_adv 发出去的数据，包含：
 * 1. Flags：声明不支持 BR/EDR，即只支持 BLE。
 * 2. Manufacturer Data：自定义 Beacon 数据。
 * 3. Complete Local Name：设备名 "Beacon Test"。
 */
static const struct bt_data beacon_ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, beacon_mfg_data, sizeof(beacon_mfg_data)),
	BT_DATA(BT_DATA_NAME_COMPLETE, BEACON_NAME, sizeof(BEACON_NAME) - 1),
};

/*
 * config_adv 的广播数据。
 *
 * 这是可连接广播，主要让手机扫描时知道：
 * “这个设备提供 BEACON_CFG_SERVICE_UUID_BYTES 这个自定义配置服务”。
 */
static const struct bt_data config_ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BEACON_CFG_SERVICE_UUID_BYTES),
};

/*
 * config_adv 的扫描响应数据。
 *
 * 广播包空间有限，所以把设备名称放到 Scan Response 里。
 * CONFIG_BT_DEVICE_NAME 来自 prj.conf 或 Kconfig 配置。
 */
static const struct bt_data config_sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/*
 * 把毫秒单位的广播间隔转换成 BLE 广播间隔单位。
 *
 * BLE 广播间隔单位是 0.625 ms，即 625 us。
 * 公式：units = interval_ms * 1000 / 625
 * 例如：200 ms = 200000 us / 625 us = 320 units。
 */
static uint16_t adv_interval_ms_to_units(uint16_t interval_ms)
{
	uint32_t units = ((uint32_t)interval_ms * 1000U) / 625U;

	return (uint16_t)units;
}

/* 检查 Beacon 广播间隔是否合法，避免手机写入过小或过大的值。 */
static bool beacon_interval_is_valid(uint16_t interval_ms)
{
	return (interval_ms >= BEACON_ADV_INTERVAL_MS_MIN) &&
	       (interval_ms <= BEACON_ADV_INTERVAL_MS_MAX);
}

/*
 * 根据当前 beacon_cfg 刷新 beacon_mfg_data。
 *
 * 这个函数只修改 Manufacturer Data 的动态字段：
 * - byte[3..4]：interval_ms，小端
 * - byte[5]：tx_power_dbm
 *
 * 注意：修改数组本身还不等于广播包已经更新，后面还需要调用 bt_le_ext_adv_set_data()。
 */
static void beacon_payload_refresh(void)
{
	sys_put_le16(beacon_cfg.interval_ms, &beacon_mfg_data[3]);
	beacon_mfg_data[5] = (uint8_t)beacon_cfg.tx_power_dbm;
}

/*
 * 构造 Beacon 不可连接广播参数。
 *
 * id：使用默认蓝牙身份。
 * sid：广播集 ID，这里 beacon_adv 使用 0。
 * options：BT_LE_ADV_OPT_NONE 表示不可连接、不可扫描等默认扩展广播行为。
 * interval_min/max：这里设成相同值，表示固定广播间隔。
 */
static struct bt_le_adv_param beacon_adv_param_build(uint16_t interval_ms)
{
	uint16_t interval_units = adv_interval_ms_to_units(interval_ms);

	return (struct bt_le_adv_param) {
		.id = BT_ID_DEFAULT,
		.sid = 0U,
		.options = BT_LE_ADV_OPT_NONE,
		.interval_min = interval_units,
		.interval_max = interval_units,
	};
}

/*
 * 构造手机配置用的可连接广播参数。
 *
 * sid：这里 config_adv 使用 1，与 beacon_adv 的 sid=0 区分。
 * options：BT_LE_ADV_OPT_CONN 表示这个广播集允许手机连接。
 */
static struct bt_le_adv_param config_adv_param_build(void)
{
	return (struct bt_le_adv_param) {
		.id = BT_ID_DEFAULT,
		.sid = 1U,
		.options = BT_LE_ADV_OPT_CONN,
		.interval_min = BT_GAP_MS_TO_ADV_INTERVAL(CONFIG_ADV_INTERVAL_MS_MIN),
		.interval_max = BT_GAP_MS_TO_ADV_INTERVAL(CONFIG_ADV_INTERVAL_MS_MAX),
	};
}

/*
 * 设置某个扩展广播集的发射功率。
 *
 * 参数：
 * adv：要设置的广播集句柄，例如 beacon_adv。
 * tx_power_dbm：输入为希望设置的功率，输出会被改成协议栈/控制器实际选择的功率。
 *
 * 关键点：
 * 1. 这里使用 HCI Vendor Specific 命令，不是标准 GATT 操作。
 * 2. 不是所有芯片都能精确设置任意 dBm，控制器可能会选择最接近的可用功率。
 * 3. 所以函数返回后，*tx_power_dbm 会更新为 selected_tx_power。
 */
static int adv_tx_power_set(struct bt_le_ext_adv *adv, int8_t *tx_power_dbm)
{
	struct bt_hci_cp_vs_write_tx_power_level *cp; /* HCI 命令参数结构体 */
	struct bt_hci_rp_vs_write_tx_power_level *rp; /* HCI 命令响应结构体 */
	struct net_buf *buf;                           /* HCI 命令发送缓冲区 */
	struct net_buf *rsp = NULL;                    /* HCI 命令响应缓冲区 */
	int err;

	/* 参数保护，避免空指针。 */
	if ((adv == NULL) || (tx_power_dbm == NULL)) {
		return -EINVAL;
	}

	/* 申请一个 HCI 命令缓冲区，K_FOREVER 表示一直等到申请成功。 */
	buf = bt_hci_cmd_alloc(K_FOREVER);
	if (buf == NULL) {
		return -ENOMEM;
	}

	/* 在 HCI 命令缓冲区中追加命令参数。 */
	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle_type = BT_HCI_VS_LL_HANDLE_TYPE_ADV;                 /* 设置对象是广播 */
	cp->handle = sys_cpu_to_le16(bt_le_ext_adv_get_index(adv));     /* 广播集 handle/index */
	cp->tx_power_level = *tx_power_dbm;                              /* 期望设置的功率 */

	/* 同步发送 HCI 命令，并等待控制器返回响应。 */
	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, buf, &rsp);
	if (err) {
		return err;
	}

	/* 从响应中取出控制器实际选择的功率值。 */
	rp = (struct bt_hci_rp_vs_write_tx_power_level *)rsp->data;
	*tx_power_dbm = rp->selected_tx_power;

	/* 释放响应缓冲区，避免内存泄漏。 */
	net_buf_unref(rsp);

	return 0;
}

/*
 * 应用新的 Beacon 配置。
 *
 * 这是本文件里最核心的函数之一。无论手机修改广播间隔还是修改发射功率，
 * 最后都会调用这个函数，让配置真正生效。
 *
 * 执行流程：
 * 1. 检查参数。
 * 2. 检查新的广播间隔是否合法。
 * 3. 根据新的 interval_ms 构造广播参数。
 * 4. 停止当前 Beacon 广播。
 * 5. 更新广播参数。
 * 6. 更新本地 beacon_cfg，并刷新 Manufacturer Data。
 * 7. 更新广播数据。
 * 8. 重新启动 Beacon 广播。
 * 9. 设置发射功率。
 * 10. 把实际设置成功的功率写回 beacon_cfg，再次刷新广播数据。
 */
static int beacon_adv_apply(struct beacon_runtime_cfg *new_cfg)
{
	struct bt_le_adv_param param;
	int8_t applied_tx_power;
	int err;

	if ((beacon_adv == NULL) || (new_cfg == NULL)) {
		return -EINVAL;
	}

	if (!beacon_interval_is_valid(new_cfg->interval_ms)) {
		return -EINVAL;
	}

	/* 根据新的广播间隔生成 Zephyr 广播参数。 */
	param = beacon_adv_param_build(new_cfg->interval_ms);

	/* 更新广播参数前，先停止当前 Beacon 广播。 */
	err = bt_le_ext_adv_stop(beacon_adv);
	if (err) {
		df_print("Failed to stop beacon advertising (err %d)\n", err);
		return err;
	}

	/* 更新 Beacon 广播参数，例如 interval_min / interval_max。 */
	err = bt_le_ext_adv_update_param(beacon_adv, &param);
	if (err) {
		df_print("Failed to update beacon advertising param (err %d)\n", err);
		return err;
	}

	/* 先把新配置保存到全局变量，再刷新 Manufacturer Data 中的动态字段。 */
	beacon_cfg = *new_cfg;
	beacon_payload_refresh();

	/* 把刷新后的 Manufacturer Data 重新设置到 Beacon 广播包中。 */
	err = bt_le_ext_adv_set_data(beacon_adv, beacon_ad, ARRAY_SIZE(beacon_ad), NULL, 0);
	if (err) {
		df_print("Failed to update beacon advertising data (err %d)\n", err);
		return err;
	}

	/* 重新启动 Beacon 广播。 */
	err = bt_le_ext_adv_start(beacon_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		df_print("Failed to restart beacon advertising (err %d)\n", err);
		return err;
	}

	/* 设置发射功率。注意控制器可能不会完全按请求值设置，所以要读取实际值。 */
	applied_tx_power = beacon_cfg.tx_power_dbm;
	err = adv_tx_power_set(beacon_adv, &applied_tx_power);
	if (err) {
		df_print("Failed to apply beacon TX power (err %d)\n", err);
		return err;
	}

	/* 保存实际设置成功的功率，而不是手机请求的原始功率。 */
	beacon_cfg.tx_power_dbm = applied_tx_power;
	beacon_payload_refresh();

	/* 再次更新广播数据，让 Manufacturer Data 里的 tx_power_dbm 也变成实际功率。 */
	err = bt_le_ext_adv_set_data(beacon_adv, beacon_ad, ARRAY_SIZE(beacon_ad), NULL, 0);
	if (err) {
		df_print("Failed to refresh beacon payload (err %d)\n", err);
		return err;
	}

	df_print("Beacon updated: interval=%u ms, tx_power=%d dBm\n",
	       beacon_cfg.interval_ms, beacon_cfg.tx_power_dbm);

	return 0;
}

/* 启动 config_adv 可连接广播，让手机可以扫描并连接。 */
static int config_adv_start(void)
{
	int err;

	if (config_adv == NULL) {
		return -EINVAL;
	}

	err = bt_le_ext_adv_start(config_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if ((err != 0) && (err != -EALREADY)) {
		df_print("Failed to start config advertising (err %d)\n", err);
		return err;
	}

	return 0;
}

/*
 * config_adv 重启工作函数。
 *
 * 手机断开连接后，Zephyr 可能不会自动重新开启可连接广播。
 * 所以在 disconnected() 回调里提交这个 work，重新启动 config_adv。
 */
static void config_adv_restart_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	(void)config_adv_start();
}

/*
 * config_adv 广播集自己的 connected 回调。
 *
 * 这个回调属于扩展广播集回调，用来确认是哪个广告集被连接了。
 * 真正的 BLE 连接管理仍然在下面的 BT_CONN_CB_DEFINE(conn_callbacks) 里处理。
 */
static void config_adv_connected_cb(struct bt_le_ext_adv *adv,
				       struct bt_le_ext_adv_connected_info *info)
{
	df_print("Config advertiser[%u] connected, conn %p\n",
	       bt_le_ext_adv_get_index(adv), info->conn);
}

/* config_adv 的扩展广播回调集合。这里只关心 connected 事件。 */
static const struct bt_le_ext_adv_cb config_adv_cb = {
	.connected = config_adv_connected_cb,
};

/*
 * BLE 全局连接成功/失败回调。
 *
 * conn：连接对象。
 * err：0 表示连接成功，非 0 表示连接失败。
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		df_print("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		return;
	}

	/* 把连接对端地址转换成字符串，方便打印调试。 */
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	/* 保存当前连接对象引用。
	 * bt_conn_ref() 会增加引用计数，后续断开时必须 bt_conn_unref()。
	 */
	if (config_conn == NULL) {
		config_conn = bt_conn_ref(conn);
	}

	df_print("Phone connected: %s\n", addr);
}

/* BLE 全局断开连接回调。 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	df_print("Phone disconnected: %s, reason 0x%02x %s\n",
	       addr, reason, bt_hci_err_to_str(reason));

	/* 释放之前在 connected() 中保存的连接引用。 */
	if (config_conn != NULL) {
		bt_conn_unref(config_conn);
		config_conn = NULL;
	}

	/* 断开后重新启动可连接广播，方便手机下一次再连接。 */
	k_work_submit(&config_adv_restart_work);
}

/* 注册 BLE 连接回调。Zephyr 会在连接/断开时自动调用对应函数。 */
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/*
 * GATT 读回调：读取当前 Beacon 广播间隔。
 *
 * 手机读 BEACON_CFG_INTERVAL_UUID_BYTES 特征值时会进入这里。
 * 返回值格式：uint16_t，小端，单位 ms。
 */
static ssize_t read_interval_ms(struct bt_conn *conn,
				const struct bt_gatt_attr *attr,
				void *buf, uint16_t len, uint16_t offset)
{
	uint16_t value = sys_cpu_to_le16(beacon_cfg.interval_ms);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));
}

/*
 * GATT 写回调：修改 Beacon 广播间隔。
 *
 * 手机写 BEACON_CFG_INTERVAL_UUID_BYTES 特征值时会进入这里。
 * 要求：
 * - offset 必须为 0，不支持分片/偏移写。
 * - len 必须等于 sizeof(uint16_t)，也就是 2 字节。
 * - 数据格式为小端 uint16_t，单位 ms。
 */
static ssize_t write_interval_ms(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 const void *buf, uint16_t len,
				 uint16_t offset, uint8_t flags)
{
	struct beacon_runtime_cfg new_cfg;
	uint16_t new_interval_ms;
	uint16_t old_interval_ms = beacon_cfg.interval_ms;
	int err;

	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	/* 不支持带 offset 的写入。 */
	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	/* interval_ms 是 uint16_t，所以必须正好写入 2 字节。 */
	if (len != sizeof(uint16_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	/* 从手机写入的 buffer 中按小端解析出新的广播间隔。 */
	new_interval_ms = sys_get_le16(buf);
	if (!beacon_interval_is_valid(new_interval_ms)) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	df_print("Beacon interval write request: old=%u ms, new=%u ms\n",
	 old_interval_ms, new_interval_ms);

	/* 基于当前配置复制一份，只改 interval_ms。 */
	new_cfg = beacon_cfg;
	new_cfg.interval_ms = new_interval_ms;

	/* 重新应用 Beacon 广播配置。 */
	err = beacon_adv_apply(&new_cfg);
	if (err) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return len;
}

/*
 * GATT 读回调：读取当前 Beacon 发射功率。
 *
 * 手机读 BEACON_CFG_TX_POWER_UUID_BYTES 特征值时会进入这里。
 * 返回值格式：int8_t，单位 dBm。
 */
static ssize_t read_tx_power_dbm(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &beacon_cfg.tx_power_dbm,
				 sizeof(beacon_cfg.tx_power_dbm));
}

/*
 * GATT 写回调：修改 Beacon 发射功率。
 *
 * 手机写 BEACON_CFG_TX_POWER_UUID_BYTES 特征值时会进入这里。
 * 要求：
 * - offset 必须为 0。
 * - len 必须等于 sizeof(int8_t)，也就是 1 字节。
 * - 数据格式为 int8_t，单位 dBm。
 */
static ssize_t write_tx_power_dbm(struct bt_conn *conn,
				  const struct bt_gatt_attr *attr,
				  const void *buf, uint16_t len,
				  uint16_t offset, uint8_t flags)
{
	struct beacon_runtime_cfg new_cfg;
	int err;

	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	/* tx_power_dbm 是 int8_t，所以必须正好写入 1 字节。 */
	if (len != sizeof(int8_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	/* 基于当前配置复制一份，只改 tx_power_dbm。 */
	new_cfg = beacon_cfg;
	new_cfg.tx_power_dbm = *((const int8_t *)buf);

	/* 重新应用 Beacon 广播配置。
	 * 如果控制器不支持请求的功率，beacon_adv_apply() 会记录实际选择的功率。
	 */
	err = beacon_adv_apply(&new_cfg);
	if (err) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return len;
}

/*
 * 定义并注册自定义 GATT 服务。
 *
 * 服务结构：
 * - Primary Service：beacon_cfg_service_uuid
 * - Characteristic 1：beacon_cfg_interval_uuid
 *   - 权限：可读、可写
 *   - 读回调：read_interval_ms
 *   - 写回调：write_interval_ms
 * - Characteristic 2：beacon_cfg_tx_power_uuid
 *   - 权限：可读、可写
 *   - 读回调：read_tx_power_dbm
 *   - 写回调：write_tx_power_dbm
 */
BT_GATT_SERVICE_DEFINE(beacon_cfg_svc,
	BT_GATT_PRIMARY_SERVICE(&beacon_cfg_service_uuid.uuid),
	BT_GATT_CHARACTERISTIC(&beacon_cfg_interval_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_interval_ms, write_interval_ms, NULL),
	BT_GATT_CHARACTERISTIC(&beacon_cfg_tx_power_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_tx_power_dbm, write_tx_power_dbm, NULL),
);

/*
 * 创建并启动 Beacon 不可连接广播。
 *
 * 执行流程：
 * 1. 根据默认 beacon_cfg.interval_ms 构造广播参数。
 * 2. 创建扩展广播集 beacon_adv。
 * 3. 刷新 Manufacturer Data。
 * 4. 设置 Beacon 广播数据。
 * 5. 启动 Beacon 广播。
 * 6. 调用 beacon_adv_apply() 再完整应用一次配置，包括发射功率。
 */
static int beacon_adv_create(void)
{
	struct bt_le_adv_param param = beacon_adv_param_build(beacon_cfg.interval_ms);
	int err;

	err = bt_le_ext_adv_create(&param, NULL, &beacon_adv);
	if (err) {
		df_print("Failed to create beacon advertising set (err %d)\n", err);
		return err;
	}

	beacon_payload_refresh();

	err = bt_le_ext_adv_set_data(beacon_adv, beacon_ad, ARRAY_SIZE(beacon_ad), NULL, 0);
	if (err) {
		df_print("Failed to set beacon advertising data (err %d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_start(beacon_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		df_print("Failed to start beacon advertising (err %d)\n", err);
		return err;
	}

	/* 再调用一次 apply，确保默认发射功率也通过 HCI VS 命令真正设置下去。 */
	err = beacon_adv_apply(&beacon_cfg);
	if (err) {
		return err;
	}

	df_print("Beacon advertiser started on handle %u\n", bt_le_ext_adv_get_index(beacon_adv));

	return 0;
}

/*
 * 创建并启动 config 可连接广播。
 *
 * 执行流程：
 * 1. 构造可连接广播参数。
 * 2. 创建扩展广播集 config_adv，并绑定 config_adv_cb。
 * 3. 设置广播数据 config_ad 和扫描响应数据 config_sd。
 * 4. 启动 config_adv，让手机可以扫描连接。
 */
static int config_adv_create(void)
{
	struct bt_le_adv_param param = config_adv_param_build();
	int err;

	err = bt_le_ext_adv_create(&param, &config_adv_cb, &config_adv);
	if (err) {
		df_print("Failed to create config advertising set (err %d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_set_data(config_adv, config_ad, ARRAY_SIZE(config_ad),
				     config_sd, ARRAY_SIZE(config_sd));
	if (err) {
		df_print("Failed to set config advertising data (err %d)\n", err);
		return err;
	}

	err = config_adv_start();
	if (err) {
		return err;
	}

	df_print("Config advertiser started on handle %u\n", bt_le_ext_adv_get_index(config_adv));

	return 0;
}

/*
 * 对外入口函数。
 *
 * 注意：该函数默认前提是蓝牙协议栈已经在其他地方完成 bt_enable() 初始化。
 * 如果还没有启用蓝牙，需要先调用 bt_enable(NULL) 或对应初始化函数。
 *
 * 启动后会同时存在两个广播：
 * - beacon_adv：不可连接，用来发送 Beacon 数据。
 * - config_adv：可连接，用来让手机通过 GATT 修改 Beacon 参数。
 */
int broadcaster_multiple(void)
{
	int err;

	/* 创建并启动不可连接 Beacon 广播。 */
	err = beacon_adv_create();
	if (err) {
		return err;
	}

	/* 创建并启动可连接配置广播。 */
	err = config_adv_create();
	if (err) {
		return err;
	}

	df_print("Beacon set: non-connectable, phone config set: connectable\n");
	df_print("Write interval(ms) or tx_power(dBm) through the custom GATT service\n");

	return 0;
}
