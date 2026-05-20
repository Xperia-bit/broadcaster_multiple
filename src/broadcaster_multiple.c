#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/bluetooth/uuid.h>

BUILD_ASSERT(IS_ENABLED(CONFIG_BT_HAS_HCI_VS),
	     "This application requires Zephyr HCI vendor extensions");

#define BEACON_ADV_INTERVAL_MS_MIN      20U
#define BEACON_ADV_INTERVAL_MS_MAX      10240U
#define BEACON_ADV_INTERVAL_MS_DEFAULT  200U
#define CONFIG_ADV_INTERVAL_MS_MIN      100U
#define CONFIG_ADV_INTERVAL_MS_MAX      150U

#define BEACON_NAME                     "Nordic Beacon"
#define BEACON_MFG_COMPANY_ID           0xFFFF
#define BEACON_MFG_FRAME_TYPE           0x01

#define BEACON_CFG_SERVICE_UUID_BYTES   \
	0xF0, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, \
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
#define BEACON_CFG_INTERVAL_UUID_BYTES  \
	0xF1, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, \
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
#define BEACON_CFG_TX_POWER_UUID_BYTES  \
	0xF2, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, \
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12

struct beacon_runtime_cfg {
	uint16_t interval_ms;
	int8_t tx_power_dbm;
};

static struct bt_le_ext_adv *beacon_adv;
static struct bt_le_ext_adv *config_adv;
static struct bt_conn *config_conn;

static struct beacon_runtime_cfg beacon_cfg = {
	.interval_ms = BEACON_ADV_INTERVAL_MS_DEFAULT,
	.tx_power_dbm = 0,
};

static void config_adv_restart_work_handler(struct k_work *work);

static K_WORK_DEFINE(config_adv_restart_work, config_adv_restart_work_handler);

static struct bt_uuid_128 beacon_cfg_service_uuid =
	BT_UUID_INIT_128(BEACON_CFG_SERVICE_UUID_BYTES);
static struct bt_uuid_128 beacon_cfg_interval_uuid =
	BT_UUID_INIT_128(BEACON_CFG_INTERVAL_UUID_BYTES);
static struct bt_uuid_128 beacon_cfg_tx_power_uuid =
	BT_UUID_INIT_128(BEACON_CFG_TX_POWER_UUID_BYTES);

/*
 * Manufacturer specific payload:
 * byte[0..1]  Company ID
 * byte[2]     Frame type
 * byte[3..4]  Beacon interval in ms
 * byte[5]     Applied TX power in dBm
 */
static uint8_t beacon_mfg_data[] = {
	(BEACON_MFG_COMPANY_ID & 0xFF),
	(BEACON_MFG_COMPANY_ID >> 8),
	BEACON_MFG_FRAME_TYPE,
	(BEACON_ADV_INTERVAL_MS_DEFAULT & 0xFF),
	(BEACON_ADV_INTERVAL_MS_DEFAULT >> 8),
	0,
};

static const struct bt_data beacon_ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, beacon_mfg_data, sizeof(beacon_mfg_data)),
	BT_DATA(BT_DATA_NAME_COMPLETE, BEACON_NAME, sizeof(BEACON_NAME) - 1),
};

static const struct bt_data config_ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BEACON_CFG_SERVICE_UUID_BYTES),
};

static const struct bt_data config_sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static uint16_t adv_interval_ms_to_units(uint16_t interval_ms)
{
	uint32_t units = ((uint32_t)interval_ms * 1000U) / 625U;

	return (uint16_t)units;
}

static bool beacon_interval_is_valid(uint16_t interval_ms)
{
	return (interval_ms >= BEACON_ADV_INTERVAL_MS_MIN) &&
	       (interval_ms <= BEACON_ADV_INTERVAL_MS_MAX);
}

static void beacon_payload_refresh(void)
{
	sys_put_le16(beacon_cfg.interval_ms, &beacon_mfg_data[3]);
	beacon_mfg_data[5] = (uint8_t)beacon_cfg.tx_power_dbm;
}

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

static int adv_tx_power_set(struct bt_le_ext_adv *adv, int8_t *tx_power_dbm)
{
	struct bt_hci_cp_vs_write_tx_power_level *cp;
	struct bt_hci_rp_vs_write_tx_power_level *rp;
	struct net_buf *buf;
	struct net_buf *rsp = NULL;
	int err;

	if ((adv == NULL) || (tx_power_dbm == NULL)) {
		return -EINVAL;
	}

	buf = bt_hci_cmd_alloc(K_FOREVER);
	if (buf == NULL) {
		return -ENOMEM;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle_type = BT_HCI_VS_LL_HANDLE_TYPE_ADV;
	cp->handle = sys_cpu_to_le16(bt_le_ext_adv_get_index(adv));
	cp->tx_power_level = *tx_power_dbm;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, buf, &rsp);
	if (err) {
		return err;
	}

	rp = (struct bt_hci_rp_vs_write_tx_power_level *)rsp->data;
	*tx_power_dbm = rp->selected_tx_power;
	net_buf_unref(rsp);

	return 0;
}

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

	param = beacon_adv_param_build(new_cfg->interval_ms);

	err = bt_le_ext_adv_stop(beacon_adv);
	if (err) {
		printk("Failed to stop beacon advertising (err %d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_update_param(beacon_adv, &param);
	if (err) {
		printk("Failed to update beacon advertising param (err %d)\n", err);
		return err;
	}

	beacon_cfg = *new_cfg;
	beacon_payload_refresh();

	err = bt_le_ext_adv_set_data(beacon_adv, beacon_ad, ARRAY_SIZE(beacon_ad), NULL, 0);
	if (err) {
		printk("Failed to update beacon advertising data (err %d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_start(beacon_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to restart beacon advertising (err %d)\n", err);
		return err;
	}

	applied_tx_power = beacon_cfg.tx_power_dbm;
	err = adv_tx_power_set(beacon_adv, &applied_tx_power);
	if (err) {
		printk("Failed to apply beacon TX power (err %d)\n", err);
		return err;
	}

	beacon_cfg.tx_power_dbm = applied_tx_power;
	beacon_payload_refresh();

	err = bt_le_ext_adv_set_data(beacon_adv, beacon_ad, ARRAY_SIZE(beacon_ad), NULL, 0);
	if (err) {
		printk("Failed to refresh beacon payload (err %d)\n", err);
		return err;
	}

	printk("Beacon updated: interval=%u ms, tx_power=%d dBm\n",
	       beacon_cfg.interval_ms, beacon_cfg.tx_power_dbm);

	return 0;
}

static int config_adv_start(void)
{
	int err;

	if (config_adv == NULL) {
		return -EINVAL;
	}

	err = bt_le_ext_adv_start(config_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if ((err != 0) && (err != -EALREADY)) {
		printk("Failed to start config advertising (err %d)\n", err);
		return err;
	}

	return 0;
}

static void config_adv_restart_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	(void)config_adv_start();
}

static void config_adv_connected_cb(struct bt_le_ext_adv *adv,
				       struct bt_le_ext_adv_connected_info *info)
{
	printk("Config advertiser[%u] connected, conn %p\n",
	       bt_le_ext_adv_get_index(adv), info->conn);
}

static const struct bt_le_ext_adv_cb config_adv_cb = {
	.connected = config_adv_connected_cb,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (config_conn == NULL) {
		config_conn = bt_conn_ref(conn);
	}

	printk("Phone connected: %s\n", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Phone disconnected: %s, reason 0x%02x %s\n",
	       addr, reason, bt_hci_err_to_str(reason));

	if (config_conn != NULL) {
		bt_conn_unref(config_conn);
		config_conn = NULL;
	}

	k_work_submit(&config_adv_restart_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static ssize_t read_interval_ms(struct bt_conn *conn,
				const struct bt_gatt_attr *attr,
				void *buf, uint16_t len, uint16_t offset)
{
	uint16_t value = sys_cpu_to_le16(beacon_cfg.interval_ms);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));
}

static ssize_t write_interval_ms(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 const void *buf, uint16_t len,
				 uint16_t offset, uint8_t flags)
{
	struct beacon_runtime_cfg new_cfg;
	uint16_t new_interval_ms;
	int err;

	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(flags);

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(uint16_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	new_interval_ms = sys_get_le16(buf);
	if (!beacon_interval_is_valid(new_interval_ms)) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	new_cfg = beacon_cfg;
	new_cfg.interval_ms = new_interval_ms;

	err = beacon_adv_apply(&new_cfg);
	if (err) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return len;
}

static ssize_t read_tx_power_dbm(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr,
				 void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &beacon_cfg.tx_power_dbm,
				 sizeof(beacon_cfg.tx_power_dbm));
}

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

	if (len != sizeof(int8_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	new_cfg = beacon_cfg;
	new_cfg.tx_power_dbm = *((const int8_t *)buf);

	err = beacon_adv_apply(&new_cfg);
	if (err) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return len;
}

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

static int beacon_adv_create(void)
{
	struct bt_le_adv_param param = beacon_adv_param_build(beacon_cfg.interval_ms);
	int err;

	err = bt_le_ext_adv_create(&param, NULL, &beacon_adv);
	if (err) {
		printk("Failed to create beacon advertising set (err %d)\n", err);
		return err;
	}

	beacon_payload_refresh();

	err = bt_le_ext_adv_set_data(beacon_adv, beacon_ad, ARRAY_SIZE(beacon_ad), NULL, 0);
	if (err) {
		printk("Failed to set beacon advertising data (err %d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_start(beacon_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start beacon advertising (err %d)\n", err);
		return err;
	}

	err = beacon_adv_apply(&beacon_cfg);
	if (err) {
		return err;
	}

	printk("Beacon advertiser started on handle %u\n", bt_le_ext_adv_get_index(beacon_adv));

	return 0;
}

static int config_adv_create(void)
{
	struct bt_le_adv_param param = config_adv_param_build();
	int err;

	err = bt_le_ext_adv_create(&param, &config_adv_cb, &config_adv);
	if (err) {
		printk("Failed to create config advertising set (err %d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_set_data(config_adv, config_ad, ARRAY_SIZE(config_ad),
				     config_sd, ARRAY_SIZE(config_sd));
	if (err) {
		printk("Failed to set config advertising data (err %d)\n", err);
		return err;
	}

	err = config_adv_start();
	if (err) {
		return err;
	}

	printk("Config advertiser started on handle %u\n", bt_le_ext_adv_get_index(config_adv));

	return 0;
}

int broadcaster_multiple(void)
{
	int err;

	err = beacon_adv_create();
	if (err) {
		return err;
	}

	err = config_adv_create();
	if (err) {
		return err;
	}

	printk("Beacon set: non-connectable, phone config set: connectable\n");
	printk("Write interval(ms) or tx_power(dBm) through the custom GATT service\n");

	return 0;
}
