#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>

static struct bt_le_ext_adv *adv_legacy;      // 经典广播集
static struct bt_le_ext_adv *adv_extended;    // 扩展广播集

/* 广播数据：只有设备名 */
static const struct bt_data ad_legacy[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* 广播数据：厂商数据 + 设备名（更长，用于演示扩展广播） */
static uint8_t mfg_data[] = {0xFF,0xFF, 0x01, 0x02, 0x03, 0x04};  // 示例厂商数据
static const struct bt_data ad_extended[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

int broadcaster_multiple(void)
{
    int err;

    /* ========== 1. 创建经典广播（兼容所有手机） ========== */
    struct bt_le_adv_param legacy_param = {
        .id = BT_ID_DEFAULT,
        .sid = 0,
        .options = BT_LE_ADV_OPT_NONE,           // 经典广播
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    };
    
    err = bt_le_ext_adv_create(&legacy_param, NULL, &adv_legacy);
    if (err) {
        printk("创建经典广播失败 (err %d)\n", err);
        return err;
    }
    
    err = bt_le_ext_adv_set_data(adv_legacy, ad_legacy, ARRAY_SIZE(ad_legacy), NULL, 0);
    if (err) {
        printk("设置经典广播数据失败 (err %d)\n", err);
        return err;
    }
    
    err = bt_le_ext_adv_start(adv_legacy, BT_LE_EXT_ADV_START_DEFAULT);
    if (err) {
        printk("启动经典广播失败 (err %d)\n", err);
        return err;
    }
    printk("✓ 经典广播已启动（所有手机都能扫到）\n");
    
    /* ========== 2. 创建扩展广播（需要BLE 5.0手机才能扫到） ========== */
    struct bt_le_adv_param extended_param = {
        .id = BT_ID_DEFAULT,
        .sid = 1,                                // 不同的SID
        .options = BT_LE_ADV_OPT_EXT_ADV,        // 扩展广播（使用2M PHY）
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    };
    
    err = bt_le_ext_adv_create(&extended_param, NULL, &adv_extended);
    if (err) {
        printk("创建扩展广播失败 (err %d)\n", err);
        return err;
    }
    
    err = bt_le_ext_adv_set_data(adv_extended, ad_extended, ARRAY_SIZE(ad_extended), NULL, 0);
    if (err) {
        printk("设置扩展广播数据失败 (err %d)\n", err);
        return err;
    }
    
    err = bt_le_ext_adv_start(adv_extended, BT_LE_EXT_ADV_START_DEFAULT);
    if (err) {
        printk("启动扩展广播失败 (err %d)\n", err);
        return err;
    }
    printk("✓ 扩展广播已启动（需要BLE 5.0手机+专业App才能看到）\n");
    
    return 0;
}