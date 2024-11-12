#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bt_abmt, LOG_LEVEL_DBG);

#include "adc_ble_max_throughput_service.h"

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

const static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

atomic_t data_notif_enabled;
static void abmt_data_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	atomic_set(&data_notif_enabled, value == BT_GATT_CCC_NOTIFY);
    LOG_INF("ABMT data notifications %s", atomic_get(&data_notif_enabled) ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(abmt_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_ABMT),
	BT_GATT_CHARACTERISTIC(BT_UUID_ABMT_DATA,
						   BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
						   BT_GATT_PERM_READ,
						   NULL, NULL, NULL),
	BT_GATT_CCC(abmt_data_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static struct bt_gatt_exchange_params exchange_params;
static void exchange_func(struct bt_conn *conn, uint8_t err,
    struct bt_gatt_exchange_params *params)
{
    if (!err) {
        uint32_t bt_max_send_len = bt_gatt_get_mtu(conn) - 3;
        LOG_INF("max send len is %d", bt_max_send_len);
    }
}

static void connected(struct bt_conn *conn, uint8_t err) {
    char addr[BT_ADDR_LE_STR_LEN];

    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected %s", addr);

    err = bt_le_adv_stop();
    if (err) {
        LOG_ERR("Failed to stop advertising, err: %d", err);
    }

	exchange_params.func = exchange_func;
	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		LOG_ERR("MTU exchange failed (err %d)", err);
	}

	struct bt_conn_le_phy_param preferred_phy;

	preferred_phy.pref_tx_phy = BT_GAP_LE_PHY_2M;
	preferred_phy.pref_rx_phy = BT_GAP_LE_PHY_2M;

    err = bt_conn_le_phy_update(conn, &preferred_phy);
    if (err < 0) {
        LOG_ERR("bt_conn_le_phy_update() returned %d", err);
		return;
    }
}

static void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param) {
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("LE PHY Updated: %s Tx 0x%x, Rx 0x%x", addr, param->tx_phy, param->rx_phy);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
	.le_phy_updated = le_phy_updated,
};

K_MSGQ_DEFINE(abmt_data_queue, CONFIG_ABMT_ADC_BUFFER_SIZE * 2, 512, 1);

static void abmt_notification_thread(void) {
    int err;
    uint16_t block_data[CONFIG_ABMT_ADC_BUFFER_SIZE];
	
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth initialization failed (err %d)", err);
		return;
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Bluetooth advertising failed (err %d)", err);
		return;
	}

	while (true) {
        if (atomic_get(&data_notif_enabled)) {
			err = k_msgq_get(&abmt_data_queue, block_data, K_FOREVER);

			err = bt_gatt_notify(NULL, &abmt_svc.attrs[1],
							   block_data,
							   CONFIG_ABMT_ADC_BUFFER_SIZE * 2);
        } else {
            k_msleep(500);
        }
    }
}

K_THREAD_DEFINE(abmt_notify_thread, 8192, abmt_notification_thread, NULL, NULL, NULL, 1, 0, 0);
