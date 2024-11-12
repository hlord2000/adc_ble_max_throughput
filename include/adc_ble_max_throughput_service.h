#ifndef ADC_BLE_MAX_THROUGHPUT_H__
#define ADC_BLE_MAX_THROUGHPUT_H__

#include <stdint.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#define BT_UUID_ABMT_VAL \
    BT_UUID_128_ENCODE(0x4D7CF781, 0x6054, 0x4C04, 0xB3FD, 0x0DE391356984)
#define BT_UUID_ABMT_DATA_VAL \
    BT_UUID_128_ENCODE(0x4D7CF782, 0x6054, 0x4C04, 0xB3FD, 0x0DE391356984)

#define BT_UUID_ABMT            BT_UUID_DECLARE_128(BT_UUID_ABMT_VAL)
#define BT_UUID_ABMT_DATA      BT_UUID_DECLARE_128(BT_UUID_ABMT_DATA_VAL)

extern atomic_t data_notif_enabled;

struct __attribute__((packed)) abmt_data_t {
	uint8_t data[CONFIG_BT_L2CAP_TX_MTU];
};

#endif
