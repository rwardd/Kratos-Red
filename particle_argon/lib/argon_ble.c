/*
 * @file oslib/scu_ble/scu_ble.c
 * @author Thomas Salpietro 45822490
 * @date 31/03/2022
 * @brief ble scu driver which initialises BT drivers, starts
 *          advertising, establishes connections and responds to its 
 *          attributes being written to
 * 
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <stdio.h>
#include <drivers/gpio.h>
#include <stdlib.h>
#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include <bluetooth/services/hrs.h>

#define BLE_DISC_SLEEP_MS 250
#define BLE_CONN_SLEEP_MS 1000
#define SHORT_SLEEP_MS 50

static struct bt_conn *g_conn;
bool bleConnected = false;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0xd0, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
                  0x26, 0x49, 0x60, 0xeb, 0x12, 0x34, 0x56, 0x78),
};





static void connected(struct bt_conn *conn, uint8_t err)
{
    g_conn = bt_conn_ref(conn);
    if (err)
    {
        printk("Connection failed (err 0x%02x)\n", err);
        bleConnected = false;
    }
    else
    {
        printk("BLE Connected to Device\n");
        bleConnected = true;
        struct bt_le_conn_param *param = BT_LE_CONN_PARAM(6, 6, 0, 800);
        gatt_discover();

        if (bt_conn_le_param_update(conn, param) < 0)
        {
            while (1)
            {
                printk("Connection Update Error\n");
                k_msleep(10);
            }
        }
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{

    if (conn != g_conn) {
        return;
    }
    
    
    printk("Disconnected: (reason 0x%02x)\n", reason);
    bt_conn_unref(g_conn);
    g_conn = NULL;
    bleConnected = false;
    
}


static struct bt_conn_cb connCallbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static void bt_ready(void) {
    int err;
    
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");
    
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        //settings_load();
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);

    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    //bt_passkey_set(0xAA289);
    printk("Advertising successfully started\n");
}

void scu_ble_connect_thread_entry(void) {
    
    bt_ready();

    bt_conn_cb_register(&connCallbacks);

    while (1) {
        k_msleep(SHORT_SLEEP_MS);
    }
}