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


#include "argon_ble.h"

#define BLE_DISC_SLEEP_MS 250
#define BLE_CONN_SLEEP_MS 1000
#define SHORT_SLEEP_MS 50

static struct bt_conn* g_conn;
static uint16_t chrc_handle;
static uint16_t long_chrc_handle;


static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0xd0, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
                  0x26, 0x49, 0x60, 0xeb, 0x12, 0x34, 0x56, 0x78),
};

bool bleConnected = false;

static struct bt_uuid_128 scuUUID = BT_UUID_INIT_128(
    0xd0, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x12, 0x34, 0x56, 0x78);

static struct bt_uuid_128 bpmSpoUUID = BT_UUID_INIT_128(
    0xd2, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);


int8_t bpmSpoValues[2] = { 0x01, 0x02 };


static uint8_t discover_func(struct bt_conn* conn,
    const struct bt_gatt_attr* attr,
    struct bt_gatt_discover_params* params)
{
    int err;

    if (attr == NULL) {
        if (chrc_handle == 0 || long_chrc_handle == 0) {
            //FAIL("Did not discover chrc (%x) or long_chrc (%x)",
              //   chrc_handle, long_chrc_handle);
        }

        (void)memset(params, 0, sizeof(*params));

        //SET_FLAG(flag_discover_complete);

        return BT_GATT_ITER_STOP;
    }

    printk("[ATTRIBUTE] handle %u\n", attr->handle);

    if (params->type == BT_GATT_DISCOVER_PRIMARY &&
        bt_uuid_cmp(params->uuid, &scuUUID.uuid) == 0) {
        printk("Found test service\n");
        params->uuid = NULL;
        params->start_handle = attr->handle + 1;
        params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

        err = bt_gatt_discover(conn, params);
        if (err != 0) {
            //FAIL("Discover failed (err %d)\n", err);
        }

        return BT_GATT_ITER_STOP;
    }
    else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        struct bt_gatt_chrc* chrc = (struct bt_gatt_chrc*)attr->user_data;

        if (bt_uuid_cmp(chrc->uuid, &bpmSpoUUID.uuid) == 0) {
            printk("Found chrc\n");
            chrc_handle = chrc->value_handle;
        }
        else if (bt_uuid_cmp(chrc->uuid, &bpmSpoUUID.uuid) == 0) {
            printk("Found long_chrc\n");
            long_chrc_handle = chrc->value_handle;
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

static void gatt_discover(void)
{
    static struct bt_gatt_discover_params discover_params;
    int err;

    printk("Discovering services and characteristics\n");

    discover_params.uuid = &scuUUID.uuid;
    discover_params.func = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    err = bt_gatt_discover(g_conn, &discover_params);
    if (err != 0) {
        //FAIL("Discover failed(err %d)\n", err);
    }


    printk("Discover complete\n");
}

static ssize_t read_bpm_spo2_cb(struct bt_conn* conn,
    const struct bt_gatt_attr* attr, void* buff,
    uint16_t len, uint16_t offset) {

    const int16_t* value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buff, len, offset, value,
        sizeof(bpmSpoValues));

}



BT_GATT_SERVICE_DEFINE(mobile_svc,
    BT_GATT_PRIMARY_SERVICE(&scuUUID.uuid),

    BT_GATT_CHARACTERISTIC(&bpmSpoUUID.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE | BT_GATT_PERM_READ | BT_GATT_PERM_PREPARE_WRITE,
        read_bpm_spo2_cb, NULL, &bpmSpoValues),

    );


static void connected(struct bt_conn* conn, uint8_t err)
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
        struct bt_le_conn_param* param = BT_LE_CONN_PARAM(6, 6, 0, 800);
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

static void disconnected(struct bt_conn* conn, uint8_t reason)
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


static void bt_ready(void)
{
    int err;

    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        //settings_load();
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);

    if (err)
    {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    //bt_passkey_set(0xAA289);
    printk("Advertising successfully started\n");
}

void scu_ble_connect_thread_entry(void)
{
    bt_ready();

    bt_conn_cb_register(&connCallbacks);

    while (1)
    {
        k_msleep(SHORT_SLEEP_MS);

    }

}

