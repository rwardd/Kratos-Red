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

#include "hci.h"
#include "scu_rgb_led.h"
#include "scu_sensors.h"
#include "scu_pb.h"
#include "scu_buzzer.h"
#include "scu_power_management.h"

#define BLE_DISC_SLEEP_MS 250
#define BLE_CONN_SLEEP_MS 1000
#define SHORT_SLEEP_MS 50

static struct bt_conn *g_conn;
static uint16_t chrc_handle;
static uint16_t long_chrc_handle;
uint8_t chrc_data[13] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t zero_data[13] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

struct sensor_data data;
struct pb_info request;
struct buzzerData buzzerData;
struct dcData dcData;


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

static struct bt_uuid_128 commandUUID = BT_UUID_INIT_128(
    0xd1, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);

static struct bt_uuid_128 rssiUUID = BT_UUID_INIT_128(
    0xd2, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);

static struct bt_uuid_128 ultraSonicUUID = BT_UUID_INIT_128(
    0xd3, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);

static struct bt_uuid_128 imuAccelUUID = BT_UUID_INIT_128(
    0xd4, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);

static struct bt_uuid_128 imuGyroUUID = BT_UUID_INIT_128(
    0xd5, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);

static struct bt_uuid_128 imuMagUUID = BT_UUID_INIT_128(
    0xd6, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);

static struct bt_uuid_128 timeUUID = BT_UUID_INIT_128(
    0xd7, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);


uint8_t nodeData[13] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

int8_t rssiValues[12] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint16_t ultraSonicValues[4] = {0x00, 0x00, 0x00, 0x00};

int16_t imuAccel[3] = {0x00, 0x00, 0x00};
int16_t imuGyro[3] = {0x00, 0x00, 0x00};
int16_t imuMag[3] = {0x00, 0x00, 0x00};
int64_t mobTime[1] = {0x00};

static uint8_t discover_func(struct bt_conn *conn,
		const struct bt_gatt_attr *attr,
		struct bt_gatt_discover_params *params)
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
	} else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
		struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;

		if (bt_uuid_cmp(chrc->uuid, &commandUUID.uuid) == 0) {
			printk("Found chrc\n");
			chrc_handle = chrc->value_handle;
		} else if (bt_uuid_cmp(chrc->uuid, &commandUUID.uuid) == 0) {
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

static ssize_t read_rssi_cb(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr, void *buff,
                            uint16_t len, uint16_t offset) {

    const int16_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buff, len, offset, value, 
                                sizeof(rssiValues));

}

static ssize_t read_ultra_sonic_cb(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr, void *buff,
                            uint16_t len, uint16_t offset) {

    const uint16_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buff, sizeof(ultraSonicValues), offset, value, 
                                sizeof(ultraSonicValues));
                                
}

static ssize_t read_imu_cb(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr, void *buff,
                            uint16_t len, uint16_t offset) {

    const int16_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buff, len, offset, value, 
                                sizeof(imuAccel));
                                
}

static ssize_t read_time_cb(struct bt_conn *conn,
                            const struct bt_gatt_attr *attr, void *buff,
                            uint16_t len, uint16_t offset) {

    const int16_t *value = attr->user_data;
    return bt_gatt_attr_read(conn, attr, buff, len, offset, value, 
                                sizeof(mobTime));
                                
}


static void gatt_write_cb(struct bt_conn *conn, uint8_t err,
			  struct bt_gatt_write_params *params)
{
	if (err != BT_ATT_ERR_SUCCESS) {
		printk("Write failed: 0x%02X\n", err);
	}

	(void)memset(params, 0, sizeof(*params));
    

	
}



static ssize_t command_callback(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len,
			       uint16_t offset, uint8_t flags)
{   
	/*******************************INIT VALUES*********************/
    struct sensor_data *rx_data;
    struct accel_data *rx_accel_data;
    struct pb_info *rx_pb;
   
    char vals[10];
    static struct bt_gatt_write_params write_params;
    /***********check characteric is correct length*****************/
    if (len > sizeof(chrc_data)) {
		printk("Invalid chrc length\n");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	} else if (offset + len > sizeof(chrc_data)) {
		printk("Invalid chrc offset and length\n");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
    /***************Copy data from buffer into node****************/
	(void)memcpy(chrc_data + offset, buf, len);

    /***************************TEMPERATURE************************/
    if (chrc_data[3] == HCI_DEVICE_TEMP) {
        data.type = HCI_DEVICE_TEMP;
        k_fifo_put(&sensorFIFO, &data);
        rx_data = k_fifo_get(&sensorFIFO, K_MSEC(1500));
        if (rx_data == NULL) {
            printk("Device currently off\n");
            create_packet(zero_data, HCI_DEVICE_OFF, 0, 0);
            write_params.data = zero_data;
	        write_params.length = sizeof(zero_data);
            write_params.func = gatt_write_cb;
	        write_params.handle = chrc_handle;
            int err = bt_gatt_write(g_conn, &write_params);
            if (err != 0) {
                printk("Failed\n");
            }
        } else {
            sprintf(vals, "%d.%d", rx_data->data.val1, rx_data->data.val2);
                
            create_packet(nodeData, HCI_DEVICE_TEMP, strlen(vals), vals);
            write_params.data = nodeData;
	        write_params.length = sizeof(nodeData);
            write_params.func = gatt_write_cb;
	        write_params.handle = chrc_handle;
            int err = bt_gatt_write(g_conn, &write_params);
            if (err != 0) {
                printk("Failed\n");
            }   
        }
        
               


    /**************************HUMIDITY**************************/           
    } else if (chrc_data[3] == HCI_DEVICE_HUM) {
        data.type = HCI_DEVICE_HUM;
        k_fifo_put(&sensorFIFO, &data);
        rx_data = k_fifo_get(&sensorFIFO, K_MSEC(1500));
        if (rx_data == NULL) {
            printk("Device currently off\n");
            create_packet(zero_data, HCI_DEVICE_OFF, 0, 0);
            write_params.data = zero_data;
	        write_params.length = sizeof(zero_data);
            write_params.func = gatt_write_cb;
	        write_params.handle = chrc_handle;
            int err = bt_gatt_write(g_conn, &write_params);
            if (err != 0) {
                printk("Failed\n");
            } 
        } else {
            sprintf(vals, "%d", rx_data->data.val1);
        
            create_packet(nodeData, HCI_DEVICE_HUM, strlen(vals), vals);

            write_params.data = nodeData;
	        write_params.length = sizeof(nodeData);
            write_params.func = gatt_write_cb;
	        write_params.handle = chrc_handle;
            int err = bt_gatt_write(g_conn, &write_params);
            if (err != 0) {
                printk("Failed\n");
            } 
        }      

    /**********************AIR PRESSURE*************************/
    } else if (chrc_data[3] == HCI_DEVICE_AIR) {
        data.type = HCI_DEVICE_AIR;
        k_fifo_put(&sensorFIFO, &data);
        rx_data = k_fifo_get(&sensorFIFO, K_MSEC(1500));
        if (rx_data == NULL) {
            printk("Device currently off\n");
            create_packet(zero_data, HCI_DEVICE_OFF, 0, 0);
            write_params.data = zero_data;
	        write_params.length = sizeof(zero_data);
            write_params.func = gatt_write_cb;
	        write_params.handle = chrc_handle;
            int err = bt_gatt_write(g_conn, &write_params);
            if (err != 0) {
                printk("Failed\n");
            } 
        } else {
            sprintf(vals, "%d.%d", rx_data->data.val1, rx_data->data.val2);

            create_packet(nodeData, HCI_DEVICE_AIR, strlen(vals), vals);

            write_params.data = nodeData;
	        write_params.length = sizeof(nodeData);
            write_params.func = gatt_write_cb;
	        write_params.handle = chrc_handle;
            int err = bt_gatt_write(g_conn, &write_params);
            if (err != 0) {
                printk("Failed\n");
            }
        }
    /**********************VOC*********************************/   
    } else if (chrc_data[3] == HCI_DEVICE_VOC) {
        data.type = HCI_DEVICE_VOC;
        k_fifo_put(&sensorFIFO, &data);
        rx_data = k_fifo_get(&sensorFIFO, K_MSEC(1500));
        if (rx_data == NULL) {
            printk("Device currently off\n");
            create_packet(zero_data, HCI_DEVICE_OFF, 0, 0);
            write_params.data = zero_data;
	        write_params.length = sizeof(zero_data);
            write_params.func = gatt_write_cb;
	        write_params.handle = chrc_handle;
            int err = bt_gatt_write(g_conn, &write_params);
            if (err != 0) {
                printk("Failed\n");
            }
        } else {
            sprintf(vals, "%d", rx_data->data.val1);
        

            create_packet(nodeData, HCI_DEVICE_VOC, strlen(vals), vals);

            write_params.data = nodeData;
	        write_params.length = sizeof(nodeData);
            write_params.func = gatt_write_cb;
	        write_params.handle = chrc_handle;
            int err = bt_gatt_write(g_conn, &write_params);
            if (err != 0) {
                printk("Failed\n");
            }
        }
    /**********************LED*********************************/
    } else if (chrc_data[3] == HCI_DEVICE_LED) {
        //read data byte and set event: 4 is red, 5 is green, 6 is blue
        if (chrc_data[4] == 1) {
            //send event bit to led RED
            k_event_set(&ledEvents, 0x1);
        } else if (chrc_data[5] == 1) {
            k_event_set(&ledEvents, 0x2);
        } else if (chrc_data[6] == 1) {
            k_event_set(&ledEvents, 0x4);
        }

    /**********************XYZ*********************************/    
    } else if (chrc_data[3] == HCI_DEVICE_X || 
                chrc_data[3] == HCI_DEVICE_Y ||
                chrc_data[3] == HCI_DEVICE_Z) {
        printk("accel request\n");
        data.type = chrc_data[3];
        k_fifo_put(&sensorFIFO, &data);
        rx_accel_data = k_fifo_get(&sensorFIFO, K_FOREVER);
        if (chrc_data[3] == HCI_DEVICE_X) {
            sprintf(vals, "%d.%d", rx_accel_data->x.val1, abs(rx_accel_data->x.val2));
        } else if (chrc_data[3] == HCI_DEVICE_Y) {
            sprintf(vals, "%d.%d", rx_accel_data->y.val1, abs(rx_accel_data->y.val2));
        } else if (chrc_data[3] == HCI_DEVICE_Z) {
            sprintf(vals, "%d.%d", rx_accel_data->z.val1, abs(rx_accel_data->z.val2));
        }
        create_packet(nodeData, chrc_data[3], strlen(vals), vals);
        write_params.data = nodeData;
	    write_params.length = sizeof(nodeData);
        write_params.func = gatt_write_cb;
	    write_params.handle = chrc_handle;
        int err = bt_gatt_write(g_conn, &write_params);
        if (err != 0) {
            printk("Failed\n");
        }
    /**********************PushButton****************************/
    } else if (chrc_data[3] == HCI_DEVICE_PB) {
        request.request = HCI_DEVICE_PB;
        k_fifo_put(&pbFIFO, &request);
        rx_pb = k_fifo_get(&pbFIFO, K_FOREVER);
        if (rx_pb->state) {
            printk("PB ON\n");
            sprintf(vals, "1");
        } else if (!rx_pb->state) {
            printk("PB OFF\n");
            sprintf(vals, "0");
        }
        create_packet(nodeData, HCI_DEVICE_PB, 1, vals);
        write_params.data = nodeData;
	    write_params.length = sizeof(nodeData);
        write_params.func = gatt_write_cb;
	    write_params.handle = chrc_handle;
        int err = bt_gatt_write(g_conn, &write_params);
        if (err != 0) {
            printk("Failed\n");
        }
    /**********************BUZZER****************************/

    } else if (chrc_data[3] == HCI_DEVICE_BUZZER) {
        buzzerData.request = HCI_DEVICE_BUZZER;
        buzzerData.freq = chrc_data[4];
        k_fifo_put(&buzzerFIFO, &buzzerData);
        
    /**********************DC****************************/
    } else if (chrc_data[3] == HCI_DEVICE_DC) {
        dcData.request = HCI_DEVICE_DC;
        dcData.dc = chrc_data[4];
        k_fifo_put(&dcFIFO, &dcData);
    }
    



    
	return len;
}


BT_GATT_SERVICE_DEFINE(mobile_svc,
                       BT_GATT_PRIMARY_SERVICE(&scuUUID.uuid),

                       BT_GATT_CHARACTERISTIC(&commandUUID.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE | BT_GATT_PERM_READ | BT_GATT_PERM_PREPARE_WRITE,
                                              NULL, command_callback, &nodeData),

                        BT_GATT_CHARACTERISTIC(&rssiUUID.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE | BT_GATT_PERM_READ | BT_GATT_PERM_PREPARE_WRITE,
                                              read_rssi_cb, NULL, &rssiValues),

                        BT_GATT_CHARACTERISTIC(&ultraSonicUUID.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE | BT_GATT_PERM_READ | BT_GATT_PERM_PREPARE_WRITE,
                                              read_ultra_sonic_cb, NULL, &ultraSonicValues),

                        BT_GATT_CHARACTERISTIC(&imuAccelUUID.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE | BT_GATT_PERM_READ | BT_GATT_PERM_PREPARE_WRITE,
                                              read_imu_cb, NULL, &imuAccel),

                        BT_GATT_CHARACTERISTIC(&imuGyroUUID.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE | BT_GATT_PERM_READ | BT_GATT_PERM_PREPARE_WRITE,
                                              read_imu_cb, NULL, &imuGyro),

                        BT_GATT_CHARACTERISTIC(&imuMagUUID.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE | BT_GATT_PERM_READ | BT_GATT_PERM_PREPARE_WRITE,
                                              read_imu_cb, NULL, &imuMag),

                        BT_GATT_CHARACTERISTIC(&timeUUID.uuid,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                              BT_GATT_PERM_WRITE | BT_GATT_PERM_READ | BT_GATT_PERM_PREPARE_WRITE,
                                              read_time_cb, NULL, &mobTime),

);


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
        mobTime[0] = k_uptime_get();
        
    }

}

