/**
 ************************************************************************
 * @file ahu_ble.c
 * @author Ryan Ward
 * @date 31.03.2022
 * @brief Contains Source code for the AHU ble drivers
 **********************************************************************
 * */

#include "ahu_ble.h"

 /* Get Pushbutton GPIO from devicetree */
/* Register LOG Module */
LOG_MODULE_REGISTER(ble_module, LOG_LEVEL_DBG);

/* Function Signature to start BLE Scan */
static void start_scan(void);

/* Connection Struct */
static struct bt_conn* default_conn;

bool dev_found = false;
bool discovered = false;

static uint16_t chrc_handle;
static uint16_t long_chrc_handle;
uint8_t bpm_spo2[2] = { 0x00, 0x00 };

/* UUID for device */
uint16_t dongle_uuid[] = { 0xd0, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
						  0x26, 0x49, 0x60, 0xeb, 0x12, 0x34, 0x56, 0x78 };

static struct bt_uuid_128 deviceUUID = BT_UUID_INIT_128(
	0xd0, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
	0x26, 0x49, 0x60, 0xeb, 0x12, 0x34, 0x56, 0x78);
/* UUID for device */
static struct bt_uuid_128 bpmSpo2UUID = BT_UUID_INIT_128(
	0xd2, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
	0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);

/* Function to check connecting device UUID */
static bool parse_device(struct bt_data* data, void* user_data)
{
	bt_addr_le_t* addr = user_data;
	int i;
	int matchedCount = 0;

	if (data->type == BT_DATA_UUID128_ALL)
	{
		uint16_t temp = 0;
		printk("Found Device\n");
		for (i = 0; i < data->data_len; i++)
		{
			temp = data->data[i];
			if (temp == dongle_uuid[i])
			{
				matchedCount++;
			}
		}

		if (matchedCount == UUID_BUFFER_SIZE)
		{
			// MOBILE UUID MATCHED
			LOG_INF("Device Found");
			int err = bt_le_scan_stop();
			k_msleep(10);
			if (err)
			{
				printk("Stop LE scan failed (err %d)\n", err);
				return true;
			}
			struct bt_le_conn_param* param = BT_LE_CONN_PARAM(BT_GAP_INIT_CONN_INT_MIN, BT_GAP_INIT_CONN_INT_MAX, 0, 800);
			err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN_AUTO,
				param, &default_conn);
			LOG_INF("Device Connected");
			if (err)
			{
				printk("Create conn failed (err %d)\n", err);
				start_scan();
			}

			return false;
		}
	}
	return true;
}

/* Device found callback */
static void device_found(const bt_addr_le_t* addr, int8_t rssi, uint8_t type,
	struct net_buf_simple* ad)
{

	if (default_conn)
	{
		return;
	}

	/* We're only interested in connectable events */

	bt_data_parse(ad, parse_device, (void*)addr);

}

/* Device disconnected callback */
static void disconnected(struct bt_conn* conn, uint8_t reason) {
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn)
	{
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_WRN("Device disconnected, %d", reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;
	dev_found = false;
	discovered = false;

	start_scan();
}


/* Discover GATT characteristics */
static uint8_t discover_func(struct bt_conn* conn,
	const struct bt_gatt_attr* attr,
	struct bt_gatt_discover_params* params)
{
	int err;

	if (attr == NULL)
	{
		if (chrc_handle == 0 || long_chrc_handle == 0)
		{
			LOG_INF("Found Characteristics");
		}

		(void)memset(params, 0, sizeof(*params));

		return BT_GATT_ITER_STOP;
	}

	if (params->type == BT_GATT_DISCOVER_PRIMARY &&
		bt_uuid_cmp(params->uuid, &deviceUUID.uuid) == 0)
	{
		params->uuid = NULL;
		params->start_handle = attr->handle + 1;
		params->type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, params);
		if (err != 0)
		{
			LOG_WRN("Discover failed");
		}

		return BT_GATT_ITER_STOP;
	}
	else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC)
	{
		struct bt_gatt_chrc* chrc = (struct bt_gatt_chrc*)attr->user_data;

		if (bt_uuid_cmp(chrc->uuid, &bpmSpo2UUID.uuid) == 0)
		{
			LOG_INF("Found characteristics");
			chrc_handle = chrc->value_handle;
		}
		else if (bt_uuid_cmp(chrc->uuid, &bpmSpo2UUID.uuid) == 0)
		{
			long_chrc_handle = chrc->value_handle;
		}
	}

	return BT_GATT_ITER_CONTINUE;
}

/* Discover GATT characteristics */
static void gatt_discover(void)
{
	static struct bt_gatt_discover_params discover_params;
	int err;

	LOG_INF("Discovering services and characteristics");

	discover_params.uuid = &deviceUUID.uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	err = bt_gatt_discover(default_conn, &discover_params);

	LOG_INF("Discover complete");
	discovered = true;
}



/* Device connected callback */
static void connected(struct bt_conn* conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err)
	{
		LOG_INF("Failed to connect");

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn)
	{
		return;
	}
	default_conn = conn;

	gatt_discover();

	dev_found = true;
}

/* BT disconnected and connected callback functions */
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};


/* Scan for devices */
static void start_scan(void)
{
	int err;


	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err)
	{
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}
	LOG_INF("Scanning successfully started\n");
}

uint8_t read_bpm_spo2(struct bt_conn* conn, uint8_t err,
	struct bt_gatt_read_params* params,
	const void* data, uint16_t length)
{
	memcpy(&bpm_spo2, data, sizeof(bpm_spo2));

	//printk("RSSI: N1:%d, N2:%d, N3:%d, N4:%d\n", rx_rssi[0], rx_rssi[1], rx_rssi[2], rx_rssi[3]);
	return 0;
}


/* Thread for searching and discovery of BLE devices */
void thread_ble_entry(void)
{
	int err;
	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
	LOG_INF("Bluetooth initialized");
	start_scan();
}



void thread_base_read(void) {
	struct data_item_base* rx_base;
	uint8_t mode = 0;
	static struct bt_gatt_read_params read_params_bpm_spo = {
		.func = read_bpm_spo2,
		.handle_count = 0,
		.by_uuid.uuid = &bpmSpo2UUID.uuid,
		.by_uuid.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
		.by_uuid.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
	};

	int i = 0;
	int err;
	int64_t baseUptime;
	while (1) {
		//printk("THis device is actually working youre just tripping\n");
		if (dev_found) {
			bt_gatt_read(default_conn, &read_params_bpm_spo);
			printk("BPM: %d \n", bpm_spo2[0]);
			printk("SPO2: %d\n", bpm_spo2[1]);
		}
		k_sleep(K_MSEC(10));
	}

}

