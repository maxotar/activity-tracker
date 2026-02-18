#include <stdio.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/lbs.h>
#include <bluetooth/services/nus.h>
#include <zephyr/bluetooth/services/dis.h>

/* ----------------- Hardware Setup ----------------- */
#define LED0_NODE DT_ALIAS(led0)
#define IMU_NODE DT_NODELABEL(lsm6ds3tr_c)
#define IMU_ACCEL_FS_MS2_VAL1 156 /* 16 g ~= 156.9064 m/s^2 */
#define IMU_ACCEL_FS_MS2_VAL2 906400
#define IMU_GYRO_FS_RAD_VAL1 34 /* 2000 dps ~= 34.906585 rad/s */
#define IMU_GYRO_FS_RAD_VAL2 906585
#define MAIN_LOOP_SLEEP_MS 10
#define BLE_DIAG_PERIOD_MS 1000
#define BLE_SEND_PERIOD_MS 100 /* 10 Hz: 1000 / 10 = 100 ms */

#if !DT_NODE_HAS_STATUS(IMU_NODE, okay)
#error "Onboard IMU device is not available"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device *const imu = DEVICE_DT_GET(IMU_NODE);
static bool led_state = true;
static struct bt_conn *nus_conn;
static bool nus_notify_enabled;
static bool nus_tx_ready;
static uint32_t nus_tx_ok_count;
static uint32_t nus_tx_enotconn_count;
static uint32_t nus_tx_eagain_count;
static uint32_t nus_tx_enomem_count;
static uint32_t nus_tx_einval_count;
static uint32_t nus_tx_other_err_count;
static int last_nus_err;

static void nus_send_enabled_cb(enum bt_nus_send_status status)
{
	nus_notify_enabled = (status == BT_NUS_SEND_STATUS_ENABLED);
	nus_tx_ready = nus_notify_enabled;
	printf("NUS notify %s\n", nus_notify_enabled ? "enabled" : "disabled");
}

static void nus_sent_cb(struct bt_conn *conn)
{
	ARG_UNUSED(conn);
	nus_tx_ready = true;
}

static struct bt_nus_cb nus_callbacks = {
	.sent = nus_sent_cb,
	.send_enabled = nus_send_enabled_cb,
};

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err)
	{
		printf("BLE connect failed (err 0x%02x)\n", err);
		return;
	}

	if (nus_conn != NULL)
	{
		bt_conn_unref(nus_conn);
	}

	nus_conn = bt_conn_ref(conn);
	nus_notify_enabled = false;
	nus_tx_ready = false;
	printf("BLE connected\n");
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);

	if (nus_conn != NULL)
	{
		bt_conn_unref(nus_conn);
		nus_conn = NULL;
	}

	nus_notify_enabled = false;
	nus_tx_ready = false;

	printf("BLE disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
};

static int imu_configure_for_dog_motion(const struct device *imu_dev)
{
	struct sensor_value accel_fs = {
		.val1 = IMU_ACCEL_FS_MS2_VAL1,
		.val2 = IMU_ACCEL_FS_MS2_VAL2,
	};
	struct sensor_value gyro_fs = {
		.val1 = IMU_GYRO_FS_RAD_VAL1,
		.val2 = IMU_GYRO_FS_RAD_VAL2,
	};
	struct sensor_value odr = {
		.val1 = 104,
		.val2 = 0,
	};
	int ret;

	ret = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
						  SENSOR_ATTR_FULL_SCALE, &accel_fs);
	if (ret < 0)
	{
		return ret;
	}

	ret = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
						  SENSOR_ATTR_FULL_SCALE, &gyro_fs);
	if (ret < 0)
	{
		return ret;
	}

	ret = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
						  SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	if (ret < 0)
	{
		return ret;
	}

	ret = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
						  SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

	return ret;
}

static void sensor_value_to_tenth_str(const struct sensor_value *value, char *out, size_t out_len)
{
	int64_t micro = (int64_t)value->val1 * 1000000LL + value->val2;
	bool negative = micro < 0;
	int64_t tenths;
	int32_t whole;
	int32_t frac;

	if (negative)
	{
		micro = -micro;
	}

	tenths = (micro + 50000LL) / 100000LL;
	whole = (int32_t)(tenths / 10LL);
	frac = (int32_t)(tenths % 10LL);

	snprintk(out, out_len, "%s%d.%d", negative ? "-" : "", whole, frac);
}

/* ----------------- LBS Callbacks ----------------- */

/* Called by LBS when phone writes to the LED characteristic */
static void lbs_led_cb(bool on)
{
	led_state = on;
	gpio_pin_set_dt(&led, on ? 1 : 0);
	printf("BLE: LED set to %s\n", on ? "ON" : "OFF");
}

/* Called by LBS when phone reads the Button characteristic */
static bool lbs_button_cb(void)
{
	return led_state;
}

static struct bt_lbs_cb lbs_callbacks = {
	.led_cb = lbs_led_cb,
	.button_cb = lbs_button_cb,
};

/* ----------------- Advertising Data ----------------- */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

int main(void)
{
	int ret;
	uint16_t send_period_ms = BLE_SEND_PERIOD_MS;
	uint32_t last_send_ms = 0;
	uint32_t last_diag_ms = 0;

	/* Hardware Init */
	if (!gpio_is_ready_dt(&led))
	{
		printf("Error: GPIO not ready\n");
		return 0;
	}

	// Initialize LED to ON (matches led_state_val = 1)
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		return 0;
	}

	/* LED Button Service Init */
	ret = bt_lbs_init(&lbs_callbacks);
	if (ret)
	{
		printf("LBS init failed (err %d)\n", ret);
		return 0;
	}

	/* Bluetooth Init */
	ret = bt_enable(NULL);
	if (ret)
	{
		printf("Bluetooth init failed (err %d)\n", ret);
		return 0;
	}

	printf("Bluetooth initialized.\n");

	ret = bt_nus_init(&nus_callbacks);
	if (ret)
	{
		printf("NUS init failed (err %d)\n", ret);
		return 0;
	}

	/* Advertising Start */
	ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (ret)
	{
		printf("Advertising failed to start (err %d)\n", ret);
		return 0;
	}

	printf("Advertising successfully started.\n");

	if (!device_is_ready(imu))
	{
		printf("Error: IMU device not ready\n");
		return 0;
	}

	ret = imu_configure_for_dog_motion(imu);
	if (ret < 0)
	{
		printf("Error: failed to configure IMU profile (err %d)\n", ret);
		return 0;
	}

	printf("IMU streaming at 104 Hz, accel=16g, gyro=2000dps\n");
	printf("BLE UART output: 3-axis accel at 10 Hz, numeric CSV format\n");
	// printf("ACC [m/s^2]                     | GYR [rad/s]                      | FLAGS\n");
	// printf("X        Y        Z             | X        Y        Z              | Axyz Gxyz\n");

	last_send_ms = k_uptime_get_32();
	last_diag_ms = last_send_ms;

	/* Main loop */
	while (1)
	{
		struct sensor_value ax, ay, az;
		char ax_str[16], ay_str[16], az_str[16];
		char nus_row[120];
		int nus_len;
		uint32_t now_ms = k_uptime_get_32();

		if (now_ms - last_send_ms < send_period_ms)
		{
			if (now_ms - last_diag_ms >= BLE_DIAG_PERIOD_MS)
			{
				last_diag_ms = now_ms;
				printf("BLE diag: conn=%s notify=%s tx_ready=%s tx_ok=%u enotconn=%u eagain=%u enomem=%u einval=%u other=%u last=%d rate=%uHz\n",
					   (nus_conn != NULL) ? "yes" : "no",
					   nus_notify_enabled ? "on" : "off",
					   nus_tx_ready ? "yes" : "no",
					   nus_tx_ok_count,
					   nus_tx_enotconn_count,
					   nus_tx_eagain_count,
					   nus_tx_enomem_count,
					   nus_tx_einval_count,
					   nus_tx_other_err_count,
					   last_nus_err,
					   1000U / send_period_ms);
			}

			k_sleep(K_MSEC(MAIN_LOOP_SLEEP_MS));
			continue;
		}
		last_send_ms = now_ms;

		ret = sensor_sample_fetch(imu);
		if (ret < 0)
		{
			printf("IMU sample fetch failed (err %d)\n", ret);
			k_sleep(K_MSEC(100));
			continue;
		}

		ret = sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &ax);
		ret |= sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &ay);
		ret |= sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &az);
		if (ret < 0)
		{
			printf("IMU channel read failed (err %d)\n", ret);
			k_sleep(K_MSEC(100));
			continue;
		}

		sensor_value_to_tenth_str(&ax, ax_str, sizeof(ax_str));
		sensor_value_to_tenth_str(&ay, ay_str, sizeof(ay_str));
		sensor_value_to_tenth_str(&az, az_str, sizeof(az_str));

		if (nus_conn != NULL && nus_notify_enabled && nus_tx_ready)
		{
			nus_len = snprintk(nus_row, sizeof(nus_row),
							   "%s,%s,%s\n",
							   ax_str, ay_str, az_str);
			if (nus_len > 0)
			{
				nus_tx_ready = false;
				ret = bt_nus_send(nus_conn, (const uint8_t *)nus_row, (uint16_t)nus_len);
				if (ret == 0)
				{
					nus_tx_ok_count++;
				}
				else if (ret == -ENOTCONN)
				{
					nus_tx_enotconn_count++;
					nus_tx_ready = true;
				}
				else if (ret == -EAGAIN)
				{
					nus_tx_eagain_count++;
					nus_tx_ready = true;
				}
				else if (ret == -ENOMEM)
				{
					nus_tx_enomem_count++;
					nus_tx_ready = true;
				}
				else if (ret == -EINVAL)
				{
					nus_tx_einval_count++;
					nus_notify_enabled = false;
					nus_tx_ready = false;
					last_nus_err = ret;
				}
				else
				{
					nus_tx_other_err_count++;
					nus_tx_ready = true;
					last_nus_err = ret;
				}
			}
		}

		if (now_ms - last_diag_ms >= BLE_DIAG_PERIOD_MS)
		{
			last_diag_ms = now_ms;
			printf("BLE diag: conn=%s notify=%s tx_ready=%s tx_ok=%u enotconn=%u eagain=%u enomem=%u einval=%u other=%u last=%d rate=%uHz\n",
				   (nus_conn != NULL) ? "yes" : "no",
				   nus_notify_enabled ? "on" : "off",
				   nus_tx_ready ? "yes" : "no",
				   nus_tx_ok_count,
				   nus_tx_enotconn_count,
				   nus_tx_eagain_count,
				   nus_tx_enomem_count,
				   nus_tx_einval_count,
				   nus_tx_other_err_count,
				   last_nus_err,
				   1000U / send_period_ms);
		}

		k_sleep(K_MSEC(MAIN_LOOP_SLEEP_MS));
	}
	return 0;
}