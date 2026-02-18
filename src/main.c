#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/lbs.h>
#include <zephyr/bluetooth/services/dis.h>

/* ----------------- Hardware Setup ----------------- */
#define LED0_NODE DT_ALIAS(led0)
#define IMU_NODE DT_NODELABEL(lsm6ds3tr_c)
#define IMU_ACCEL_FS_MS2_VAL1 156 /* 16 g ~= 156.9064 m/s^2 */
#define IMU_ACCEL_FS_MS2_VAL2 906400
#define IMU_GYRO_FS_RAD_VAL1 34 /* 2000 dps ~= 34.906585 rad/s */
#define IMU_GYRO_FS_RAD_VAL2 906585
#define IMU_NEAR_LIMIT_NUM 9
#define IMU_NEAR_LIMIT_DEN 10

#if !DT_NODE_HAS_STATUS(IMU_NODE, okay)
#error "Onboard IMU device is not available"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device *const imu = DEVICE_DT_GET(IMU_NODE);
static bool led_state = true;

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

static void sensor_value_to_milli_str(const struct sensor_value *value, char *out, size_t out_len)
{
	int64_t micro = (int64_t)value->val1 * 1000000LL + value->val2;
	bool negative = micro < 0;

	if (negative)
	{
		micro = -micro;
	}

	snprintk(out, out_len, "%s%lld.%03lld", negative ? "-" : "",
			 (long long)(micro / 1000000LL),
			 (long long)((micro % 1000000LL) / 1000LL));
}

static int64_t sensor_value_abs_micro(const struct sensor_value *value)
{
	int64_t micro = (int64_t)value->val1 * 1000000LL + value->val2;

	return micro < 0 ? -micro : micro;
}

static char axis_limit_flag(const struct sensor_value *value, int64_t full_scale_micro)
{
	int64_t abs_micro = sensor_value_abs_micro(value);

	if (abs_micro >= full_scale_micro)
	{
		return 'X';
	}

	if (abs_micro * IMU_NEAR_LIMIT_DEN >= full_scale_micro * IMU_NEAR_LIMIT_NUM)
	{
		return '!';
	}

	return '.';
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
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

int main(void)
{
	int ret;

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

	/* Advertising Start */
	ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
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
	printf("ACC [m/s^2]                     | GYR [rad/s]                      | FLAGS\n");
	printf("X        Y        Z             | X        Y        Z              | Axyz Gxyz\n");

	/* Main loop */
	while (1)
	{
		struct sensor_value ax, ay, az;
		struct sensor_value gx, gy, gz;
		char ax_str[20], ay_str[20], az_str[20];
		char gx_str[20], gy_str[20], gz_str[20];
		char afx, afy, afz;
		char gfx, gfy, gfz;
		int64_t accel_fs_micro;
		int64_t gyro_fs_micro;

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
		ret |= sensor_channel_get(imu, SENSOR_CHAN_GYRO_X, &gx);
		ret |= sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y, &gy);
		ret |= sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z, &gz);
		if (ret < 0)
		{
			printf("IMU channel read failed (err %d)\n", ret);
			k_sleep(K_MSEC(100));
			continue;
		}

		sensor_value_to_milli_str(&ax, ax_str, sizeof(ax_str));
		sensor_value_to_milli_str(&ay, ay_str, sizeof(ay_str));
		sensor_value_to_milli_str(&az, az_str, sizeof(az_str));
		sensor_value_to_milli_str(&gx, gx_str, sizeof(gx_str));
		sensor_value_to_milli_str(&gy, gy_str, sizeof(gy_str));
		sensor_value_to_milli_str(&gz, gz_str, sizeof(gz_str));

		accel_fs_micro = (int64_t)IMU_ACCEL_FS_MS2_VAL1 * 1000000LL + IMU_ACCEL_FS_MS2_VAL2;
		gyro_fs_micro = (int64_t)IMU_GYRO_FS_RAD_VAL1 * 1000000LL + IMU_GYRO_FS_RAD_VAL2;
		afx = axis_limit_flag(&ax, accel_fs_micro);
		afy = axis_limit_flag(&ay, accel_fs_micro);
		afz = axis_limit_flag(&az, accel_fs_micro);
		gfx = axis_limit_flag(&gx, gyro_fs_micro);
		gfy = axis_limit_flag(&gy, gyro_fs_micro);
		gfz = axis_limit_flag(&gz, gyro_fs_micro);

		printf("%8s %8s %8s | %8s %8s %8s |  %c%c%c  %c%c%c\n",
			   ax_str, ay_str, az_str, gx_str, gy_str, gz_str,
			   afx, afy, afz, gfx, gfy, gfz);

		k_sleep(K_USEC(9615));
	}
	return 0;
}