#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/lbs.h>
#include <zephyr/bluetooth/services/dis.h>

/* ----------------- Hardware Setup ----------------- */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static bool led_state = true;

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

	/* Main loop */
	while (1)
	{
		k_sleep(K_FOREVER);
	}
	return 0;
}