#include <stdio.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <hal/nrf_gpio.h>

#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include "Lib/contrain.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

#define LDO_NODE 		    		DT_ALIAS(pinled)
#define SWRING_NODE		    		DT_ALIAS(swring)
#define BT_UUID_SERVICE_REMOTE_BIKE      			BT_UUID_128_ENCODE(0x1025101d, 0x243f, 0x4de0, 0x8df5, 0x870c2b7d500e)
#define BT_UUID_CHARACTERISTIC_BUTTON_REMOTE		BT_UUID_128_ENCODE(0xe87d4210, 0xd28d, 0x42b1, 0x947b, 0x4a2fff0b0409)
#define BATTERY_MIN					3490		
#define BATTERY_MAX					3690
#define TIME_READ_LOW_BATTERY		500			// unit ms
#define TIME_READ_BATTERY			500			// unit seconds

static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(BT_UUID_SERVICE_REMOTE_BIKE);
static struct bt_uuid_128 charc_button_uuid = BT_UUID_INIT_128(BT_UUID_CHARACTERISTIC_BUTTON_REMOTE);

const struct device *uart0_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LDO_NODE, gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SWRING_NODE, gpios);

static struct gpio_callback button_cb_data;
static uint8_t notifyMain;
struct k_timer bat_timer;
struct k_timer timer_1000;

int16_t buf;
struct adc_sequence sequence = {
	.buffer = &buf,
	/* buffer size in bytes, not number of samples */
	.buffer_size = sizeof(buf),
};

uint32_t timerDelay = 0;
uint32_t lastTimeButton = 0;
uint32_t lastTimeLed = 0;
bool stateSleep = true;
bool connect = false;
uint8_t mode = 0;
static float Vin = 0;

void wakeup();
void deepSleep(bool stateSleep);
void handleControl();
void setLed(uint8_t);
void setLedBattery(uint16_t _delayMS);
void beginAD(void);
uint16_t analogRead(uint8_t _pinAdc);


extern void battery_timerFunc(struct k_timer *timer_id) {
	int16_t batteryPercent = 0;
	for (uint8_t i = 0 ; i < 100; i++) {
		Vin = analogRead(0);
		UTILS_LOW_PASS_FILTER(Vin, analogRead(0), 0.05);
		batteryPercent = (Vin - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN) * 100.0;
    	batteryPercent = constrain(batteryPercent, 0, 100);
	}
	LOG_INF("Battery: %d", batteryPercent);
	if (batteryPercent < 15) {
		k_timer_start(&bat_timer, K_MSEC(TIME_READ_LOW_BATTERY), K_NO_WAIT);
		setLedBattery(80);
	}else {
		k_timer_start(&bat_timer, K_SECONDS(TIME_READ_BATTERY), K_NO_WAIT);
	}
}

extern void timer_1000_Func(struct k_timer *timer_id) {
	gpio_pin_set_raw(led.port, led.pin, 0);
}

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_SERVICE_REMOTE_BIKE),
};

static void RemoteBike_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
	notifyMain = value == BT_GATT_CCC_NOTIFY; 
}

BT_GATT_SERVICE_DEFINE(RemoteBike,
    BT_GATT_PRIMARY_SERVICE(&service_uuid),
    BT_GATT_CHARACTERISTIC(&charc_button_uuid.uuid,
                            BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_WRITE | BT_GATT_PERM_READ,
                            NULL, NULL, NULL),
    BT_GATT_CCC(RemoteBike_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx) {
	LOG_INF("Updated MTU: TX: %d RX: %d bytes", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated
};

static void connected(struct bt_conn *conn, uint8_t err) {
	if (err) {
		LOG_WRN("Connection failed (err 0x%02x)", err);
		connect = false;
	} else {
		LOG_INF("Connected");
        connect = true;
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
	LOG_WRN("Disconnected (reason 0x%02x)", reason);
    connect = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};

static void bt_ready(void)
{
	int err;
	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

    static struct bt_le_adv_param param; 
    param.id = BT_ID_DEFAULT;
	param.interval_max = 0x0060;
	param.interval_min = 0x0030;
	param.options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME;
	param.peer = NULL;
	param.secondary_max_skip = 0;
	param.sid = 0;

    err = bt_le_adv_start(&param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_WRN("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
}

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {	
	handleControl();
}

int main(void) {
    LOG_INF("This is a slaver remote bike addr: E0:62:BD:03:22:07 v0.0");
    pm_device_action_run(uart0_dev, PM_DEVICE_ACTION_SUSPEND);
	gpio_pin_configure_dt(&led, GPIO_OUTPUT_LOW);
	gpio_pin_configure(button.port, (gpio_port_pins_t)button.pin, GPIO_INPUT);
	beginAD();

	wakeup();
	
	gpio_pin_set_raw(led.port, led.pin, 0);

	int ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		LOG_INF("Error %d: failed to configure interrupt on %s pin %d",
			ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	int err;
    err = bt_enable(NULL);
    if (err) {
        LOG_WRN("Bluetooth init failed (err %d)", err);
        return 0;
    }

    bt_ready();
    bt_gatt_cb_register(&gatt_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);
	
	k_timer_init(&bat_timer, battery_timerFunc, NULL);
	k_timer_start(&bat_timer, K_SECONDS(5), K_NO_WAIT);
	k_timer_init(&timer_1000, timer_1000_Func, NULL);

    return 0;
}

void handleControl() {
	lastTimeButton = k_uptime_get_32();
	while (gpio_pin_get_dt(&button)) {
		if (k_uptime_get_32() - lastTimeButton > 50) mode = 1;
		if (k_uptime_get_32() - lastTimeButton > 500) mode = 2;
		if (k_uptime_get_32() - lastTimeButton > 1000) mode = 3;
		if (k_uptime_get_32() - lastTimeButton > 1500) {
			mode = 0;
			gpio_pin_set_raw(led.port, led.pin, 1);
		}
	}
	
	timerDelay = k_uptime_get_32();
	while(k_uptime_get_32() - timerDelay < 150);

	switch (mode) {
		case 1:
			LOG_INF("Mode 1");
			bt_gatt_notify_uuid(NULL, &charc_button_uuid.uuid, &RemoteBike.attrs[2], &mode, sizeof(mode));
			setLed(1);
			break;
		case 2:
			LOG_INF("Mode 2");
			bt_gatt_notify_uuid(NULL, &charc_button_uuid.uuid, &RemoteBike.attrs[2], &mode, sizeof(mode));
			setLed(2);
			break;
		case 3:
			LOG_INF("Mode 3");
			bt_gatt_notify_uuid(NULL, &charc_button_uuid.uuid, &RemoteBike.attrs[2], &mode, sizeof(mode));
			setLed(3);
			break;
		default:
			deepSleep(true);
			break;
	} 
}

void setLed(uint8_t count) {
	for (uint8_t i = 0 ; i < count ; i++) {
		lastTimeLed = k_uptime_get_32();
		gpio_pin_set_raw(led.port, led.pin, 1);
		while(k_uptime_get_32() - lastTimeLed < 200);
		lastTimeLed = k_uptime_get_32();
		gpio_pin_set_raw(led.port, led.pin, 0);
		while(k_uptime_get_32() - lastTimeLed < 150);
	}
}

/// @brief Set up and check pin Adc
void beginAD(void) {
    /* Configure channels individually prior to sampling. */
	LOG_INF("Set up ADC");
	int8_t error;
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!device_is_ready(adc_channels[i].dev)) {
			LOG_WRN("ADC controller device %s not ready\n", adc_channels[i].dev->name);
		}

		error = adc_channel_setup_dt(&adc_channels[i]);
		if (error < 0) {
			LOG_ERR("Could not setup channel #%d (%d)\n", i, error);
		}
	}
}

/// @brief Analog reading
/// @param _pinAdc analog read pin (takes the order set in the .overlay file, reads from 0-n)
/// @return adc value is read from that pin
uint16_t analogRead(uint8_t _pinAdc) {
    (void)adc_sequence_init_dt(&adc_channels[_pinAdc], &sequence);
    adc_read(adc_channels[_pinAdc].dev, &sequence);
    return (buf < 0) ? 0 : buf;
}

void wakeup() {
	if (gpio_pin_get_dt(&button)) { 
		lastTimeButton = k_uptime_get_32();
		LOG_INF("State Sleep : %d", stateSleep);
		k_msleep(100);
	}
	while (gpio_pin_get_dt(&button)) {
		if (k_uptime_get_32() - lastTimeButton > 3000) {
			stateSleep = false;
			LOG_INF("State Sleep : %d", stateSleep);
			gpio_pin_set_raw(led.port, led.pin, 1);
			while (gpio_pin_get_dt(&button)) {
				k_msleep(10);
			}
			k_msleep(100);
		}
	}
	
	deepSleep(stateSleep);
}

void deepSleep(bool stateSleep) {
	if (!stateSleep) return;

	LOG_INF("System off remote");
	timerDelay = k_uptime_get_32();
	while(k_uptime_get_32() - timerDelay < 1000);

	nrf_gpio_cfg_input(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(swring), gpios),
			   NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_sense_set(NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(swring), gpios),
			       NRF_GPIO_PIN_SENSE_HIGH);
	
	gpio_pin_set_raw(led.port, led.pin, 0);

    struct pm_state_info info;
    info.state = PM_STATE_SOFT_OFF;
    info.min_residency_us = 0;
    info.exit_latency_us = 0;
    pm_state_force(0u, &info);
}

void setLedBattery(uint16_t _delayMS) {
	gpio_pin_set_raw(led.port, led.pin, 1);
	k_timer_start(&timer_1000, K_MSEC(_delayMS), K_NO_WAIT);	
}