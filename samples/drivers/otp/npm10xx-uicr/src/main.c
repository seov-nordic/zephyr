#include <zephyr/kernel.h>
#include <zephyr/drivers/mfd/npm10xx.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/otp.h>

#define NPM10XX_UICR_LEN 13U
//static const uint8_t uicr_data[NPM10XX_UICR_LEN] = { 0xDE, 0xAD, 0x0B, 0xEE, 0xF0, 0xBA, 0xD0,
//						     0xA0, 0xBA, 0xD0, 0xBA, 0xBE, 0x00, };

static const struct device *npm1012 = DEVICE_DT_GET(DT_NODELABEL(npm1012));
static const struct device *npm1012_charger = DEVICE_DT_GET(DT_NODELABEL(npm1012_charger));
static const struct device *npm1012_uicr = DEVICE_DT_GET(DT_NODELABEL(npm1012_uicr));

int main(void)
{
	int ret;
	static union charger_propval val;

	printf("Should program bits: 6 7");
	//for (int i=0; i<NPM10XX_UICR_LEN; i++) {
	//	uint8_t byte = uicr_data[i];
	//	uint8_t j = 0;
	//	while (byte) {
	//		if (byte & 1U) {
	//			printf("%d ", 8 * i + j);
	//		}
	//		j++; byte >>= 1;
	//	}
	//}

	while (val.online == CHARGER_ONLINE_OFFLINE) {
		ret = charger_get_prop(npm1012_charger, CHARGER_PROP_ONLINE, &val);
		if (ret < 0) {
			printf("Error: failed to get VBUS status (%d)\n", ret);
			return ret;
		}
	}
	printf("VBUS connected, starting UICR programming...\n");

	ret = otp_program(npm1012_uicr, 0U, (uint8_t []){0xC0}, 1U);

	if (ret < 0) {
		printf("Error: UICR programming failed (%d)\n", ret);
	} else {
		printf("UICR programming success. Reseting PMIC...\n");
		k_sleep(K_SECONDS(2));
		(void)mfd_npm10xx_reset(npm1012);
	}

	return ret;
}
