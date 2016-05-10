#include <usb.h>
#include <usb_host_android.h>

#define GOOGLE_USB_VENDER_ID 0x18D1ul
#define AOA_V1_ACCESSORY_PRODUCT_ID 0x2D00ul
#define AOA_V1_ACCESSORY_ADB_PRODUCT_ID 0x2D01ul
#define AOA_V2_AUDIO_PRODUCT_ID 0x2D02ul
#define AOA_V2_AUDIO_ADB_PRODUCT_ID 0x2D03ul
#define AOA_V2_ACCESSORY_AUDIO_PRODUCT_ID 0x2D04ul
#define AOA_V2_ACCESSORY_AUDIO_ADB_PRODUCT_ID 0x2D05ul

USB_TPL usbTPL[NUM_TPL_ENTRIES] = {
    { INIT_VID_PID(GOOGLE_USB_VENDER_ID, AOA_V1_ACCESSORY_PRODUCT_ID), 0, 1, { TPL_EP0_ONLY_CUSTOM_DRIVER } },
    { INIT_VID_PID(GOOGLE_USB_VENDER_ID, AOA_V1_ACCESSORY_ADB_PRODUCT_ID), 0, 1, { TPL_EP0_ONLY_CUSTOM_DRIVER } },
    { INIT_VID_PID(GOOGLE_USB_VENDER_ID, AOA_V2_AUDIO_PRODUCT_ID), 0, 1, { TPL_EP0_ONLY_CUSTOM_DRIVER } },
    { INIT_VID_PID(GOOGLE_USB_VENDER_ID, AOA_V2_AUDIO_ADB_PRODUCT_ID), 0, 1, { TPL_EP0_ONLY_CUSTOM_DRIVER } },
    { INIT_VID_PID(GOOGLE_USB_VENDER_ID, AOA_V2_ACCESSORY_AUDIO_PRODUCT_ID), 0, 1, { TPL_EP0_ONLY_CUSTOM_DRIVER } },
    { INIT_VID_PID(GOOGLE_USB_VENDER_ID, AOA_V2_ACCESSORY_AUDIO_ADB_PRODUCT_ID), 0, 1, { TPL_EP0_ONLY_CUSTOM_DRIVER } },
    { INIT_VID_PID(0xFFFFul, 0xFFFFul), 0, 0, { 0 } }
};

CLIENT_DRIVER_TABLE usbClientDrvTable[NUM_CLIENT_DRIVER_ENTRIES] = {
    { AndroidAppInitialize, AndroidAppEventHandler, AndroidAppDataEventHandler, 0 },
    { AndroidAppInitialize, AndroidAppEventHandler, AndroidAppDataEventHandler, ANDROID_INIT_FLAG_BYPASS_PROTOCOL }
};
