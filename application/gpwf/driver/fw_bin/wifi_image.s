//;======================================================================================================
//        AREA    Resource, CODE, READONLY
//;======================================================================================================
//            EXPORT FW_LOADER_START
//            EXPORT FW_LOADER_END
//            EXPORT WIFI_APP_START
//            EXPORT WIFI_APP_END
//            EXPORT WIFI_APP_WS_START
//            EXPORT WIFI_APP_WS_END
.section .rodata
            .global FW_LOADER_START
            .global FW_LOADER_END
FW_LOADER_START:
        .incbin  "../application/gpwf/driver/fw_bin/fw_loader.bin"
FW_LOADER_END:

.section .rodata
            .global WIFI_APP_START
            .global WIFI_APP_END

WIFI_APP_START:
        .incbin  "../application/gpwf/driver/fw_bin/wifi_app.bin"
WIFI_APP_END:

.section .rodata
            .global WIFI_APP_WS_START
            .global WIFI_APP_WS_END

WIFI_APP_WS_START:
        .incbin  "../application/gpwf/driver/fw_bin/wifi_app_ws.bin"
WIFI_APP_WS_END:
.section .rodata
            .global WIFI_APP_SSL_START
            .global WIFI_APP_SSL_END

WIFI_APP_SSL_START:
        .incbin  "../application/gpwf/driver/fw_bin/wifi_app_ssl.bin"
WIFI_APP_SSL_END:
.section .rodata
            .global WIFI_APP_TUTK_START
            .global WIFI_APP_TUTK_END

WIFI_APP_TUTK_START:
        .incbin  "../application/gpwf/driver/fw_bin/wifi_app_tutk.bin"
WIFI_APP_TUTK_END:
                .end
