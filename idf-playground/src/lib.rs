#![cfg_attr(target_arch = "xtensa", no_std)]

#[cfg(target_arch = "xtensa")]
extern crate panic_halt;

use idf_sys::{
    wifi::*,
    error::*
};
use idf_sys::network_adapter::esp_interface_t_ESP_IF_WIFI_AP;

#[no_mangle]
extern "C" fn app_main() {
    unsafe {
        const SSID_NAME: &[u8] = b"Cake is a lie!";

        let wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
        if esp_wifi_init(&wifi_init_config) != esp_err_t_ESP_OK {
            panic!("Can't init wifi adapter");
        }

        let mut wifi_config = ::core::mem::zeroed::<wifi_config_t>();
        wifi_config.ap.ssid_len = 5;

        for (d, s) in (wifi_config.ap.ssid.iter_mut()).zip(SSID_NAME.iter().copied()) {
            *d = s;
        }
        wifi_config.ap.ssid_len = SSID_NAME.len() as u8;
        wifi_config.ap.max_connection = 1;
        wifi_config.ap.authmode = wifi_auth_mode_t_WIFI_AUTH_OPEN;

        if esp_wifi_set_mode(wifi_mode_t_WIFI_MODE_AP) != esp_err_t_ESP_OK {
            panic!("Can't set wifi mode to AP");
        }

        if esp_wifi_set_config(esp_interface_t_ESP_IF_WIFI_AP, &mut wifi_config) != esp_err_t_ESP_OK {
            panic!("Can't set wifi mode to AP");
        }

        if esp_wifi_start() != esp_err_t_ESP_OK {
            panic!("Can't start wifi!");
        }
    }
}
