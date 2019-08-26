#![cfg_attr(target_arch = "xtensa", no_std)]

#[cfg(target_arch = "xtensa")]
extern crate panic_halt;

use idf_hal::{
    peripherals::*,
    wifi::*
};

#[no_mangle]
extern "C" fn app_main() {
    let peripherals = Peripherals::take().unwrap();

    let ap_config = WiFiApConfigurationBuilder::new()
        .ssid("Привет, мир!")
        .auth_mode(WiFiAuthMode::Wpa2Psk)
        .password("mypassword")
        .build().ok().unwrap();

    let wifi_configurator = WiFiHardware::new(peripherals.wifi)
        .initialize()
        .ok().unwrap();

    let _wifi = wifi_configurator
        .set_ap_config(ap_config)
        .start()
        .ok().unwrap();
}
