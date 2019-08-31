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

    let mut wifi = WiFiHardware::new(peripherals.wifi)
        .initialize()
        .ok().unwrap();

    wifi.set_ap_config(ap_config).ok().unwrap();
    wifi.set_mode(WiFiMode::Ap).ok().unwrap();
    wifi.start().ok().unwrap();
}
