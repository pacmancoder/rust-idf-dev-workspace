#![feature(alloc_error_handler)]

#![cfg_attr(target_arch = "xtensa", no_std)]

#[cfg(target_arch = "xtensa")]
extern crate panic_halt;

extern crate alloc;


#[cfg(target_arch = "xtensa")]
mod custom_allocator {
    use idf_alloc::IdfAllocator;

    use core::alloc::{GlobalAlloc, Layout};

    #[global_allocator]
    static GLOBAL_ALLOCATOR: IdfAllocator = IdfAllocator;

    #[no_mangle]
    pub unsafe fn __rust_alloc(size: usize, align: usize) -> *mut u8 {
        GLOBAL_ALLOCATOR.alloc( Layout::from_size_align_unchecked(size, align))
    }

    #[no_mangle]
    pub unsafe fn __rust_dealloc(ptr: *mut u8, size: usize, align: usize) {
        GLOBAL_ALLOCATOR.dealloc(ptr, Layout::from_size_align_unchecked(size, align))
    }

    #[no_mangle]
    pub unsafe fn __rust_realloc(ptr: *mut u8,
                                 old_size: usize,
                                 align: usize,
                                 new_size: usize
    ) -> *mut u8 {
        GLOBAL_ALLOCATOR.realloc(ptr, Layout::from_size_align_unchecked(old_size, align), new_size)
    }

    #[no_mangle]
    pub unsafe fn __rust_alloc_zeroed(size: usize, align: usize) -> *mut u8 {
        GLOBAL_ALLOCATOR.alloc_zeroed(Layout::from_size_align_unchecked(size, align))
    }


    #[alloc_error_handler]
    fn rust_idf_oom_error_handler(info: Layout) -> ! { panic!("Out of memory error!"); }
}

use idf_hal::{
    peripherals::*,
    wifi::*,
    gpio::*,
    pwm::*,
    freertos::*,
    watchdog::*,
    uart::*,
};

use alloc::string::String;

enum AppError {
    Unknown,
}

impl From<PwmInitializationError> for AppError {
    fn from(err: PwmInitializationError) -> Self {
        AppError::Unknown
    }
}

impl From<PwmConfigurationError> for AppError {
    fn from(err: PwmConfigurationError) -> Self {
        AppError::Unknown
    }
}

impl From<UartConfigError> for AppError {
    fn from(err: UartConfigError) -> Self {
        AppError::Unknown
    }
}


fn init_pwm<C1, C2>(channel1 : C1, channel2: C2) -> Result<Pwm, AppError>
    where C1: GpioPin + PwmPinMarker, C2: GpioPin + PwmPinMarker
{
    let period = 1000 * 500; // 500 ms
    let duty_channel1 = period / 2;
    let duty_channel2 = period / 4;

    let phase_channel1 = 30;
    let phase_channel2 = 60;

    let mut pwm = PwmInitializer::new()
        .set_period(period)?
        .add_channel(channel1, duty_channel1)?
        .add_channel(channel2, duty_channel2)?
        .initialize().map_err(|(err, _)| err)?;

    pwm.configure(|config| {
        config
            .set_phase(0, phase_channel1)?
            .set_phase(1, phase_channel2)?;
        Ok(())
    })?.start();

    Ok(pwm)
}

fn init_uart0(hw: Uart0Hardware, gpio_hw: &mut GpioHardware) -> Result<Uart0, AppError> {
    Ok(UartInitializer::new(hw).initialize(gpio_hw)?)
}

#[no_mangle]
extern "C" fn app_main() {
    let peripherals = Peripherals::take().unwrap();

    // TODO: Fix WiFi after SDK v3.3 migration
    /*
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
    */
    let mut gpio = GpioHardware::new(peripherals.gpio);

    let mut red_led_gpio = PinInitializer::new(gpio.gpio12.take().unwrap()).configure_as_output().init();

    let mut pwm = init_pwm(gpio.gpio14.take().unwrap(), gpio.gpio13.take().unwrap()).ok().unwrap();

    delay_ms(3000);
    pwm.stop();

    let mut uart = init_uart0(UartHardware::new(peripherals.uart).uart0.unwrap(), &mut gpio)
        .ok().unwrap();

    let test_string = String::from("hello world!\n");

    loop {
        task_yield();
        reset_watchdog();

        uart.write_bytes(test_string.as_bytes());
        delay_ms(1000);
    };

}