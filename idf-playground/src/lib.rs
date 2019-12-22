#![feature(alloc_error_handler)]

#![cfg_attr(target_arch = "xtensa", no_std)]

#[cfg(target_arch = "xtensa")]
extern crate panic_halt;

#[macro_use]
extern crate alloc;

use idf_hal::{
    peripherals::*,
    wifi::*,
    gpio::*,
    pwm::*,
    watchdog::*,
    uart::*,
};

use idf_sys::{
    error::*,
    system_event::*,
    wifi::*,
    network_adapter::*,
};

use alloc::string::String;
use freertos_rs::{TaskBuilder, Task, CurrentTask, Duration};
use idf_sys::ffi::xtensa_void;
use core::ptr::null_mut;


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
    pub unsafe fn __rust_realloc(
        ptr: *mut u8,
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
    fn from(_err: UartConfigError) -> Self {
        AppError::Unknown
    }
}


fn init_pwm<C1, C2>(channel1 : C1, channel2: C2) -> Result<Pwm, AppError>
    where C1: GpioPin + PwmPinMarker, C2: GpioPin + PwmPinMarker
{
    let period = 1000 * 100; // 500 ms
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


const ESP_ERR_NVS_NO_FREE_PAGES: i32 = 0x1100 + 0x0d;

pub type system_event_cb_t = ::core::option::Option<
    unsafe extern "C" fn(ctx: *mut xtensa_void, event: *mut system_event_t) -> esp_err_t,
>;

extern "C" {
    pub fn esp_event_loop_init(
        cb: system_event_cb_t,
        ctx: *mut xtensa_void,
    ) -> esp_err_t;
}


extern "C" {
    fn nvs_flash_init() -> esp_err_t;
    fn nvs_flash_erase() -> esp_err_t;

}


fn init_nvs() {
    unsafe {
        if nvs_flash_init() == ESP_ERR_NVS_NO_FREE_PAGES {
            if nvs_flash_erase() != esp_err_t_ESP_OK {
                panic!("Can't erase NVS")
            }
            if nvs_flash_init() != esp_err_t_ESP_OK {
                panic!("Can't initialize NVS");
            }
        }
    }
}

static mut gWifi: Option<WiFi> = None;
static mut gUart: Option<Uart0> = None;

unsafe extern "C" fn event_loop(ctx: *mut xtensa_void, event: *mut system_event_t) -> esp_err_t {
    match (*event).event_id {
        system_event_id_t_SYSTEM_EVENT_STA_START => {
            gWifi.as_mut().unwrap().connect();
            gUart.as_mut().unwrap().write_bytes("STA STARTED!!!!\n".as_bytes());
        }
        system_event_id_t_SYSTEM_EVENT_STA_GOT_IP => {
            gUart.as_mut().unwrap().write_bytes("GOT IP!!!!\n".as_bytes());
        }
        system_event_id_t_SYSTEM_EVENT_STA_DISCONNECTED => {
            if (*event).event_info.disconnected.reason as u32
                == wifi_err_reason_t_WIFI_REASON_BASIC_RATE_NOT_SUPPORT
            {
                esp_wifi_set_protocol(
                    esp_interface_t_ESP_IF_WIFI_STA,
                    (WIFI_PROTOCAL_11B | WIFI_PROTOCAL_11G | WIFI_PROTOCAL_11N) as u8
                );

            }
            gWifi.as_mut().unwrap().connect();
            gUart.as_mut().unwrap().write_bytes("RECONNECTION BECAUSE OF BGN!!!!\n".as_bytes());
        }
        _ => {}
    };

    esp_err_t_ESP_OK
}

fn init_event_loop() {
    unsafe {
        esp_event_loop_init(Some(event_loop), null_mut());
    }
}

#[no_mangle]
extern "C" fn app_main() {
    init_nvs();
    init_event_loop();

    let peripherals = Peripherals::take().unwrap();

    let mut gpio = GpioHardware::new(peripherals.gpio);

    let mut uart = init_uart0(UartHardware::new(peripherals.uart).uart0.unwrap(), &mut gpio)
        .ok().unwrap();

    let ap_config = WiFiApConfigurationBuilder::new()
        .ssid("Привет, мир!")
        .auth_mode(WiFiAuthMode::Wpa2Psk)
        .password("mypassword")
        .build().ok().unwrap();

    let sta_config = WiFiStaConfigurationBuilder::new()
        .ssid("Mantra_2.4GHz")
        .password("21216vki")
        .build().ok().unwrap();

    let mut wifi = WiFiHardware::new(peripherals.wifi)
        .initialize()
        .ok().unwrap();

    wifi.set_sta_config(sta_config)
        .set_ap_config(ap_config)
        .start().ok().unwrap();

    unsafe {
        gWifi = Some(wifi);
        gUart = Some(uart);
    };


    let mut red_led_gpio = PinInitializer::new(gpio.gpio12.take().unwrap()).configure_as_output().init();


    let mut pwm = init_pwm(gpio.gpio14.take().unwrap(), gpio.gpio13.take().unwrap()).ok().unwrap();
    pwm.start();

    Task::new().name("test_task").start(move || {
        let mut led_state = true;

        loop {
            // yield
            CurrentTask::delay(Duration::zero());
            reset_watchdog();

            red_led_gpio.set_level(led_state);
            led_state = !led_state;

            CurrentTask::delay(Duration::ms(1000));
        }
    }).ok().unwrap();

    loop {
        CurrentTask::delay(Duration::zero());
        reset_watchdog();
        CurrentTask::delay(Duration::ms(500));
    };

}