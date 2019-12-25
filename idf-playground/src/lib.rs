#![feature(alloc_error_handler)]

#![cfg_attr(target_arch = "xtensa", no_std)]

#[cfg(target_arch = "xtensa")]
extern crate panic_halt;

extern crate alloc;

use idf_hal::{
    peripherals::*,
    wifi::*,
    gpio::*,
    pwm::*,
    watchdog::*,
    uart::*,
    nvs::*,
    system_event::*,
};


use freertos_rs::{Task, CurrentTask, Duration};




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
    fn from(_err: PwmInitializationError) -> Self {
        AppError::Unknown
    }
}

impl From<PwmConfigurationError> for AppError {
    fn from(_err: PwmConfigurationError) -> Self {
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

static mut G_WIFI: Option<WiFi> = None;
static mut G_UART: Option<Uart0> = None;

fn init_event_loop() {
    set_event_loop(move |event| {
        let wifi = unsafe { G_WIFI.as_mut().unwrap() };
        let uart = unsafe { G_UART.as_mut().unwrap() };

        match event {
            SystemEvent::StaStarted => {
                wifi.connect().ok().unwrap();
                uart.write_bytes("STA STARTED!!!!\n".as_bytes());
            },
            SystemEvent::StaConnected(_) => {
                uart.write_bytes("GOT IP!!!!\n".as_bytes());
            },
            SystemEvent::StaDisconnected(event_info) => {
                if event_info.reason == StaDisconnectReason::BasicRateIsNotSupported {
                    wifi.switch_sta_to_bgn_mode().ok().unwrap();
                }
            },
            SystemEvent::Unknown => {},
        };
    })
}

#[no_mangle]
extern "C" fn app_main() {
    init_event_loop();

    let peripherals = Peripherals::take().unwrap();

    Nvs::init(peripherals.nvs).init_partition(PartitionId::default()).ok().unwrap();

    let mut gpio = GpioHardware::new(peripherals.gpio);

    let uart = init_uart0(UartHardware::new(peripherals.uart).uart0.unwrap(), &mut gpio)
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
        G_WIFI = Some(wifi);
        G_UART = Some(uart);
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