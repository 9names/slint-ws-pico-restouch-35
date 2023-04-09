#![no_std]
#![no_main]
extern crate alloc;
mod xpt2046;

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

// SPI display interface
use display_interface_spi::SPIInterface;

// Graphics drawing utilities
use embedded_graphics::{
    pixelcolor::raw::RawU16, pixelcolor::Rgb565, prelude::*, primitives::Rectangle,
};
use mipidsi::{self, Builder, HorizontalRefreshOrder, RefreshOrder, VerticalRefreshOrder};

use fugit::RateExtU32;

use alloc::{boxed::Box, rc::Rc};
use embedded_alloc::Heap;

use slint::platform::{
    software_renderer::{MinimalSoftwareWindow, RepaintBufferType},
    Platform, WindowEvent,
};

slint::include_modules!();

struct MyPlatform {
    window: Rc<MinimalSoftwareWindow>,
    // optional: some timer device from your device's HAL crate
    timer: hal::Timer,
    // ... maybe more devices
}

impl Platform for MyPlatform {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_micros(self.timer.get_counter().ticks())
    }
}

struct DisplayWrapper<'a, T> {
    display: &'a mut T,
    line_buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}
impl<T: DrawTarget<Color = embedded_graphics::pixelcolor::Rgb565>>
    slint::platform::software_renderer::LineBufferProvider for DisplayWrapper<'_, T>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;
    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        // Render into the line
        render_fn(&mut self.line_buffer[range.clone()]);

        // Send the line to the screen using DrawTarget::fill_contiguous
        self.display
            .fill_contiguous(
                &Rectangle::new(
                    Point::new(range.start as _, line as _),
                    Size::new(range.len() as _, 1),
                ),
                self.line_buffer[range]
                    .iter()
                    .map(|p| RawU16::new(p.0).into()),
            )
            .map_err(drop)
            .unwrap();
    }
}

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[entry]
fn main() -> ! {
    // initialise heap for use with allocator
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 200 * 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }
    info!("slint demo start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut display_dc = pins.gpio8.into_push_pull_output();
    display_dc.set_high().unwrap();
    let mut display_cs = pins.gpio9.into_push_pull_output();
    display_cs.set_high().unwrap();

    let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio12.into_mode::<hal::gpio::FunctionSpi>();
    let mut display_bl = pins.gpio13.into_push_pull_output();
    let mut rst_pin = pins.gpio15.into_push_pull_output();
    rst_pin.set_high().unwrap();
    delay.delay_ms(50);
    let spi = hal::Spi::<_, _, 8>::new(pac.SPI1);

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        60_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );
    let spi = shared_bus::BusManagerSimple::new(spi);
    display_bl.set_high().unwrap();

    let di = SPIInterface::new(spi.acquire_spi(), display_dc, display_cs);

    let mut display = Builder::ili9486_rgb565(di)
        .with_color_order(mipidsi::ColorOrder::Bgr)
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .with_framebuffer_size(320, 480)
        .with_display_size(320, 480)
        .with_refresh_order(RefreshOrder::new(
            VerticalRefreshOrder::BottomToTop,
            HorizontalRefreshOrder::LeftToRight,
        ))
        .with_orientation(mipidsi::Orientation::LandscapeInverted(false))
        .init(&mut delay, Some(rst_pin))
        .unwrap();

    let touch_irq = pins.gpio17.into_pull_up_input();
    let touch_cs = pins.gpio16.into_push_pull_output();
    let mut touch = xpt2046::XPT2046::new(touch_irq, touch_cs, spi.acquire_spi()).unwrap();

    display.clear(Rgb565::BLACK).unwrap();

    let window = MinimalSoftwareWindow::new(RepaintBufferType::NewBuffer);
    slint::platform::set_platform(Box::new(MyPlatform {
        window: window.clone(),
        timer,
    }))
    .unwrap();

    const DISPLAY_WIDTH: usize = 480;
    let mut line_buffer = [slint::platform::software_renderer::Rgb565Pixel(0); DISPLAY_WIDTH];

    let _ui = MainUI::new();
    let mut last_touch = None;
    loop {
        // Let Slint run the timer hooks and update animations.
        slint::platform::update_timers_and_animations();

        // Check the touch screen or input device using your driver.
        let button = slint::platform::PointerEventButton::Left;
        if let Some(event) = touch
            .read()
            .map_err(|_| ())
            .unwrap()
            .map(|point| {
                let position =
                    slint::PhysicalPosition::new((point.0 * 320.) as _, (point.1 * 240.) as _)
                        .to_logical(window.scale_factor());
                match last_touch.replace(position) {
                    Some(_) => WindowEvent::PointerMoved { position },
                    None => WindowEvent::PointerPressed { position, button },
                }
            })
            .or_else(|| {
                last_touch
                    .take()
                    .map(|position| WindowEvent::PointerReleased { position, button })
            })
        {
            window.dispatch_event(event);
            // Print touch position to debug console if it has changed
            if let Some(last_touch) = last_touch {
                let x = last_touch.x;
                let y = last_touch.y;
                debug!("touch position: {:?},{:?}", x, y);
            }
        }

        window.draw_if_needed(|renderer| {
            renderer.render_by_line(DisplayWrapper {
                display: &mut display,
                line_buffer: &mut line_buffer,
            });
        });

        // // Try to put the MCU to sleep
        // if !window.has_active_animations() {
        //     if let Some(duration) = slint::platform::duration_until_next_timer_update() {
        //         // ... schedule a timer interrupt in `duration` ...
        //     }
        //     cortex_m::asm::wfi(); // Wait for interrupt
        // }
    }
}
