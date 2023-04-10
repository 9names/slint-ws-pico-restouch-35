#![no_std]
pub mod xpt2046;
extern crate alloc;

use alloc::rc::Rc;
use rp_pico::hal;

use slint::platform::{software_renderer::MinimalSoftwareWindow, Platform};

use embedded_graphics::{pixelcolor::raw::RawU16, prelude::*, primitives::Rectangle};

pub struct MyPlatform {
    pub window: Rc<MinimalSoftwareWindow>,
    // optional: some timer device from your device's HAL crate
    pub timer: hal::Timer,
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

pub struct DisplayWrapper<'a, T> {
    pub display: &'a mut T,
    pub line_buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
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
