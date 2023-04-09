// This implementation has been copied from slint mcu template: https://github.com/slint-ui/slint-mcu-rust-template/blob/5fefaaad9afb1a17bc8a3e9de649cef1a46b4fe7/src/main.rs#L197
// The only modification was dropping SPI frequency for this chip
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use fugit::RateExtU32;

pub struct XPT2046<IRQ: InputPin + 'static, CS: OutputPin, SPI: Transfer<u8>> {
    irq: IRQ,
    cs: CS,
    spi: SPI,
    pressed: bool,
}

impl<PinE, IRQ: InputPin<Error = PinE>, CS: OutputPin<Error = PinE>, SPI: Transfer<u8>>
    XPT2046<IRQ, CS, SPI>
{
    pub fn new(irq: IRQ, mut cs: CS, spi: SPI) -> Result<Self, PinE> {
        cs.set_high()?;
        Ok(Self {
            irq,
            cs,
            spi,
            pressed: false,
        })
    }

    pub fn read(&mut self) -> Result<Option<(f32, f32)>, Error<PinE, SPI::Error>> {
        const PRESS_THRESHOLD: i32 = -25_000;
        const RELEASE_THRESHOLD: i32 = -30_000;
        let threshold = if self.pressed {
            RELEASE_THRESHOLD
        } else {
            PRESS_THRESHOLD
        };
        self.pressed = false;

        if self.irq.is_low().map_err(|e| Error::Pin(e))? {
            const CMD_X_READ: u8 = 0b10010000;
            const CMD_Y_READ: u8 = 0b11010000;
            const CMD_Z1_READ: u8 = 0b10110000;
            const CMD_Z2_READ: u8 = 0b11000000;

            // These numbers were measured approximately.
            const MIN_X: u32 = 1900;
            const MAX_X: u32 = 30300;
            const MIN_Y: u32 = 2300;
            const MAX_Y: u32 = 30300;

            // FIXME! how else set the frequency to this device
            unsafe { set_spi_freq(3_000_000u32.Hz()) };

            self.cs.set_low().map_err(|e| Error::Pin(e))?;

            macro_rules! xchg {
                ($byte:expr) => {
                    match self
                        .spi
                        .transfer(&mut [$byte, 0, 0])
                        .map_err(|e| Error::Transfer(e))?
                    {
                        [_, h, l] => ((*h as u32) << 8) | (*l as u32),
                        _ => return Err(Error::InternalError),
                    }
                };
            }

            let z1 = xchg!(CMD_Z1_READ);
            let z2 = xchg!(CMD_Z2_READ);
            let z = z1 as i32 - z2 as i32;

            if z < threshold {
                xchg!(0);
                self.cs.set_high().map_err(|e| Error::Pin(e))?;
                unsafe { set_spi_freq(60_000_000u32.Hz()) };
                return Ok(None);
            }

            xchg!(CMD_X_READ | 1); // Dummy read, first read is a outlier

            let mut point = (0u32, 0u32);
            for _ in 0..10 {
                let y = xchg!(CMD_Y_READ);
                let x = xchg!(CMD_X_READ);
                point.0 += i16::MAX as u32 - x;
                point.1 += y;
            }

            let z1 = xchg!(CMD_Z1_READ);
            let z2 = xchg!(CMD_Z2_READ);
            let z = z1 as i32 - z2 as i32;

            xchg!(0);
            self.cs.set_high().map_err(|e| Error::Pin(e))?;
            unsafe { set_spi_freq(60_000_000u32.Hz()) };

            if z < RELEASE_THRESHOLD {
                return Ok(None);
            }

            point.0 /= 10;
            point.1 /= 10;
            self.pressed = true;
            Ok(Some((
                point.0.saturating_sub(MIN_X) as f32 / (MAX_X - MIN_X) as f32,
                point.1.saturating_sub(MIN_Y) as f32 / (MAX_Y - MIN_Y) as f32,
            )))
        } else {
            Ok(None)
        }
    }
}

pub enum Error<PinE, TransferE> {
    Pin(PinE),
    Transfer(TransferE),
    InternalError,
}

unsafe fn set_spi_freq(freq: impl Into<fugit::Hertz<u32>>) {
    use rp_pico::hal;
    // FIXME: the touchscreen and the LCD have different frequencies, but we cannot really set different frequencies to different SpiProxy without this hack
    hal::spi::Spi::<_, _, 8>::new(hal::pac::Peripherals::steal().SPI1)
        .set_baudrate(125_000_000.Hz(), freq);
}
