use spidev::{Spidev, SpidevOptions, SpiModeFlags};
use rppal::gpio::{Gpio, InputPin, OutputPin, Level};
use std::{thread, time::Duration};
use std::io::{Read, Write, BufWriter};

// GPIO PINS
const RST_PIN: u8 = 18;   // Reset pin number
const CS_PIN: u8 = 22;    // Chip select pin number
const DRDY_PIN: u8 = 17;  // Data Ready pin number

#[derive(Copy, Clone)]
pub enum Pin {
    RST,
    CS,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum Register {
    Status = 0x00,
    Mux    = 0x01,
    Adcon  = 0x02,
    Drate  = 0x03,
    Io     = 0x04,
    Ofc0   = 0x05,
    Ofc1   = 0x06,
    Ofc2   = 0x07,
    Fsc0   = 0x08,
    Fsc1   = 0x09,
    Fsc2   = 0x0A,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum Command {
    Wakeup = 0x00,     // Completes SYNC and Exits Standby Mode 0000  0000 (00h)
    Rdata= 0x01,       // Read Data 0000  0001 (01h)
    Rdatac = 0x03,     // Read Data Continuously 0000   0011 (03h)
    Sdatac = 0x0F,     // Stop Read Data Continuously 0000   1111 (0Fh)
    Rreg = 0x10,       // Read from REG rrr 0001 rrrr (1xh)
    Wreg = 0x50,       // Write to REG rrr 0101 rrrr (5xh)
    Selfcal = 0xF0,    // Offset and Gain Self-Calibration 1111    0000 (F0h)
    Selfocal = 0xF1,   // Offset Self-Calibration 1111    0001 (F1h)
    Selfgcal = 0xF2,   // Gain Self-Calibration 1111    0010 (F2h)
    Sysocal = 0xF3,    // System Offset Calibration 1111   0011 (F3h)
    Sysgcal = 0xF4,    // System Gain Calibration 1111    0100 (F4h)
    Sync = 0xFC,       // Synchronize the A/D Conversion 1111   1100 (FCh)
    Standby = 0xFD,    // Begin Standby Mode 1111   1101 (FDh)
    Reset = 0xFE,      // Reset to Power-Up Values 1111   1110 (FEh)
}


pub struct Ads1256 {
    spi: Spidev, // spi device
    drdy_pin: InputPin, // data ready pin with pull up
    rst_pin: OutputPin, // reset pin
    cs_pin: OutputPin, // chip slect pin
}

impl Ads1256 {
    

    pub fn new() -> Self {

        let mut spi = Spidev::open("/dev/spidev0.0").unwrap();
        let options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(1_000_000)  
            .mode(SpiModeFlags::SPI_MODE_1)
            .build();
        spi.configure(&options).unwrap();

        let gpio = Gpio::new().unwrap();
        let drdy_pin = gpio.get(Pin::DRDY_PIN).unwrap().into_input_pullup(); // drdy with pullup
        let rst_pin = gpio.get(Pin::RST_PIN).unwrap().into_output();   // reset
        let cs_pin = gpio.get(Pin::CS_PIN).unwrap().into_output();  // chip select

        Self {
            spi,
            drdy_pin,
            rst_pin,
            cs_pin,
        }
    }

    
    pub fn digital_write(&mut self, pin: Pin, data: bool) {
        // function set reset or chip select pin high or low
        match pin {
            Pin::RST => self.rst_pin.write(if value { Level::HIGH } else { Level::LOW }),
            Pin::CS => self.cs_pin.write(if value { Level::HIGH } else { Level::LOW }),

        }
        
    }

    pub fn digital_read(&self) -> bool {
        // function determine if data ready pin is high
        self.drdy_pin.is_high()
    }

    pub fn delay_ms(&self, ms: u64) {
        // function to sleep for specified amount of time
        // needed to allow adc to settle
        thread::sleep(Duration::from_millis(ms));
    }

    pub fn reset(&self) {
        // harward reset
        digital_write(RST, Level::HIGH);
        delay_ms(200);
        digital_write(RST, Level::LOW);
        delay_ms(200);
        digital_write(RST, Level::HIGH);
    }

    pub fn write_cmd(&self, reg: u8) {
        digital_write(CS, Level::LOW);
        self.spi.write_all(&[reg]).unwrap();
        digital_write(CS, Level::LOW);
    }

    pub fn write_reg(&self, reg: u8, data: u8) {
        let cmd = (Command::WReg as u8) | (reg as u8);
        let count = 0x00; // writing 1 byte

        digital_write(CS, Level::LOW);
        self.spi.write_all(&[cmd, count, data]).unwrap();
        digital_write(CS, Level::HIGH);
    }

    pub fn read_data(&self, reg: u8) -> u8 {
        let cmd = (Command::WReg as u8) | (reg as u8);
        let count = 0x00; // writing 1 byte

        digital_write(CS, Level::LOW);
        self.spi.write_all(&[cmd, count, data]).unwrap();

        let mut buf = [0u8; 1];
        spi.read(&mut buf);

        digital_write(CS, Level::HIGH);

        buf[0]
    }

    


}
