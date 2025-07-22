//! ads1256_reader.rs
//! Example Rust program to read ADS1256 ADC via SPI on Raspberry Pi 5
use spidev::{Spidev, SpidevOptions, SpiModeFlags};
use rppal::gpio::{Gpio, InputPin, OutputPin, Level};
use std::io::{Read, Write, BufWriter};
use std::thread::sleep;
use std::time::Duration;
use std::time::Instant;
use std::time::{SystemTime, UNIX_EPOCH};
use std::fs::File;

// ADS1256 commands
#[allow(dead_code)]
const CMD_WAKEUP: u8 = 0x00;
const CMD_RDATA: u8 = 0x01;
#[allow(dead_code)]
const CMD_RDATAC: u8 = 0x03;
const CMD_SDATAC: u8 = 0x0F;
const CMD_WREG: u8 = 0x50;  // Write Register command - this was missing!
const CMD_RESET: u8 = 0xFE;

// Register addresses
const REG_ADCON:  u8 = 0x02;
const REG_DRATE:  u8 = 0x03;

// PGA gain codes (bits 2:0 of ADCON)
const GAIN_1X: u8 = 0x00; //   000

// Data‑rate codes (30 kSPS = default after reset)
const DRATE_30000SPS: u8 = 0xF0;

// Reference voltage for conversion (in volts)
const VREF: f64 = 5.0;

// Full-scale count for 24-bit bipolar ADC (2^23)
const FS_COUNT: f64 = (1 << 23) as f64;

const ZERO: f64 = 0.003;
const SPAN: f64 = 5.004;
const PMAX: f64 = 10.0;
const C: f64 = 100.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // SPI setup - match Python config: 20kHz speed, mode 1
    let mut spi = Spidev::open("/dev/spidev0.0")?;
    let options = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(1_000_000)  
        .mode(SpiModeFlags::SPI_MODE_1)
        .build();
    spi.configure(&options)?;

    // GPIO setup matching Python config
    let gpio = Gpio::new()?;
    
    // RST_PIN = 18 (reset pin)
    let mut rst_pin: OutputPin = gpio.get(18)?.into_output();
    
    // CS_PIN = 22 (chip select)  
    let mut cs_pin: OutputPin = gpio.get(22)?.into_output();
    
    // DRDY_PIN = 17 (data ready with pull-up)
    let drdy_pin: InputPin = gpio.get(17)?.into_input_pullup();

    // Hardware reset like Python ADS1256_reset()
    rst_pin.set_high();
    sleep(Duration::from_millis(200));
    rst_pin.set_low();
    sleep(Duration::from_millis(200));
    rst_pin.set_high();

    // Reset the ADC
    spi_write_with_cs(&mut spi, &mut cs_pin, &[CMD_RESET])?;
    sleep(Duration::from_millis(5));

    // Stop continuous read mode if active
    spi_write_with_cs(&mut spi, &mut cs_pin, &[CMD_SDATAC])?;
    sleep(Duration::from_millis(1));

    // Configure ADC like the Python version - write 4 registers starting from 0
    let buf = [
        (0<<3) | (1<<2) | (1<<1),  // REG_STATUS (0)
        0x08,                      // REG_MUX (1) 
        (0<<5) | (0<<3) | (GAIN_1X<<0),  // REG_ADCON (2) - gain in bits 2:0
        DRATE_30000SPS             // REG_DRATE (3)
    ];
    
    // Write to register 0, with 4 bytes (0x03 means write 4 bytes: 0x03 + 1)
    cs_pin.set_low();
    spi.write_all(&[CMD_WREG | 0, 0x03])?;
    spi.write_all(&buf)?;
    cs_pin.set_high();
    sleep(Duration::from_millis(1));

    
    let interval = Duration::from_millis(5);
    let mut last_tick = Instant::now();

    let mut time_vals: Vec<u128> = Vec::new(); // empty vector

    let file = File::create("time_data.txt")?;
    let mut writer = BufWriter::new(file);

    while time_vals.len() < 10_000 {

        let now = Instant::now();

        if now.duration_since(last_tick) >= interval {

            // Wait for DRDY low (match Python ADS1256_WaitDRDY)
            let mut timeout = 0;
            while drdy_pin.is_high() {
                timeout += 1;
                if timeout >= 400000 {
                    println!("Time Out ...");
                    break;
                }
            }

            // Send RDATA and read 3 bytes (match Python ADS1256_Read_ADC_Data)
            cs_pin.set_low();
            spi.write_all(&[CMD_RDATA])?;
            let mut buf = [0u8; 3];
            spi.read(&mut buf)?;
            cs_pin.set_high();

            // Assemble 24-bit signed value
            let raw = ((buf[0] as u32) << 16) | ((buf[1] as u32) << 8) | (buf[2] as u32);
            
            // Convert 24-bit unsigned to signed (proper sign extension)
            let value = if raw & 0x800000 != 0 {
                // Negative: sign extend by OR-ing with 0xFF000000
                (raw | 0xFF000000) as i32
            } else {
                // Positive: just cast to i32
                raw as i32
            };

            
            let delta = last_tick.elapsed();
            time_vals.push(delta.as_millis());

            // Use current timestamp in milliseconds
            let timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)?
                .as_millis();

            // Prometheus line protocol format:
            // metric_name{label="value"} <value> <timestamp>
            writeln!(
                writer,
                "my_metric{{source=\"rust\"}} {:?} {}",
                delta, timestamp
            )?;

            // Convert to voltage
            let voltage = (value as f64) * VREF / FS_COUNT;
            let pressure = (voltage - ZERO) / SPAN * PMAX * C;

            last_tick = now;
            // sleep(Duration::from_millis(1));

            // Print results
            println!("Raw ADC count: {} | Voltage: {:.2} V | Pressure: {:.2} kPa | time: {:?}", value, voltage, pressure, delta);

        }

        
    }

    // print time delta results
    let sum: u128 = time_vals.iter().sum();
    let mean = sum as f64 / time_vals.len() as f64;
    let max = Duration::from_millis(*time_vals.iter().max().unwrap() as u64);
    let min = Duration::from_millis(*time_vals.iter().min().unwrap() as u64);

    println!("Number of observations: {}", time_vals.len());
    println!("Mean time (ms): {:.2}", mean);
    println!("Max time (ms): {:?}", max);
    println!("Min time (ms): {:?}", min);

    Ok(())
}

fn spi_write_with_cs(spi: &mut Spidev, cs_pin: &mut OutputPin, data: &[u8]) -> std::io::Result<()> {
    cs_pin.set_low();
    spi.write_all(data)?;
    cs_pin.set_high();
    Ok(())
}
