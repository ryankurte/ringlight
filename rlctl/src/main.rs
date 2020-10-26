
use log::{debug, info, error};
use simplelog::{TermLogger, LevelFilter, TerminalMode};
use structopt::StructOpt;

use hid_cli::*;

#[derive(Clone, PartialEq, Debug, StructOpt)]
#[structopt(name = "rlctl", about = "Ringlight Control Utility")]
pub struct Options {
    #[structopt(subcommand)]
    command: Command,

    #[structopt(long, default_value="1209", parse(try_from_str=u16_parse_hex), env="RL_USB_VID")]
    /// Ringling USB Device Vendor ID (VID) in hex
    pub vid: u16,

    #[structopt(long, default_value="fff1", parse(try_from_str=u16_parse_hex), env="RL_USB_PID")]
    /// Ringling USB Device Product ID (PID) in hex
    pub pid: u16,

    #[structopt(long, env = "RL_SERIAL")]
    /// Ringlight serial number for filtering
    pub serial: Option<String>,

    #[structopt(long = "log-level", default_value = "info")]
    /// Enable verbose logging
    level: LevelFilter,
}

#[derive(Clone, PartialEq, Debug, StructOpt)]
enum Command {
    /// Reset to default configuration
    Reset,
    /// Set output as RGB
    SetRgb {
        #[structopt(parse(try_from_str=u8_parse_radix))]
        r: u8,
        #[structopt(parse(try_from_str=u8_parse_radix))]
        g: u8,
        #[structopt(parse(try_from_str=u8_parse_radix))]
        b: u8,
    },
    /// Set output as HSV
    SetHsv {
        #[structopt(parse(try_from_str=u8_parse_radix))]
        h: u8,
        #[structopt(parse(try_from_str=u8_parse_radix))]
        s: u8,
        #[structopt(parse(try_from_str=u8_parse_radix))]
        v: u8,
    },
    /// Set brightness
    SetBrightness{
        #[structopt(parse(try_from_str=u8_parse_radix))]
        b: u8,
    },
    /// Enter bootloader
    Bootload,
}


pub fn u8_parse_radix(s: &str) -> Result<u8, std::num::ParseIntError> {
    match s.strip_prefix("0x") {
        Some(v) => u8::from_str_radix(v, 16),
        None => u8::from_str_radix(s, 10)
    }
}

fn main() {
    // Parse options
    let opts = Options::from_args();

    // Setup logging
    let mut config = simplelog::ConfigBuilder::new();
    config.set_time_level(LevelFilter::Off);

    TermLogger::init(opts.level, config.build(), TerminalMode::Mixed).unwrap();

    // Connect to led ring
    let mut d = match HidDevice::connect(opts.vid, opts.pid, opts.serial.as_deref()) {
        Ok(d) => d,
        Err(e) => {
            error!("Error connecting to device ({:02x}:{:02x}): {:?}", opts.vid, opts.pid, e);
            return;
        }
    };

    // Fetch info
    let i = d.info();
    debug!("Connected to device: {:02x?}", i);

    // Execute command
    let r = match opts.command {
        Command::Reset => {
            info!("Reset to default output");
            let cmd = [0x00, 0x00];
            d.write(&cmd)
        },
        Command::SetRgb{ r, g, b } => {
            info!("Set RGB: {} {} {}", r, g, b);
            let cmd = [0x00, 0x01, r, g, b];
            d.write(&cmd)
        },
        Command::SetHsv{ h, s, v } => {
            info!("Set HSV: {} {} {}", h, s, v);
            let cmd = [0x00, 0x02, h, s, v];
            d.write(&cmd)
        },
        Command::SetBrightness{ b } => {
            info!("Set brightness: {}", b);
            let cmd = [0x00, 0x03, b];
            d.write(&cmd)
        },
        Command::Bootload => {
            info!("Sending bootload command");
            let cmd = [0x00, 0xf0];
            d.write(&cmd)
        },
    };

    if let Err(e) = r {
        error!("Command error: {:?}", e);
    }
}
