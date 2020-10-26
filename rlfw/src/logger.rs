use core::fmt::Write;

use log::{Level, Log, Metadata, Record};

/// Generic logger implemented over `core::fmt::Write`.
pub struct Logger {
    pub level: Level,
    pub writer: Option<&'static mut dyn Write>,
}

unsafe impl Send for Logger {}

// We're definitely not sync atm eh
unsafe impl Sync for Logger {}

static mut SERIAL_LOGGER: Logger = Logger {
    level: Level::Info,
    writer: None,
};

impl Logger {
    /// Initialise a new
    pub fn init(level: Level, w: &'static mut dyn Write) {
        unsafe {
            // Set global SERIAL_LOGGER log context
            SERIAL_LOGGER.level = level;
            SERIAL_LOGGER.writer = Some(w);

            // Attach to logger
            let _ = log::set_logger_racy(&SERIAL_LOGGER);
            log::set_max_level(SERIAL_LOGGER.level.to_level_filter());
        }
    }
}

impl Log for Logger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= self.level
    }

    /// Parse a record into a logging string
    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let _ = unsafe {
                match record.module_path() {
                    Some(p) => write!(
                        SERIAL_LOGGER,
                        "[{}] {} - {}\r\n",
                        record.level(),
                        p,
                        record.args()
                    ),
                    None => write!(
                        SERIAL_LOGGER,
                        "[{}] - {}\r\n",
                        record.level(),
                        record.args()
                    ),
                }
            };
        }
    }

    /// Flush the log output
    fn flush(&self) {}
}

impl core::fmt::Write for Logger {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        // Fetch writer (if bound)
        let w = match self.writer.as_mut() {
            Some(w) => w,
            None => return Err(core::fmt::Error),
        };

        // Write out string
        w.write_str(s).map_err(|_| core::fmt::Error)?;

        Ok(())
    }
}
