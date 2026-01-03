use alloc::{boxed::Box, sync::Arc};
use core::time::Duration;
use std::{
    fs::File,
    io::{Write, stdout},
};

use log::{Level, Metadata, Record, SetLoggerError, error};
use vexide::{prelude::spawn, sync::Mutex};

struct SimpleLogger {
    file: Arc<Mutex<Option<File>>>,
    start_time: std::time::Instant,
}
// Safety: The brain is single-threaded. Hopefully.
unsafe impl core::marker::Sync for SimpleLogger {}
unsafe impl core::marker::Send for SimpleLogger {}

impl SimpleLogger {
    pub fn new(file: &str) -> Self {
        Self {
            // We're being very unsafe here.
            #[allow(clippy::arc_with_non_send_sync)]
            file: Arc::new(Mutex::new(
                File::options().append(true).create(true).open(file).ok(),
            )),
            start_time: std::time::Instant::now(),
        }
    }
}

impl log::Log for SimpleLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Debug
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            // Only write to stdout if we're not connected to the competition field control
            // If we're connected to the field control, writing to stdout doesn't go
            // anywhere and is a waste of time.
            if !matches!(
                vexide::competition::system(),
                Some(vexide::competition::CompetitionSystem::FieldControl)
            ) {
                println!("{:<5} - {}", record.level(), record.args());
            }
            if let Some(mut guard) = self.file.try_lock() {
                if let Some(file) = guard.as_mut() {
                    _ = writeln!(
                        file,
                        "{:>3}.{:03} {:<5} {:<52} - {}",
                        self.start_time.elapsed().as_secs(),
                        self.start_time.elapsed().subsec_millis(),
                        record.level(),
                        record.module_path().unwrap_or("<unknown>"),
                        record.args()
                    )
                }
            } else {
                spawn(async {
                    vexide::time::sleep(Duration::from_millis(100)).await;
                    error!("Failed to lock log file (slept 100ms). 1 message was ignored.");
                })
                .detach();
            }
            self.flush();
        }
    }

    fn flush(&self) {
        let file = self.file.clone();
        spawn(async move {
            _ = stdout().flush();
            if let Some(file) = file.lock().await.as_mut() {
                _ = file.flush();
            }
        })
        .detach();
    }
}

pub fn init(file: &str, max_level: log::LevelFilter) -> Result<(), SetLoggerError> {
    let logger = SimpleLogger::new(file);
    let logger_ref = Box::leak(Box::new(logger));
    log::set_logger(logger_ref).map(|()| log::set_max_level(max_level))
}
