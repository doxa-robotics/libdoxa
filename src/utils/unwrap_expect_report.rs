use vexide::smart::PortError;
use vexide_motorgroup::MotorGroupError;

/// A global store to hold ports which have had a disconnect error reported.
/// This is used to avoid spamming the logs with repeated disconnect errors.
static DEVICE_DISCONNECTED_PORTS: std::sync::Mutex<Option<std::collections::HashSet<u8>>> =
    std::sync::Mutex::new(None);

pub trait UnwrapExpectReportExt<T> {
    /// Reports a disconnect if there is an error, otherwise returns the value.
    ///
    /// If there is an error, this function will log it and return None.
    ///
    /// # Panics
    ///
    /// If the port/device type is mismatched, this function will *panic* to
    /// avoid masking configuration errors.
    fn unwrap_report(self) -> Option<T>;

    /// Reports a disconnect if there is an error with a custom message,
    /// otherwise returns the value.
    ///
    /// The message is formatted with the port number.
    ///
    /// # Panics
    ///
    /// If the port/device type is mismatched, this function will *panic* to
    /// avoid masking configuration errors.
    fn expect_report<M: std::fmt::Display>(self, msg: M) -> Option<T>;
}

impl<T> UnwrapExpectReportExt<T> for Result<T, PortError> {
    fn unwrap_report(self) -> Option<T> {
        self.expect_report("called `unwrap_report` on a `Err(PortError)` value")
    }

    fn expect_report<M: std::fmt::Display>(self, msg: M) -> Option<T> {
        match self {
            Err(err) => match err {
                PortError::Disconnected { port } => {
                    let mut locked = DEVICE_DISCONNECTED_PORTS
                        .lock()
                        .expect("could not lock mutex. this should never happen.");
                    if locked.is_none() {
                        *locked = Some(std::collections::HashSet::new());
                    }
                    let ports = locked.as_mut().unwrap();
                    if !ports.contains(&port) {
                        ports.insert(port);
                        log::error!(
                            "{}: device disconnected on port {} (report-only)",
                            msg,
                            port
                        );
                    }
                    None
                }
                PortError::IncorrectDevice {
                    port,
                    expected,
                    actual,
                } => {
                    panic!(
                        "{}: mismatched device type on port {}: expected {:?}, found {:?}",
                        msg, port, expected, actual
                    )
                }
            },
            Ok(value) => Some(value),
        }
    }
}

impl<T> UnwrapExpectReportExt<T> for Option<T> {
    fn unwrap_report(self) -> Option<T> {
        self.expect_report("called `unwrap_report` on a `None` value")
    }

    fn expect_report<M: std::fmt::Display>(self, msg: M) -> Option<T> {
        match self {
            None => {
                log::error!("{}: encountered None value (report-only)", msg);
                None
            }
            Some(value) => Some(value),
        }
    }
}

impl<T> UnwrapExpectReportExt<T> for Result<T, MotorGroupError<PortError, T>> {
    fn unwrap_report(self) -> Option<T> {
        self.expect_report("called `unwrap_report` on a `MotorGroupError` value")
    }

    fn expect_report<M: std::fmt::Display>(self, msg: M) -> Option<T> {
        match self {
            Err(err) => {
                for error in err.errors {
                    Result::<T, PortError>::Err(error).expect_report(&msg);
                }
                err.result
            }
            Ok(value) => Some(value),
        }
    }
}
