#[cfg(feature = "defmt")]
pub use defmt::{error, info, panic, trace, warn};

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! unreachable_ {
    ($($args:tt)*) => {
        unreachable!($($args)*)
    };
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! info_ {
    ($($arg:expr),* $(,)?) => {
        {
            $(let _ = $arg;)*
        }
    };
}

pub use {info_ as info, unreachable_ as unreachable};
