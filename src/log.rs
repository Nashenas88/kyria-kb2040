#[cfg(feature = "defmt")]
pub use defmt::{error, info, panic, trace, unreachable, warn};

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! unreachable_ {
    ($($args:tt)*) => {
        unreachable!($($args)*)
    };
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! consume_ {
    ($($arg:expr),* $(,)?) => {
        {
            $(let _ = $arg;)*
        }
    };
}

#[cfg(not(feature = "defmt"))]
pub use {consume_ as info, consume_ as error, unreachable_ as unreachable};
