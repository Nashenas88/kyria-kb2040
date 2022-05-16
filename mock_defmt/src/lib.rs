#![no_std]

#[macro_export]
macro_rules! info {
    ($($tt:tt)*) => {{}};
}

#[macro_export]
macro_rules! unreachable {
    ($($tt:tt)*) => {
        ::core::unreachable!($($tt)*)
    };
}
