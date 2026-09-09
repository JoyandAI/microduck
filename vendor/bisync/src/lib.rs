//! This crate can be used to easily write code for synchronous as well as asynchronous clients.
//!
//! The use should be straight-forward.
//! You can put this into your file where you'd like to have both synchronous and asynchronous submodules:
//!
//! ```
//! // lib.rs
//!
//! #[path = "."]
//! pub mod asynchronous {
//!     use bisync::asynchronous::*;
//!     mod inner;
//!     pub use inner::*;
//! }
//!
//! // here you could also add `#[cfg]` attributes to enable or disable this module
//! #[path = "."]
//! pub mod blocking {
//!     use bisync::synchronous::*;
//!     mod inner;
//!     pub use inner::*;
//! }
//! ```
//! (The reason why I recommend to copy-paste this instead of providing a macro that does it for you is because LSPs struggle with modules defined within macros)
//!
//! Then, within your `inner` module, you can write code that is async-generic:
//! ```
//! // inner.rs
//!
//! // these are all the available definitions:
//! use super::{bisync, only_sync, only_async, SYNC, ASYNC};
//!  
//! #[bisync]
//! pub async fn foo() -> String {
//!     bar().await
//! }
//!  
//! #[bisync]
//! async fn bar() -> String {
//!     if ASYNC {
//!         println!("We are in async code.");
//!     } else if SYNC {
//!         println!("We are in blocking code.");
//!     } else {
//!         panic!("This is neither async nor blocking code but a secret third thing.");
//!     }
//!  
//!     baz().await
//! }
//!  
//! #[only_sync]
//! fn baz() -> String {
//!     ureq::get("https://example.com")
//!         .call()
//!         .unwrap()
//!         .into_string()
//!         .unwrap()
//! }
//!  
//! #[only_async]
//! async fn baz() -> String {
//!     reqwest::get("https://example.com")
//!         .await
//!         .unwrap()
//!         .text()
//!         .await
//!         .unwrap()
//! }
//! ```
//!
//! Here, depending on if we are within async or sync code, we use a different library to perform the requests.
//! As you can see, we prevent duplicate definitions of `foo` and `bar` because they get generated twice,
//! once in synchronous form, and once in asynchronous form.
//!
#![no_std]

/// The definitions to use for the synchronous code variation
pub mod synchronous {
    /// Specialize an item to only be emitted in the asynchronous module
    pub use ::bisync_macros::internal_delete as only_async;
    /// Specialize an item to only be emitted in the synchronous module
    pub use ::bisync_macros::internal_noop as only_sync;
    /// Emit an item in both synchronous and asynchronous code, and adjust asynchronisity depending on location
    pub use ::bisync_macros::internal_strip_async as bisync;
    /// true in the synchronous module, otherwise false
    pub const SYNC: bool = true;
    /// true in the asynchronous module, otherwise false
    pub const ASYNC: bool = false;
}

/// The definitions to use for the asynchronous code variation
pub mod asynchronous {
    /// Specialize an item to only be emitted in the synchronous module
    pub use ::bisync_macros::internal_delete as only_sync;
    /// Specialize an item to only be emitted in the asynchronous module
    pub use ::bisync_macros::internal_noop as only_async;
    /// Emit an item in both synchronous and asynchronous code, and adjust asynchronisity depending on location
    pub use ::bisync_macros::internal_noop as bisync;
    /// true in the synchronous module, otherwise false
    pub const SYNC: bool = false;
    /// true in the asynchronous module, otherwise false
    pub const ASYNC: bool = true;
}

