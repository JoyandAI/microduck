# bisync

> ⚠️ **Notice: This crate has been yanked and deprecated**
>
> **Why was this done?**
> The author does not wish to be a software supply chain risk for a single, tiny utility.
> Due to the basic nature of this crate, it requires extremely infrequent updating and maintenance and all of its code can be copy-pasted (vendored in) without issue.

Easily write synchronous and asynchronous code at the same time.

This crate is inspired by [maybe-async](https://github.com/fMeow/maybe-async-rs), but uses a different approach and tries to be more minimalistic.

## How do I use it?
Annotate the functions you'd like to have generic w.r.t. asynchronisity with `#[bisync]`.
You can specialize functions by annotating them with `#[only_sync]` or `#[only_async]`.

You need a tiny bit of boilerplate in the parent module but that's all there is to it.

## Example
```rs
// lib.rs

#[path = "."]
pub mod asynchronous {
    use bisync::asynchronous::*;
    mod inner;
    pub use inner::*;
}

// here you could also add `#[cfg]` attributes to enable or disable this module
#[path = "."]
pub mod blocking {
    use bisync::synchronous::*;
    mod inner;
    pub use inner::*;
}
```

```rs
// inner.rs

// these are all the available definitions:
use super::{bisync, only_sync, only_async, SYNC, ASYNC};
 
#[bisync]
pub async fn foo() -> String {
    bar().await
}
 
#[bisync]
async fn bar() -> String {
    if ASYNC {
        println!("We are in async code.");
    } else if SYNC {
        println!("We are in blocking code.");
    } else {
        panic!("This is neither async nor blocking code but a secret third thing.");
    }
 
    baz().await
}
 
#[only_sync]
fn baz() -> String {
    ureq::get("https://example.com")
        .call()
        .unwrap()
        .into_string()
        .unwrap()
}
 
#[only_async]
async fn baz() -> String {
    reqwest::get("https://example.com")
        .await
        .unwrap()
        .text()
        .await
        .unwrap()
}
```

The example above will strip away all `async` and `await` from the functions in the `blocking` module, but will leave them in the `asynchronous` module.
Hence, you can easily use the functions either in a synchronous or an asynchronous context.
