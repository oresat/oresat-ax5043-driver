extern crate ax5043;
use std::io;

fn main() -> io::Result<()> {
    let spi1 = ax5043::open("/dev/spidev0.0")?;
    let spi2 = ax5043::open("/dev/spidev1.0")?;

    let mut radio1 = ax5043::registers(&spi1, &|status| println!("0.0:{:?}", status));
    let mut radio2 = ax5043::registers(&spi2, &|status| println!("1.0:{:?}", status));

    radio1.reset()?;
    radio2.reset()?;

    Ok(())
}
