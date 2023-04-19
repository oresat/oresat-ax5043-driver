extern crate ax5043;
use std::{cell::Cell, io};


fn main() -> io::Result<()> {
    let spi = ax5043::open("/dev/spidev0.0")?;
    let status = Cell::new(ax5043::Status::empty());
    let callback = |s| {
        if s != status.get() {
            println!("TX Status change: {:?}", s);
            status.set(s);
        }
    };
    let mut radio = ax5043::registers(&spi, &callback);

    println!("Revision: {:?}", radio.REVISION.read()?);
    radio.SCRATCH.write(0x52)?;
    println!("Scratch: {:X?}", radio.SCRATCH.read()?);
    radio.SCRATCH.write(0xF1)?;
    println!("Scratch: {:X?}", radio.SCRATCH.read()?);
    println!("Powstat: {:?}", radio.POWSTAT.read()?);
    println!("PowStickyStat: {:?}", radio.POWSTICKYSTAT.read()?);
    println!("PowIRQMask: {:?}", radio.POWIRQMASK.read()?);

    radio.reset()?;

    Ok(())
}
