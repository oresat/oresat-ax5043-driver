use anyhow::Result;
use ax5043::{Registers, RX, TX, Status};

fn main() -> Result<()> {
    let spi = ax5043::open("/dev/spidev0.0")?;
    let mut status = Status::empty();
    let mut callback = |_: &_, _, s, _: &_| {
        if s != status {
            println!("TX Status change: {:?}", s);
            status = s;
        }
    };
    let mut radio = Registers::new(spi, &mut callback);

    println!("Revision: {:?}", radio.REVISION().read()?);
    radio.SCRATCH().write(0x52)?;
    println!("Scratch: {:X?}", radio.SCRATCH().read()?);
    radio.SCRATCH().write(0xF1)?;
    println!("Scratch: {:X?}", radio.SCRATCH().read()?);
    println!("Powstat: {:?}", radio.POWSTAT().read()?);
    println!("PowStickyStat: {:?}", radio.POWSTICKYSTAT().read()?);
    println!("PowIRQMask: {:?}", radio.POWIRQMASK().read()?);

    radio.reset()?;
    Ok(())
}
