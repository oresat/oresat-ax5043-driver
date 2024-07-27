extern crate ax5043;
use anyhow::Result;
use clap::Parser;
use std::path::PathBuf;

#[derive(Parser, Debug)]
struct Args {
    #[cfg(card = "c3")]
    #[arg(default_values_os_t=vec![PathBuf::from("/dev/spidev0.0"), PathBuf::from("/dev/spidev1.1")])]
    path: Vec<PathBuf>,

    #[cfg(card = "rpi")]
    #[arg(default_values_os_t=vec![PathBuf::from("/dev/spidev0.0"), PathBuf::from("/dev/spidev1.0")])]
    path: Vec<PathBuf>,

    #[cfg(all(not(card = "rpi"), not(card = "c3")))]
    #[arg(short, long)]
    path: Vec<PathBuf>,
}

fn main() -> Result<()> {
    let args = Args::parse();

    let callback = &mut |_: &_, _, status, _: &_| {
        println!("{:?}", status);
    };

    for path in args.path.iter() {
        println!("Resetting {}", path.display());
        let spi = ax5043::open(path)?;
        let mut radio = ax5043::Registers::new(spi, callback);
        radio.reset()?;
    }
    Ok(())
}
