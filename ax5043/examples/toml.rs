use anyhow::Result;
use ax5043::config;
use clap::Parser;
use std::fs::read_to_string;
use toml;

#[derive(Parser)]
///Validates a toml radio config
struct Args {
    file: String,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let contents = read_to_string(args.file)?;
    let config: config::Config = toml::from_str(&contents)?;

    println!("{:#?}", config);
    Ok(())
}
