use crate::config::*;

pub const ASK_9600: ChannelParameters = ChannelParameters {
    modulation: config::Modulation::ASK,
    encoding: Encoding::NRZ,
    framing: config::Framing::Raw,
    crc: CRC::None,
    datarate: 9_600,
    bitorder: BitOrder::LSBFirst,
};

pub const GMSK_60000: ChannelParameters = ChannelParameters {
    modulation: config::Modulation::GMSK {
        ramp: config::SlowRamp::Bits1,
        bt: BT(0.5),
    },
    encoding: Encoding::NRZISCR,
    framing: config::Framing::HDLC {
        fec: config::FEC {},
    },
    crc: CRC::CCITT { initial: 0xFFFF },
    datarate: 60_000,
    bitorder: BitOrder::MSBFirst,
};

pub const GMSK_96000: ChannelParameters = ChannelParameters {
    modulation: config::Modulation::GMSK {
        ramp: config::SlowRamp::Bits1,
        bt: BT(0.5),
    },
    encoding: Encoding::NRZISCR,
    framing: config::Framing::HDLC {
        fec: config::FEC {},
    },
    crc: CRC::CCITT { initial: 0xFFFF },
    datarate: 96_000,
    bitorder: BitOrder::MSBFirst,
};

pub const GMSK_9600_LSB: ChannelParameters = ChannelParameters {
    modulation: Modulation::GMSK {
        ramp: SlowRamp::Bits1,
        bt: BT(0.5),
    },
    encoding: Encoding::NRZISCR,
    framing: Framing::HDLC { fec: FEC {} },
    crc: CRC::CCITT { initial: 0xFFFF },
    datarate: 9_600,
    bitorder: BitOrder::LSBFirst,
};

pub const GMSK_9600_MSB: ChannelParameters = ChannelParameters {
    modulation: Modulation::GMSK {
        ramp: SlowRamp::Bits1,
        bt: BT(0.5),
    },
    encoding: Encoding::NRZISCR,
    framing: Framing::HDLC { fec: FEC {} },
    crc: CRC::CCITT { initial: 0xFFFF },
    datarate: 9_600,
    bitorder: BitOrder::MSBFirst,
};

