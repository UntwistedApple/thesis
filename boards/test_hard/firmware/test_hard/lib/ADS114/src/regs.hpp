enum reg_name {
    REG_ID,
    REG_STATUS,
    REG_INPMUX,
    REG_PGA,
    REG_DATARATE,
    REG_REF,
    REG_IDACMAG,
    REG_IDACMUX,
    REG_VBIAS,
    REG_SYS,
    REG_RESERVED,
    REG_OFCAL0,
    REG_OFCAL1,
    REG_RESERVED_2,
    REG_FSCAL0,
    REG_FSCAL1,
    REG_GPIODAT,
    REG_GPIOCON
};

#define DATARATE_MODE 5
enum datarate_mode {
    DATARATE_MODE_CONTINUOUS  = 0 << DATARATE_MODE,
    DATARATE_MODE_SINGLE_SHOT = 1 << DATARATE_MODE
};

#define DATARATE_CLK 6
enum datarate_clk {
    DATARATE_CLK_INTERNAL_4096MHZ = 0 << DATARATE_CLK,
    DATARATE_CLK_EXTERNAL = 1 << DATARATE_CLK
};

#define DATARATE_DR 0
#define DATARATE_DR_MASK 0b1111
enum datarate_dr {
    DATARATE_DR_2_5,
    DATARATE_DR_5,
    DATARATE_DR_10,
    DATARATE_DR_16_6,
    DATARATE_DR_20,
    DATARATE_DR_50,
    DATARATE_DR_60,
    DATARATE_DR_100,
    DATARATE_DR_200,
    DATARATE_DR_400,
    DATARATE_DR_800,
    DATARATE_DR_1000,
    DATARATE_DR_2000,
    DATARATE_DR_4000,
};

enum imag {
    IMAG_DIS,
    IMAG_10u,
    IMAG_50u,
    IMAG_100u,
    IMAG_250u,
    IMAG_500u,
    IMAG_750u,
    IMAG_1000u,
    IMAG_1500u,
    IMAG_2000u
};

#define I1MUX 0
#define I1MUX_MASK 0b1111
enum i1mux {
    I1MUX_AIN0,
    I1MUX_AIN1,
    I1MUX_AIN2,
    I1MUX_AIN3,
    I1MUX_AIN4,
    I1MUX_AIN5,
    I1MUX_AIN6,
    I1MUX_AIN7,
    I1MUX_AIN8,
    I1MUX_AIN9,
    I1MUX_AIN10,
    I1MUX_AIN11,
    I1MUX_AINCOM,
    I1MUX_DIS
};

#define I2MUX 4
#define I2MUX_MASK 0b11110000
enum i2mux {
    I2MUX_AIN0 = 0 << I2MUX,
    I2MUX_AIN1 = 1 << I2MUX,
    I2MUX_AIN2 = 2 << I2MUX,
    I2MUX_AIN3 = 3 << I2MUX,
    I2MUX_AIN4 = 4 << I2MUX,
    I2MUX_AIN5 = 5 << I2MUX,
    I2MUX_AIN6 = 6 << I2MUX,
    I2MUX_AIN7 = 7 << I2MUX,
    I2MUX_AIN8 = 8 << I2MUX,
    I2MUX_AIN9 = 9 << I2MUX,
    I2MUX_AIN10 = 10 << I2MUX,
    I2MUX_AIN11 = 11 << I2MUX,
    I2MUX_AINCOM = 12 << I2MUX,
    I2MUX_DIS = 13 << I2MUX
};

#define SYS_MON 5
#define SYS_MON_MASK 0b11100000
enum sys_mon {
    SYS_MON_DIS         = 0 << SYS_MON,
    SYS_MON_PGA_SHORTED = 1 << SYS_MON,
    SYS_MON_TEMP        = 2 << SYS_MON,
    SYS_MON_AVDD        = 3 << SYS_MON,
    SYS_MON_DVDD        = 4 << SYS_MON,
    SYS_MON_BOCS_0_2    = 5 << SYS_MON,
    SYS_MON_BOCS_1      = 6 << SYS_MON,
    SYS_MON_BOCS_10     = 7 << SYS_MON
};

#define CAL_SAMP 3
#define CAL_SAMP_MASK 0b11000
enum cal_samp {
    CAL_SAMP_1 = 0 << CAL_SAMP,
    CAL_SAMP_2 = 1 << CAL_SAMP,
    CAL_SAMP_4 = 2 << CAL_SAMP,
    CAL_SAMP_8 = 3 << CAL_SAMP
};

// MISSING: TIMEOUT

#define MUXP 4
#define MUXP_MASK 0b11110000
enum inpmux_muxp {
    MUXP_AIN0 = 0 << MUXP,
    MUXP_AIN1 = 1 << MUXP,
    MUXP_AIN2 = 2 << MUXP,
    MUXP_AIN3 = 3 << MUXP,
    MUXP_AIN4 = 4 << MUXP,
    MUXP_AIN5 = 5 << MUXP,
    MUXP_AIN6 = 6 << MUXP,
    MUXP_AIN7 = 7 << MUXP,
    MUXP_AIN8 = 8 << MUXP,
    MUXP_AIN9 = 9 << MUXP,
    MUXP_AIN10 = 10 << MUXP,
    MUXP_AIN11 = 11 << MUXP,
    MUXP_AINCOM = 12 << MUXP,
};

#define MUXN 0
#define MUXN_MASK 0b00001111
enum inpmux_muxn {
    MUXN_AIN0,
    MUXN_AIN1,
    MUXN_AIN2,
    MUXN_AIN3,
    MUXN_AIN4,
    MUXN_AIN5,
    MUXN_AIN6,
    MUXN_AIN7,
    MUXN_AIN8,
    MUXN_AIN9,
    MUXN_AIN10,
    MUXN_AIN11,
    MUXN_AINCOM,
};

#define PGA_EN 3
#define PGA_EN_MASK 0b11000
enum pga_en {
    PGA_EN_DIS  = 0b00 << PGA_EN,
    PGA_EN_EN   = 0b01 << PGA_EN,
};

#define GAIN 0
#define GAIN_MASK 0b111
enum pga_gain {
    GAIN1,
    GAIN2,
    GAIN4,
    GAIN8,
    GAIN16,
    GAIN32,
    GAIN64,
    GAIN128
};

enum status {
    STATUS_FL_REF   = 0,
    STATUS_RDY      = 6,
    STATUS_FL_POR   = 7,
};

#define REFSEL 2             // Bits to shift left when adding to register
#define REFSEL_MASK 0b1100      // reg & *_MASK unsets all bits needed
enum refsel {
    REFSEL_REF0         = 0b00 << REFSEL,
    REFSEL_REF1         = 0b01 << REFSEL,
    REFSEL_REF_INTERNAL = 0b10 << REFSEL,
};

#define REFCON 0
#define REFCON_MASK 0b11
enum refcon {
    REFCON_INTERNAL_OFF,
    REFCON_INTERNAL_ON,
    REFCON_INTERNAL_ON_ALWAYS,
};

#define REFP_BUF 5
enum refp {
    REFP_BUF_EN  = 0 << REFP_BUF,
    REFP_BUF_DIS = 1 << REFP_BUF
};

#define REFN_BUF 4
enum refn {
    REFN_BUF_EN  = 0 << REFN_BUF,
    REFN_BUF_DIS = 1 << REFN_BUF
};