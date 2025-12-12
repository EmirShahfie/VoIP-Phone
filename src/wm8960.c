#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <wm8960.h>

uint16_t _registerLocalCopy[56] = {
    0x0097, // R0 (0x00)
    0x0097, // R1 (0x01)
    0x0000, // R2 (0x02)
    0x0000, // R3 (0x03)
    0x0000, // R4 (0x04)
    0x0008, // F5 (0x05)
    0x0000, // R6 (0x06)
    0x000A, // R7 (0x07)
    0x01C0, // R8 (0x08)
    0x0000, // R9 (0x09)
    0x00FF, // R10 (0x0a)
    0x00FF, // R11 (0x0b)
    0x0000, // R12 (0x0C) RESERVED
    0x0000, // R13 (0x0D) RESERVED
    0x0000, // R14 (0x0E) RESERVED
    0x0000, // R15 (0x0F) RESERVED
    0x0000, // R16 (0x10)
    0x007B, // R17 (0x11)
    0x0100, // R18 (0x12)
    0x0032, // R19 (0x13)
    0x0000, // R20 (0x14)
    0x00C3, // R21 (0x15)
    0x00C3, // R22 (0x16)
    0x01C0, // R23 (0x17)
    0x0000, // R24 (0x18)
    0x0000, // R25 (0x19)
    0x0000, // R26 (0x1A)
    0x0000, // R27 (0x1B)
    0x0000, // R28 (0x1C)
    0x0000, // R29 (0x1D)
    0x0000, // R30 (0x1E) RESERVED
    0x0000, // R31 (0x1F) RESERVED
    0x0100, // R32 (0x20)
    0x0100, // R33 (0x21)
    0x0050, // R34 (0x22)
    0x0000, // R35 (0x23) RESERVED
    0x0000, // R36 (0x24) RESERVED
    0x0050, // R37 (0x25)
    0x0000, // R38 (0x26)
    0x0000, // R39 (0x27)
    0x0000, // R40 (0x28)
    0x0000, // R41 (0x29)
    0x0040, // R42 (0x2A)
    0x0000, // R43 (0x2B)
    0x0000, // R44 (0x2C)
    0x0050, // R45 (0x2D)
    0x0050, // R46 (0x2E)
    0x0000, // R47 (0x2F)
    0x0002, // R48 (0x30)
    0x0037, // R49 (0x31)
    0x0000, // R50 (0x32) RESERVED
    0x0080, // R51 (0x33)
    0x0008, // R52 (0x34)
    0x0031, // R53 (0x35)
    0x0026, // R54 (0x36)
    0x00e9, // R55 (0x37)
};

const uint16_t _registerDefaults[56] = {
    0x0097, // R0 (0x00)
    0x0097, // R1 (0x01)
    0x0000, // R2 (0x02)
    0x0000, // R3 (0x03)
    0x0000, // R4 (0x04)
    0x0008, // F5 (0x05)
    0x0000, // R6 (0x06)
    0x000A, // R7 (0x07)
    0x01C0, // R8 (0x08)
    0x0000, // R9 (0x09)
    0x00FF, // R10 (0x0a)
    0x00FF, // R11 (0x0b)
    0x0000, // R12 (0x0C) RESERVED
    0x0000, // R13 (0x0D) RESERVED
    0x0000, // R14 (0x0E) RESERVED
    0x0000, // R15 (0x0F) RESERVED
    0x0000, // R16 (0x10)
    0x007B, // R17 (0x11)
    0x0100, // R18 (0x12)
    0x0032, // R19 (0x13)
    0x0000, // R20 (0x14)
    0x00C3, // R21 (0x15)
    0x00C3, // R22 (0x16)
    0x01C0, // R23 (0x17)
    0x0000, // R24 (0x18)
    0x0000, // R25 (0x19)
    0x0000, // R26 (0x1A)
    0x0000, // R27 (0x1B)
    0x0000, // R28 (0x1C)
    0x0000, // R29 (0x1D)
    0x0000, // R30 (0x1E) RESERVED
    0x0000, // R31 (0x1F) RESERVED
    0x0100, // R32 (0x20)
    0x0100, // R33 (0x21)
    0x0050, // R34 (0x22)
    0x0000, // R35 (0x23) RESERVED
    0x0000, // R36 (0x24) RESERVED
    0x0050, // R37 (0x25)
    0x0000, // R38 (0x26)
    0x0000, // R39 (0x27)
    0x0000, // R40 (0x28)
    0x0000, // R41 (0x29)
    0x0040, // R42 (0x2A)
    0x0000, // R43 (0x2B)
    0x0000, // R44 (0x2C)
    0x0050, // R45 (0x2D)
    0x0050, // R46 (0x2E)
    0x0000, // R47 (0x2F)
    0x0002, // R48 (0x30)
    0x0037, // R49 (0x31)
    0x0000, // R50 (0x32) RESERVED
    0x0080, // R51 (0x33)
    0x0008, // R52 (0x34)
    0x0031, // R53 (0x35)
    0x0026, // R54 (0x36)
    0x00e9, // R55 (0x37)
};

int wm8960_write_register(const struct i2c_dt_spec *i2c, uint8_t reg, uint16_t val)
{
    uint8_t buf[2];

    buf[0] = (reg << 1) | ((val >> 8) & 0x01);
    buf[1] = val & 0xFF;

    return i2c_write_dt(i2c, buf, sizeof(buf));
}

int wm8960_write_register_bit(const struct i2c_dt_spec *i2c_dev, uint8_t reg, uint8_t bit_pos, uint8_t bit_val) {
    uint16_t reg_val = _registerLocalCopy[reg];
    if (bit_val) {
        reg_val |= (1 << bit_pos);
    } else {
        reg_val &= ~(1 << bit_pos);
    }

    int ret = wm8960_write_register(i2c_dev, reg, reg_val);
    if (ret) {
        printk("Failed to write to WM8960 register 0x%02X\n", reg);
        return ret;
    } else {
        _registerLocalCopy[reg] = reg_val;
        return 0;
    }
    return 0;
}

int wm8960_write_register_multi_bits(const struct i2c_dt_spec *i2c_dev,
                                     uint8_t reg,
                                     uint8_t bit_pos,
                                     uint8_t bit_len,
                                     uint16_t value)
{
    uint16_t reg_val = _registerLocalCopy[reg];

    uint16_t mask = ((1U << bit_len) - 1U) << bit_pos;
    reg_val = (reg_val & ~mask) | ((value << bit_pos) & mask);

    int ret = wm8960_write_register(i2c_dev, reg, reg_val);
    if (!ret) {
        _registerLocalCopy[reg] = reg_val;
    } else {
        printk("WM8960 multi-bit write failed @ reg 0x%02X (ret=%d)\n", reg, ret);
    }

    return ret;
}


int wm8960_reset(const struct i2c_dt_spec *i2c_dev)
{
    int ret = wm8960_write_register(i2c_dev, WM8960_REG_RESET, 0x0001);
    if (ret) {
        printk("WM8960 reset I2C error: %d\n", ret);
        return ret;
    }

    /* Reset local mirror to defaults */
    for (int i = 0; i < 56; i++) {
        _registerLocalCopy[i] = _registerDefaults[i];
    }

    return 0;
}


int wm8960_setup(const struct i2c_dt_spec *i2c_dev)
{
    int ret;

    /* 1. Reset and basic power */
    ret = wm8960_reset(i2c_dev);
    if (ret) {
        printk("WM8960 reset failed\n");
        return ret;
    }

    /* enableVREF(): PWR_MGMT_1 bit 6 = 1 */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_PWR_MGMT_1, 6, 1);
    if (ret) return ret;

    /* setVMID(WM8960_VMIDSEL_2X50KOHM): bits [8:7] = 0b10 */
    ret = wm8960_write_register_multi_bits(i2c_dev,
                                           WM8960_REG_PWR_MGMT_1,
                                           7, 2, 0b10);
    if (ret) return ret;

    /* Small delay to let VMID cap charge */
    k_msleep(50);

    /* 2. Route DAC outputs to the output mixers */
    /* enableLD2LO(): LEFT_OUT_MIX_1 bit 8 */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_LEFT_OUT_MIX_1, 8, 1);
    if (ret) return ret;

    /* enableRD2RO(): RIGHT_OUT_MIX_2 bit 8 */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_RIGHT_OUT_MIX_2, 8, 1);
    if (ret) return ret;

    /* setLB2LOVOL(-21dB) and setRB2ROVOL(-21dB):
     * BYPASS_1/2 bits [6:4] = 7 (datasheet: 0 = 0dB ... 7 = -21dB)
     */
    ret = wm8960_write_register_multi_bits(i2c_dev,
                                           WM8960_REG_BYPASS_1,
                                           4, 3, 7);
    if (ret) return ret;

    ret = wm8960_write_register_multi_bits(i2c_dev,
                                           WM8960_REG_BYPASS_2,
                                           4, 3, 7);
    if (ret) return ret;

    /* enableLOMIX(): PWR_MGMT_3 bit 3 */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_PWR_MGMT_3, 3, 1);
    if (ret) return ret;

    /* enableROMIX(): PWR_MGMT_3 bit 2 */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_PWR_MGMT_3, 2, 1);
    if (ret) return ret;

    /* 3. PLL and clocking for 44.1 kHz, 16-bit, BCLK = 64fs, DCLK = 705.6 kHz */

    /* enablePLL(): PWR_MGMT_2 bit 0 = 1 */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_PWR_MGMT_2, 0, 1);
    if (ret) return ret;

    /* setPLLPRESCALE(DIV_2): PLL_N bit 4 = 1 */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_PLL_N, 4, 1);
    if (ret) return ret;

    /* setSMD(FRACTIONAL): PLL_N bit 5 = 1 */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_PLL_N, 5, 1);
    if (ret) return ret;

    /* setPLLN(7): PLL_N bits [3:0] = 0b0111 */
    ret = wm8960_write_register_multi_bits(i2c_dev,
                                           WM8960_REG_PLL_N,
                                           0, 4, 7);
    if (ret) return ret;

    /* setPLLK(0x86, 0xC2, 0x26): PLLK = 0x86C226
     * K1: bits [5:0] of upper byte
     * K2: bits [8:0] mid
     * K3: bits [8:0] low
     * The SparkFun lib just writes the 8-bit values into these fields,
     * so we mirror that.
     */
    ret = wm8960_write_register_multi_bits(i2c_dev,
                                           WM8960_REG_PLL_K_1,
                                           0, 6, 0x86);
    if (ret) return ret;

    ret = wm8960_write_register_multi_bits(i2c_dev,
                                           WM8960_REG_PLL_K_2,
                                           0, 9, 0xC2);
    if (ret) return ret;

    ret = wm8960_write_register_multi_bits(i2c_dev,
                                           WM8960_REG_PLL_K_3,
                                           0, 9, 0x26);
    if (ret) return ret;

    /* setCLKSEL(PLL): CLOCKING_1 bit 0 = 1 */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_CLOCKING_1, 0, 1);
    if (ret) return ret;

    /* setSYSCLKDIV(DIV_BY_2): CLOCKING_1 bits [2:1] = 0b10 */
    ret = wm8960_write_register_multi_bits(i2c_dev,
                                           WM8960_REG_CLOCKING_1,
                                           1, 2, 0b10);
    if (ret) return ret;

    /* setADCDIV(0) and setDACDIV(0) are defaults, no need to touch. */

    /* setBCLKDIV(4): CLOCKING_2 bits [3:0] = 0b0100 */
    ret = wm8960_write_register_multi_bits(i2c_dev,
                                           WM8960_REG_CLOCKING_2,
                                           0, 4, 4);
    if (ret) return ret;

    /* setDCLKDIV(16): CLOCKING_2 bits [8:6] = 0b111 */
    ret = wm8960_write_register_multi_bits(i2c_dev,
                                           WM8960_REG_CLOCKING_2,
                                           6, 3, 0b111);
    if (ret) return ret;

    /* setWL(16BIT): AUDIO_INTERFACE_1 bits [3:2] = 0b00 */
    ret = wm8960_write_register_multi_bits(i2c_dev,
                                           WM8960_REG_AUDIO_INTERFACE_1,
                                           2, 2, 0b00);
    if (ret) return ret;

    /* enablePeripheralMode(): AUDIO_INTERFACE_1 bit 6 = 0
     * (default is 0, but make it explicit)
     */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_AUDIO_INTERFACE_1, 6, 0);
    if (ret) return ret;

    /* 4. Enable DACs, unmute, and bring up headphones */

    /* enableDacLeft(): PWR_MGMT_2 bit 8 */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_PWR_MGMT_2, 8, 1);
    if (ret) return ret;

    /* enableDacRight(): PWR_MGMT_2 bit 7 */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_PWR_MGMT_2, 7, 1);
    if (ret) return ret;

    /* disableLoopBack(): AUDIO_INTERFACE_2 bit 0 = 0 (already default) */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_AUDIO_INTERFACE_2, 0, 0);
    if (ret) return ret;

    /* disableDacMute(): ADC_DAC_CTRL_1 bit 3 = 0 */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_ADC_DAC_CTRL_1, 3, 0);
    if (ret) return ret;

    /* enableHeadphones(): PWR_MGMT_2 bits [6] & [5] */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_PWR_MGMT_2, 6, 1); /* LOUT1 */
    if (ret) return ret;

    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_PWR_MGMT_2, 5, 1); /* ROUT1 */
    if (ret) return ret;

    /* enableOUT3MIX(): PWR_MGMT_2 bit 1 (VMID buffer for HP ground) */
    ret = wm8960_write_register_bit(i2c_dev, WM8960_REG_PWR_MGMT_2, 1, 1);
    if (ret) return ret;

    /* 5. Set headphone volume to ~0 dB
     *
     * From SparkFun comments: valid input 47–127
     * 48 = -73 dB, 127 = +6 dB, step 1 dB
     * So: dB = setting - 121  => setting = dB + 121
     * For 0 dB, setting = 121.
     * LOUT1/ROUT1_VOLUME: bits [6:0] = volume, bit 8 = OUT1VU (update)
     */
    uint8_t hp_vol = 121;

    ret = wm8960_write_register_multi_bits(i2c_dev,
                                           WM8960_REG_LOUT1_VOLUME,
                                           0, 7, hp_vol);
    if (ret) return ret;
    ret = wm8960_write_register_bit(i2c_dev,
                                    WM8960_REG_LOUT1_VOLUME,
                                    8, 1); /* OUT1VU */
    if (ret) return ret;

    ret = wm8960_write_register_multi_bits(i2c_dev,
                                           WM8960_REG_ROUT1_VOLUME,
                                           0, 7, hp_vol);
    if (ret) return ret;
    ret = wm8960_write_register_bit(i2c_dev,
                                    WM8960_REG_ROUT1_VOLUME,
                                    8, 1); /* OUT1VU */
    if (ret) return ret;

    printk("WM8960 setup complete.\n");
    return 0;
}


