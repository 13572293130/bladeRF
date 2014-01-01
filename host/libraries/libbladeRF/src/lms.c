/*
 * This file is part of the bladeRF project:
 *   http://www.github.com/nuand/bladeRF
 *
 * Copyright (C) 2013 Nuand LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/*
 * If you're diving into this file, have the following documentation handy.
 *
 * As most registers don't have a clearly defined names, or are not grouped by
 * a specific set of functionality, there's little value in providing named
 * macro definitions, hence the hard-coded addresses.
 *
 * LMS6002D Project page:
 *  http://www.limemicro.com/products/LMS6002D.php?sector=default
 *
 * LMS6002D Datasheet:
 *  http://www.limemicro.com/download/LMS6002Dr2-DataSheet-1.2r0.pdf
 *
 * LMS6002D Programming and Calibration Guide:
 *  http://www.limemicro.com/download/LMS6002Dr2-Programming_and_Calibration_Guide-1.1r1.pdf
 *
 * LMS6002D FAQ:
 *  http://www.limemicro.com/download/FAQ_v1.0r10.pdf
 *
 */
#include <libbladeRF.h>
#include "lms.h"
#include "bladerf_priv.h"
#include "log.h"
#include "rel_assert.h"

#define kHz(x) (x * 1000)
#define MHz(x) (x * 1000000)
#define GHz(x) (x * 1000000000)

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

/* LPF conversion table */
static const unsigned int uint_bandwidths[] = {
    MHz(28),
    MHz(20),
    MHz(14),
    MHz(12),
    MHz(10),
    kHz(8750),
    MHz(7),
    MHz(6),
    kHz(5500),
    MHz(5),
    kHz(3840),
    MHz(3),
    kHz(2750),
    kHz(2500),
    kHz(1750),
    kHz(1500)
};

/* Frequency Range table */
struct freq_range {
    uint32_t    low;
    uint32_t    high;
    uint8_t     value;
};

const struct freq_range bands[] = {
    { FIELD_INIT(.low, 232500000),   FIELD_INIT(.high, 285625000),   FIELD_INIT(.value, 0x27) },
    { FIELD_INIT(.low, 285625000),   FIELD_INIT(.high, 336875000),   FIELD_INIT(.value, 0x2f) },
    { FIELD_INIT(.low, 336875000),   FIELD_INIT(.high, 405000000),   FIELD_INIT(.value, 0x37) },
    { FIELD_INIT(.low, 405000000),   FIELD_INIT(.high, 465000000),   FIELD_INIT(.value, 0x3f) },
    { FIELD_INIT(.low, 465000000),   FIELD_INIT(.high, 571250000),   FIELD_INIT(.value, 0x26) },
    { FIELD_INIT(.low, 571250000),   FIELD_INIT(.high, 673750000),   FIELD_INIT(.value, 0x2e) },
    { FIELD_INIT(.low, 673750000),   FIELD_INIT(.high, 810000000),   FIELD_INIT(.value, 0x36) },
    { FIELD_INIT(.low, 810000000),   FIELD_INIT(.high, 930000000),   FIELD_INIT(.value, 0x3e) },
    { FIELD_INIT(.low, 930000000),   FIELD_INIT(.high, 1142500000),  FIELD_INIT(.value, 0x25) },
    { FIELD_INIT(.low, 1142500000),  FIELD_INIT(.high, 1347500000),  FIELD_INIT(.value, 0x2d) },
    { FIELD_INIT(.low, 1347500000),  FIELD_INIT(.high, 1620000000),  FIELD_INIT(.value, 0x35) },
    { FIELD_INIT(.low, 1620000000),  FIELD_INIT(.high, 1860000000),  FIELD_INIT(.value, 0x3d) },
    { FIELD_INIT(.low, 1860000000u), FIELD_INIT(.high, 2285000000u), FIELD_INIT(.value, 0x24) },
    { FIELD_INIT(.low, 2285000000u), FIELD_INIT(.high, 2695000000u), FIELD_INIT(.value, 0x2c) },
    { FIELD_INIT(.low, 2695000000u), FIELD_INIT(.high, 3240000000u), FIELD_INIT(.value, 0x34) },
    { FIELD_INIT(.low, 3240000000u), FIELD_INIT(.high, 3900000000u), FIELD_INIT(.value, 0x3c) }
};


const uint8_t lms_reg_dumpset[] = {
    /* Top level configuration */
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0E, 0x0F,

    /* TX PLL Configuration */
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,

    /* RX PLL Configuration */
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,

    /* TX LPF Modules Configuration */
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,

    /* TX RF Modules Configuration */
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,

    /* RX LPF, ADC, and DAC Modules Configuration */
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,

    /* RX VGA2 Configuration */
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,

    /* RX FE Modules Configuration */
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C
};

/* When enabling an LPF, we must select both the module
 * and the filter bandwidth */
int lms_lpf_enable(struct bladerf *dev, bladerf_module mod, lms_bw bw)
{
    int status;
    uint8_t data;
    const uint8_t reg = (mod == BLADERF_MODULE_RX) ? 0x54 : 0x34;

    /* Check to see which bandwidth we have selected */
    status = bladerf_lms_read(dev, reg, &data);
    if (status != 0) {
        return status;
    }

    data |= (1 << 1);   /* Enable LPF module */
    data &= ~0x3c;      /* Clear out previous bandwidth setting */
    data |= (bw << 2);  /* Apply new bandwidth setting */

    status = bladerf_lms_write(dev, reg, data);
    if (status != 0) {
        return status;
    }

    /* Check to see if we are bypassed */
    status = bladerf_lms_read(dev, reg + 1, &data);
    if (status != 0) {
        return status;
    } else if (data & (1 << 6)) {
        /* Bypass is enabled; switch back to normal operation */
        data &= ~(1 << 6);
        status = bladerf_lms_write(dev, reg + 1, data);
    }

    return status;
}

int lms_lpf_get_mode(struct bladerf *dev, bladerf_module mod,
                     bladerf_lpf_mode *mode)
{
    int status;
    uint8_t data;
    const uint8_t reg = (mod == BLADERF_MODULE_RX) ? 0x54 : 0x34;

    status = bladerf_lms_read(dev, reg, &data);
    if (status != 0) {
        return status;
    }

    if ((data & (1 << 1) ) == 0) {
        *mode = BLADERF_LPF_DISABLED;
    } else {
        status = bladerf_lms_read(dev, reg + 1, &data);
        if (status != 0) {
            return status;
        }

        if (data & (1 << 6)) {
            *mode = BLADERF_LPF_BYPASSED;
        } else {
            *mode = BLADERF_LPF_NORMAL;
        }
    }

    return 0;
}

int lms_lpf_set_mode(struct bladerf *dev, bladerf_module mod,
                     bladerf_lpf_mode mode)
{
    int status;
    const uint8_t reg = (mod == BLADERF_MODULE_RX) ? 0x54 : 0x34;
    uint8_t data_l, data_h;

    status = bladerf_lms_read(dev, reg, &data_l);
    if (status != 0) {
        return status;
    }

    status = bladerf_lms_read(dev, reg + 1, &data_h);
    if (status != 0) {
        return status;
    }

    if (mode == BLADERF_LPF_DISABLED) {
        data_l &= ~(1 << 1);    /* Power down LPF */
    } else if (mode == BLADERF_LPF_BYPASSED) {
        data_l |= (1 << 1);     /* Enable LPF */
        data_h |= (1 << 6);     /* Enable LPF bypass */
    } else {
        data_l |= (1 << 1);     /* Enable LPF */
        data_h &= ~(1 << 6);    /* Disable LPF bypass */
    }

    status = bladerf_lms_write(dev, reg, data_l);
    if (status != 0) {
        return status;
    }

    status = bladerf_lms_write(dev, reg + 1, data_h);
    return status;
}

/* Get the bandwidth for the selected module */
int lms_get_bandwidth(struct bladerf *dev, bladerf_module mod, lms_bw *bw)
{
    int status;
    uint8_t data;
    const uint8_t reg = (mod == BLADERF_MODULE_RX) ? 0x54 : 0x34;

    status = bladerf_lms_read(dev, reg, &data);
    if (status != 0) {
        return status;
    }

    /* Fetch bandwidth table index from reg[5:2] */
    data >>= 2;
    data &= 0xf;

    assert(data < ARRAY_SIZE(uint_bandwidths));
    *bw = (lms_bw)data;
    return 0;
}

lms_bw lms_uint2bw(unsigned int req)
{
    lms_bw ret;

    if (     req <= kHz(1500)) ret = BW_1p5MHz;
    else if (req <= kHz(1750)) ret = BW_1p75MHz;
    else if (req <= kHz(2500)) ret = BW_2p5MHz;
    else if (req <= kHz(2750)) ret = BW_2p75MHz;
    else if (req <= MHz(3)  )  ret = BW_3MHz;
    else if (req <= kHz(3840)) ret = BW_3p84MHz;
    else if (req <= MHz(5)  )  ret = BW_5MHz;
    else if (req <= kHz(5500)) ret = BW_5p5MHz;
    else if (req <= MHz(6)  )  ret = BW_6MHz;
    else if (req <= MHz(7)  )  ret = BW_7MHz;
    else if (req <= kHz(8750)) ret = BW_8p75MHz;
    else if (req <= MHz(10) )  ret = BW_10MHz;
    else if (req <= MHz(12) )  ret = BW_12MHz;
    else if (req <= MHz(14) )  ret = BW_14MHz;
    else if (req <= MHz(20) )  ret = BW_20MHz;
    else                       ret = BW_28MHz;

    assert(ret < ARRAY_SIZE(uint_bandwidths));
    return ret;
}

/* Return the table entry */
unsigned int lms_bw2uint(lms_bw bw)
{
    unsigned int idx = bw & 0xf;
    assert(idx < ARRAY_SIZE(uint_bandwidths));
    return uint_bandwidths[idx];
}

/* Enable dithering on the module PLL */
int lms_dither_enable(struct bladerf *dev, bladerf_module mod,
                      uint8_t nbits, bool enable)
{
    int status;

    /* Select the base address based on which PLL we are configuring */
    const uint8_t reg = (mod == BLADERF_MODULE_RX) ? 0x24 : 0x14;
    uint8_t data;

    /* Valid range is 1 - 8 bits (inclusive) */
    if (nbits < 1 || nbits > 8) {
        return BLADERF_ERR_INVAL;
    }

    /* Read what we currently have in there */
    status = bladerf_lms_read(dev, reg, &data);
    if (status != 0) {
        return status;
    }

    if (enable) {
        /* Enable dithering */
        data |= (1 << 7);

        /* Clear out the previous setting of the number of bits to dither */
        data &= ~(7 << 4);

        /* Update with the desired number of bits to dither */
        data |= (((nbits - 1) & 7) << 4);

    } else {
        /* Clear dithering enable bit */
        data &= ~(1 << 7);
    }

    /* Write it out */
    status = bladerf_lms_write(dev, reg, data);
    return status;
}

/* Soft reset of the LMS */
int lms_soft_reset(struct bladerf *dev)
{

    int status = bladerf_lms_write(dev, 0x05, 0x12);

    if (status == 0) {
        status = bladerf_lms_write(dev, 0x05, 0x32);
    }

    return status;
}

/* Set the gain on the LNA */
int lms_lna_set_gain(struct bladerf *dev, bladerf_lna_gain gain)
{
    int status;
    uint8_t data;

    if (gain == BLADERF_LNA_GAIN_BYPASS || gain == BLADERF_LNA_GAIN_MID ||
        gain == BLADERF_LNA_GAIN_MAX) {

        status = bladerf_lms_read(dev, 0x75, &data);
        if (status == 0) {
            data &= ~(3 << 6);          /* Clear out previous gain setting */
            data |= ((gain & 3) << 6);  /* Update gain value */
            status = bladerf_lms_write(dev, 0x75, data);
        }

    } else {
        status = BLADERF_ERR_INVAL;
    }

    return status;
}

int lms_lna_get_gain(struct bladerf *dev, bladerf_lna_gain *gain)
{
    int status;
    uint8_t data;

    status = bladerf_lms_read(dev, 0x75, &data);
    if (status == 0) {
        data >>= 6;
        data &= 3;
        *gain = (bladerf_lna_gain)data;

        if (*gain == BLADERF_LNA_GAIN_UNKNOWN) {
            status = BLADERF_ERR_INVAL;
        }
    }

    return status;
}

/* Select which LNA to enable */
int lms_lna_select(struct bladerf *dev, lms_lna lna)
{
    int status;
    uint8_t data;

    status = bladerf_lms_read(dev, 0x75, &data);
    if (status == 0) {
        data &= ~(3 << 4);
        data |= ((lna & 3) << 4);
        status = bladerf_lms_write(dev, 0x75, data);
    }

    return status;
}

int lms_rxvga1_enable(struct bladerf *dev, bool enable)
{
    int status;

    if (enable) {
        /* Set bias current to nominal */
        status = bladerf_lms_write(dev, 0x7b, 0x33);
    } else {
        /* Set bias current to 0 */
        status = bladerf_lms_write(dev, 0x7b, 0x03);
    }

    return status;
}

/* Set the RFB_TIA_RXFE mixer gain */
int lms_rxvga1_set_gain(struct bladerf *dev, uint8_t gain)
{
    int status;
    uint8_t data;

    if (gain > 120) {
        log_info("%s: %d being clamped to 120\n", __FUNCTION__, gain);
        gain = 120;
    }

    status = bladerf_lms_read(dev, 0x76, &data);
    if (status == 0) {
        data &= ~(0x7f);
        data |= gain;
        status = bladerf_lms_write(dev, 0x76, gain & 0x7f);
    }

    return status;
}

/* Get the RFB_TIA_RXFE mixer gain */
int lms_rxvga1_get_gain(struct bladerf *dev, uint8_t *gain)
{
    int status;
    uint8_t data;
    status = bladerf_lms_read(dev, 0x76, &data);
    if (status == 0) {
        *gain = data & 0x7f;
    }

    return status;
}

/* Enable RXVGA2 */
int lms_rxvga2_enable(struct bladerf *dev, bool enable, uint8_t gain)
{
    int status;
    uint8_t data;

    status = bladerf_lms_read(dev, 0x64, &data);
    if (status != 0) {
        return status;
    }

    if (enable) {
        data |= (1 << 1);
        status = lms_rxvga2_set_gain(dev, gain);
    } else {
        data &= ~(1 << 1);
    }

    if (status == 0) {
        status = bladerf_lms_write(dev, 0x64, data);
    }

    return status;
}


/* Set the gain on RXVGA2 */
int lms_rxvga2_set_gain(struct bladerf *dev, uint8_t gain)
{
    int status;
    uint8_t data;

    /* NOTE: Gain is calculated as gain*3dB and shouldn't really */
    /* go above 30dB */
    if ((gain & 0x1f) > 10)
    {
        log_info("Clamping gain to 30dB\n");
        gain = 10;
    }

    status = bladerf_lms_read(dev, 0x65, &data);
    if (status == 0) {
        data &= ~(0x1f);
        data |= gain;
        status = bladerf_lms_write(dev, 0x65, data);
    }

    return status;
}

int lms_rxvga2_get_gain(struct bladerf *dev, uint8_t *gain)
{

    uint8_t data;
    const int status = bladerf_lms_read(dev, 0x65, &data);

    if (status == 0) {
        *gain = data & 0x1f;
    } else {
        *gain = 0;
    }

    return status;
}

/* Enable PA (PA_ALL is NOT valid for enabling) */
int lms_pa_enable(struct bladerf *dev, lms_pa pa, bool enable)
{
    int status;
    uint8_t data;

    status = bladerf_lms_read(dev, 0x44, &data);
    if (status != 0) {
        return status;
    }

    if (enable) {
        switch (pa) {
            case PA_AUX:
                data &= ~(1 << 1);
                break;

            case PA_1:
                data &= ~(3 << 3);
                data |= (1 << 3);
                break;

            case PA_2:
                data &= ~(3 << 3);
                data |= (2 << 3);
                break;

            default:
                return BLADERF_ERR_INVAL;
        }
    } else {
        switch (pa) {
            case PA_AUX:
                data |= (1 << 1);
                break;

            case PA_1:
                data &= ~(4 << 2);
                break;

            case PA_2:
                data &= ~(2 << 2);
                break;

            case PA_ALL:
                data |= (1 << 1);
                data &= ~(4 << 2);
                data &= ~(2 << 2);
                break;
        }
    }

    status = bladerf_lms_write(dev, 0x44, data);
    return status;
}

int lms_peakdetect_enable(struct bladerf *dev, bool enable)
{
    int status;
    uint8_t data;

    status = bladerf_lms_read(dev, 0x44, &data);

    if (status == 0) {
        if (enable) {
            data &= ~(1 << 0);
        } else {
            data |= (1 << 0);
        }
        status = bladerf_lms_write(dev, 0x44, data);
    }

    return status;
}

int lms_enable_rffe(struct bladerf *dev, bladerf_module module, bool enable)
{
    int status;
    uint8_t data;
    uint8_t base = module == BLADERF_MODULE_TX ? 0x40 : 0x70 ;

    status = bladerf_lms_read(dev, base, &data);
    if (status == 0) {
        if (enable) {
            data |= (1 << 1);
        } else {
            data &= ~(1 << 1);
        }
        status = bladerf_lms_write(dev, base, data);
    }

    return status;
}

int lms_loopback_path_enable(struct bladerf *dev, lms_lbp mode, bool enable)
{
    int status = BLADERF_ERR_INVAL;
    uint8_t data;

    if (enable) {
        switch (mode) {
            case LBP_BB:
                status = bladerf_lms_read(dev, 0x46, &data);
                if (status == 0) {
                    /* LOOPBBEN[1:0] Close base band loopback switch */
                    data |= (3 << 2);
                    status = bladerf_lms_write(dev, 0x46, data);
                }
                break;

            case LBP_RF:
                /* Disable all the PA's first */
                status = lms_pa_enable(dev, PA_ALL, false);
                if (status != 0) {
                    return status;
                }

                /* Power up the RF loopback switch (PD[0]) */
                status = bladerf_lms_read(dev, 0x0b, &data);
                if (status != 0) {
                    return status;
                }

                data |= (1 << 0);

                status = bladerf_lms_write(dev, 0x0b, data);
                if (status != 0) {
                    return status;
                }

                /* Enable the AUX PA only */
                status = lms_pa_enable(dev, PA_AUX, true);
                if (status != 0) {
                    return status;
                }

                /* Make sure we're muxed over to the AUX mux */
                status = bladerf_lms_read(dev, 0x45, &data);
                if (status == 0) {
                    data &= ~(7 << 0);
                    status =bladerf_lms_write(dev, 0x45, data);
                }
                break;

            default:
                status = BLADERF_ERR_INVAL;
        }
    } else {
        switch (mode) {
            case LBP_BB:
                status = bladerf_lms_read(dev, 0x46, &data);
                if (status == 0) {
                    /* LOOPBBEN[1:0] Open the base band loopback switch */
                    data &= ~(3 << 2);
                    status = bladerf_lms_write(dev, 0x46, data);
                }
                break;

            case LBP_RF:
                /* Disable the AUX PA */
                status = lms_pa_enable(dev, PA_AUX, false);
                if (status != 0) {
                    return status;
                }

                /* Disconnect the switch */
                status = bladerf_lms_read(dev, 0x0b, &data);
                if (status != 0) {
                    return status;
                }

                data &= ~(1 << 0);

                status = bladerf_lms_write(dev, 0x0b, data);

                if (status == 0) {
                    /* Decode control signals (as opposed to using control
                     * signals from the test mode registers).  */
                    status = bladerf_lms_write(dev, 0x70, 0);
                }
                break;
        }
    }

    return status;
}

int lms_txvga2_set_gain(struct bladerf *dev, uint8_t gain)
{
    int status;
    uint8_t data;

    if (gain > 25) {
        log_debug("%s: Clamping gain to 25 dB\n", __FUNCTION__);
        gain = 25;
    }

    status = bladerf_lms_read(dev, 0x45, &data);
    if (status == 0) {
        data &= ~(0x1f << 3);
        data |= ((gain & 0x1f) << 3);
        status = bladerf_lms_write(dev, 0x45, data);
    }

    return status;
}

int lms_txvga2_get_gain(struct bladerf *dev, uint8_t *gain)
{
    int status;
    uint8_t data;

    status = bladerf_lms_read(dev, 0x45, &data);

    if (status == 0) {
        *gain = (data >> 3) & 0x1f;
    }

    return status;
}

int lms_txvga1_set_gain(struct bladerf *dev, int8_t gain)
{
    if (gain < -35 || gain > -4) {
        return BLADERF_ERR_INVAL;
    }

    /* Apply offset to convert gain to register table index */
    gain = (gain + 35);

    /* Since 0x41 is only VGA1GAIN, we don't need to RMW */
    return bladerf_lms_write(dev, 0x41, gain & 0x1f);
}

int lms_txvga1_get_gain(struct bladerf *dev, int8_t *gain)
{
    int status;
    uint8_t data;

    status = bladerf_lms_read(dev, 0x41, &data);
    if (status == 0) {
        /* Clamp to max value */
        data = data & 0x1f;

        /* Convert table index to value */
        *gain = data - 35;
    }

    return status;
}

/* Loopback enable */
int lms_loopback_enable(struct bladerf *dev, bladerf_loopback mode)
{
    int status;
    uint8_t data;

    switch(mode)
    {
        case BLADERF_LB_BB_LPF:
            log_debug("Enabling LB BB LPF\n");

            /* Disable RXVGA1 first */
            status = lms_rxvga1_enable(dev, false);
            if (status != 0) {
                return status;
            }

            /* Enable BB TX and RX loopback */
            status = lms_loopback_path_enable(dev, LBP_BB, true);
            if (status == 0) {
                status = bladerf_lms_write(dev, 0x08, 1 << 6);
            }
            break;

        case BLADERF_LB_BB_VGA2:
            log_debug("Enabling LB BB VGA2\n");

            /* Disable RXLPF first */
            status = lms_lpf_set_mode(dev, BLADERF_MODULE_RX,
                                      BLADERF_LPF_DISABLED);
            if (status != 0) {
                return status;
            }

            /* Enable TX and RX loopback */
            status = lms_loopback_path_enable(dev, LBP_BB, true);
            if (status == 0) {
                status = bladerf_lms_write(dev, 0x08, 1 << 5);
            }
            break;

        case BLADERF_LB_BB_OP:
            log_debug("Enabling LB BB OP\n");

            /* Disable RXLPF, RXVGA2, and RXVGA1 */
            status = lms_rxvga1_enable(dev, false);
            if (status != 0) {
                return status;
            }

            status = lms_rxvga2_enable(dev, false, 0);
            if (status != 0) {
                return status;
            }

            status = lms_lpf_set_mode(dev, BLADERF_MODULE_RX,
                                      BLADERF_LPF_DISABLED);
            if (status != 0) {
                return status;
            }

            /* Enable TX and RX loopback */
            status = lms_loopback_path_enable(dev, LBP_BB, true);
            if (status == 0) {
                status = bladerf_lms_write(dev, 0x08, 1 << 4);
            }
            break;

        case BLADERF_LB_RF_LNA1:
        case BLADERF_LB_RF_LNA2:
        case BLADERF_LB_RF_LNA3:
            log_debug("Enabling LB RF LNA%d\n",
                      mode - (BLADERF_LB_RF_LNA1 - 1));

            /* Disable all LNAs */
            status = lms_lna_select(dev, LNA_NONE);
            if (status != 0) {
                return status;
            }

            /* Enable AUX PA, PD[0], and loopback */
            status = lms_loopback_path_enable(dev, LBP_RF, true);
            if (status != 0) {
                return status;
            }

            status = bladerf_lms_read(dev, 0x7d, &data);
            if (status != 0) {
                return status;
            }

            data |= 1;
            status = bladerf_lms_write(dev, 0x7d, data);
            if (status != 0) {
                return status;
            }

            /* Choose the LNA (1 = LNA1, 2 = LNA2, 3 = LNA3) */
            status = bladerf_lms_write(dev, 0x08,
                                       (mode - (BLADERF_LB_RF_LNA1 - 1)));

            /* Enable the use of control signals from test mode registers.
             * (This allows us to power down LNAs) */
            if (status == 0) {
                status = bladerf_lms_write(dev, 0x70, (1 << 1));
            }
            break;

        default:
            log_debug("Invalid LB\n");
            status = BLADERF_ERR_INVAL;
            break;
    }

    return status;
}

/* Figure out what loopback mode we're in (if any at all!) */
int lms_get_loopback_mode(struct bladerf *dev, bladerf_loopback *mode)
{
    int status;
    uint8_t data;

    status = bladerf_lms_read(dev, 0x08, &data);
    if (status != 0) {
        return status;
    }

    if (data == 0) {
        *mode = BLADERF_LB_NONE;
    } else if (data & (1 << 6)) {
        *mode = BLADERF_LB_BB_LPF;
    } else if (data & (1 << 5)) {
        *mode = BLADERF_LB_BB_VGA2;
    } else if (data & (1 << 4)) {
        *mode = BLADERF_LB_BB_OP;
    } else if ((data & 0xf) == 1) {
        *mode = BLADERF_LB_RF_LNA1;
    } else if ((data & 0xf) == 2) {
        *mode = BLADERF_LB_RF_LNA2;
    } else if ((data & 0xf) == 3) {
        *mode = BLADERF_LB_RF_LNA3;
    } else {
        *mode = BLADERF_LB_NONE;
        log_error("Unexpected LMS mode value: 0x%02x\n", data);
        status = BLADERF_ERR_UNEXPECTED;
    }

    return status;
}

/* Disable loopback mode
 * Must choose which LNA to hook up and what bandwidth you want */
int lms_loopback_disable(struct bladerf *dev, lms_lna lna, lms_bw bw)
{
    int status;
    bladerf_loopback mode;

    /* Read which type of loopback mode we were in */
    status = lms_get_loopback_mode(dev, &mode);
    if (status != 0) {
        return status;
    }

    /* Disable all RX loopback modes */
    status = bladerf_lms_write(dev, 0x08, 0);
    if (status != 0) {
        return status;
    }


    switch (mode) {
        log_debug("Disabling LB BB LPF\n");

        case BLADERF_LB_BB_LPF:
            /* Disable TX baseband loopback */
            status = lms_loopback_path_enable(dev, LBP_BB, true);
            if (status == 0) {
                /* Enable RXVGA1 */
                status = lms_rxvga1_enable(dev, true);
            }
            break;

        case BLADERF_LB_BB_VGA2:
            log_debug("Disabling LB BB VGA2\n");

            /* Disable TX baseband loopback */
            status = lms_loopback_path_enable(dev, LBP_BB, false);
            if (status == 0) {
                /* Enable RXLPF */
                lms_lpf_enable(dev, BLADERF_MODULE_RX, bw);
            }
            break;

        case BLADERF_LB_BB_OP:
            log_debug("Disabling LB BB OP\n");

            /* Disable TX baseband loopback */
            status = lms_loopback_path_enable(dev, LBP_BB, false);
            if (status != 0) {
                return status;
            }

            /* Enable RXLPF, RXVGA1 and RXVGA2 */
            status = lms_lpf_enable(dev, BLADERF_MODULE_RX, bw);
            if (status != 0) {
                return status;
            }

            /* XXX: Hardcoded gain? */
            status = lms_rxvga2_enable(dev, true, 30/3);
            if (status == 0) {
                status = lms_rxvga1_enable(dev, true);
            }
            break;

        case BLADERF_LB_RF_LNA1:
        case BLADERF_LB_RF_LNA2:
        case BLADERF_LB_RF_LNA3:
            log_debug("Disabling LB RF LNA%d\n",
                        mode - (BLADERF_LB_RF_LNA1 - 1));

            /* Disable TX RF loopback */
            status = lms_loopback_path_enable(dev, LBP_RF, false);
            if (status == 0) {
                /* Enable selected LNA */
                lms_lna_select(dev, lna);
            }
            break;

        default:
            log_debug("Invalid LB\n");
            status = BLADERF_ERR_INVAL;
            break;
    }

    return status;
}

/* Top level power down of the LMS */
int lms_power_down(struct bladerf *dev)
{
    int status;
    uint8_t data;

    status = bladerf_lms_read(dev, 0x05, &data);
    if (status == 0) {
        data &= ~(1 << 4);
        status = bladerf_lms_write(dev, 0x05, data);
    }

    return status;
}

/* Enable the PLL of a module */
int lms_pll_enable(struct bladerf *dev, bladerf_module mod, bool enable)
{
    int status;
    const uint8_t reg = (mod == BLADERF_MODULE_RX) ? 0x24 : 0x14;
    uint8_t data;

    status = bladerf_lms_read(dev, reg, &data);
    if (status == 0) {
        if (enable) {
            data |= (1 << 3);
        } else {
            data &= ~(1 << 3);
        }
        status = bladerf_lms_write(dev, reg, data);
    }

    return status;
}

/* Enable the RX subsystem */
int lms_rx_enable(struct bladerf *dev, bool enable)
{
    int status;
    uint8_t data;

    status = bladerf_lms_read(dev, 0x05, &data);
    if (status == 0) {
        if (enable) {
            data |= (1 << 2);
        } else {
            data &= ~(1 << 2);
        }
        status = bladerf_lms_write(dev, 0x05, data);
    }

    return status;
}

/* Enable the TX subsystem */
int lms_tx_enable(struct bladerf *dev, bool enable)
{
    int status;
    uint8_t data;

    status = bladerf_lms_read(dev, 0x05, &data);

    if (status == 0) {
        if (enable) {
            data |= (1 << 3);
        } else {
            data &= ~(1 << 3);
        }
        status = bladerf_lms_write(dev, 0x05, data);
    }

    return status;
}

/* Converts frequency structure to Hz */
uint32_t lms_frequency_to_hz(struct lms_freq *f)
{
    uint64_t pll_coeff;
    uint32_t div;

    pll_coeff = (((uint64_t)f->nint) << 23) + f->nfrac;
    div = (f->x << 23);

    return (uint32_t)(((f->reference * pll_coeff) + (div >> 1)) / div);
}

/* Print a frequency structure */
void lms_print_frequency(struct lms_freq *f)
{
    log_debug("  x        : %d\n", f->x);
    log_debug("  nint     : %d\n", f->nint);
    log_debug("  nfrac    : %u\n", f->nfrac);
    log_debug("  freqsel  : %x\n", f->freqsel);
    log_debug("  reference: %u\n", f->reference);
    log_debug("  freq     : %u\n", lms_frequency_to_hz(f));
}

/* Get the frequency structure */
int lms_get_frequency(struct bladerf *dev, bladerf_module mod,
                      struct lms_freq *f)
{
    const uint8_t base = (mod == BLADERF_MODULE_RX) ? 0x20 : 0x10;
    int status;
    uint8_t data;

    status = bladerf_lms_read(dev, base + 0, &data);
    if (status != 0) {
        return status;
    }

    f->nint = ((uint16_t)data) << 1;

    status = bladerf_lms_read(dev, base + 1, &data);
    if (status != 0) {
        return status;
    }

    f->nint |= (data & 0x80) >> 7;
    f->nfrac = ((uint32_t)data & 0x7f) << 16;

    status = bladerf_lms_read(dev, base + 2, &data);
    if (status != 0) {
        return status;
    }

    f->nfrac |= ((uint32_t)data)<<8;

    status = bladerf_lms_read(dev, base + 3, &data);
    if (status != 0) {
        return status;
    }

    f->nfrac |= data;

    status = bladerf_lms_read(dev, base + 5, &data);
    if (status != 0) {
        return status;
    }

    f->freqsel = (data>>2);
    f->x = 1 << ((f->freqsel & 7) - 3);
    f->reference = 38400000;

    return status;
}

#define VCO_HIGH 0x02
#define VCO_NORM 0x00
#define VCO_LOW 0x01
static inline int tune_vcocap(struct bladerf *dev, uint8_t base, uint8_t data)
{
    int start_i = -1, stop_i = -1;
    int i;
    uint8_t vcocap = 32;
    uint8_t step = vcocap >> 1;
    uint8_t vtune;
    int status;

    status = bladerf_lms_read(dev, base + 9, &data);
    if (status != 0) {
        return status;
    }

    data &= ~(0x3f);
    for (i = 0; i < 6; i++) {
        status = bladerf_lms_write(dev, base + 9, vcocap | data);
        if (status != 0) {
            return status;
        }

        status = bladerf_lms_read(dev, base + 10, &vtune);
        if (status != 0) {
            return status;
        }

        vtune >>= 6;

        if (vtune == VCO_NORM) {
            log_verbose( "Found normal at VCOCAP: %d\n", vcocap );
            break;
        } else if (vtune == VCO_HIGH) {
            log_verbose( "Too high: %d -> %d\n", vcocap, vcocap + step );
            vcocap += step ;
        } else if (vtune == VCO_LOW) {
            log_verbose( "Too low: %d -> %d\n", vcocap, vcocap - step );
            vcocap -= step ;
        } else {
            log_error( "Invalid VTUNE value encountered\n" );
            return BLADERF_ERR_UNEXPECTED;
        }

        step >>= 1;
    }

    if (vtune != VCO_NORM) {
        log_debug( "VTUNE is not locked at the end of initial loop\n" );
        return BLADERF_ERR_UNEXPECTED;
    }

    start_i = stop_i = vcocap;
    while (start_i > 0 && vtune == VCO_NORM) {
        start_i -= 1;

        status = bladerf_lms_write(dev, base + 9, start_i | data);
        if (status != 0) {
            return status;
        }

        status = bladerf_lms_read(dev, base + 10, &vtune);
        if (status != 0) {
            return status;
        }

        vtune >>= 6;
    }

    start_i += 1;
    log_verbose( "Found lower limit VCOCAP: %d\n", start_i );

    status = bladerf_lms_write(dev, base + 9, vcocap | data );
    if (status != 0) {
        return status;
    }

    status = bladerf_lms_read(dev, base + 10, &vtune);
    if (status != 0) {
        return status;
    }

    vtune >>= 6;

    while (stop_i < 64 && vtune == VCO_NORM) {
        stop_i += 1;

        status = bladerf_lms_write(dev, base + 9, stop_i | data);
        if (status != 0) {
            return status;
        }

        status = bladerf_lms_read(dev, base + 10, &vtune);
        if (status != 0) {
            return status;
        }

        vtune >>= 6;
    }

    stop_i -= 1;
    log_verbose( "Found upper limit VCOCAP: %d\n", stop_i );

    vcocap = (start_i + stop_i) >> 1 ;

    log_verbose( "Goldilocks VCOCAP: %d\n", vcocap );

    status = bladerf_lms_write(dev, base + 9, vcocap | data );
    if (status != 0) {
        return status;
    }

    status = bladerf_lms_read(dev, base + 10, &vtune);
    if (status != 0) {
        return status;
    }

    vtune >>= 6;
    log_verbose( "VTUNE: %d\n", vtune );
    if (vtune != VCO_NORM) {
        status = BLADERF_ERR_UNEXPECTED;
        log_warning("VCOCAP could not converge and VTUNE is not locked - %d\n",
                    vtune);
    }

    return status;
}

/* Set the frequency of a module */
int lms_set_frequency(struct bladerf *dev, bladerf_module mod, uint32_t freq)
{
    /* Select the base address based on which PLL we are configuring */
    const uint8_t base = (mod == BLADERF_MODULE_RX) ? 0x20 : 0x10;
    const uint32_t lfreq = freq;
    const uint64_t ref_clock = 38400000;
    uint8_t freqsel = bands[0].value;
    uint16_t nint;
    uint32_t nfrac;
    struct lms_freq f;
    uint8_t data;
    uint64_t vco_x;
    uint64_t temp;
    int status, dsm_status;

    /* Figure out freqsel */
    if (lfreq < bands[0].low) {
        log_debug("Frequency too low: %u\n", freq);
        return BLADERF_ERR_INVAL;
    } else if (lfreq > bands[15].high) {
        log_debug("Frequency too high: %u\n", freq);
        return BLADERF_ERR_INVAL;
    } else {
        uint8_t i = 0;
        while(i < 16) {
            if ((lfreq > bands[i].low) && (lfreq <= bands[i].high)) {
                freqsel = bands[i].value;
                break;
            }
            i++;
        }
    }

    vco_x = ((uint64_t)1) << ((freqsel & 7) - 3);
    temp = (vco_x * freq) / ref_clock;
    assert(temp <= UINT16_MAX);
    nint = (uint16_t)temp;
    temp = ((1 << 23) * (vco_x * freq - nint * ref_clock)) / ref_clock;
    assert(temp <= UINT32_MAX);
    nfrac = (uint32_t)temp;

    assert(vco_x <= UINT8_MAX);
    f.x = (uint8_t)vco_x;
    f.nint = nint;
    f.nfrac = nfrac;
    f.freqsel = freqsel;
    assert(ref_clock <= UINT32_MAX);
    f.reference = (uint32_t)ref_clock;
    lms_print_frequency(&f);

    /* Turn on the DSMs */
    status = bladerf_lms_read(dev, 0x09, &data);
    if (status == 0) {
        data |= 0x05;
        status = bladerf_lms_write(dev, 0x09, data);
    }

    if (status != 0) {
        log_debug("Failed to turn on DSMs\n");
        return status;
    }

    /* Program freqsel, selout (rx only), nint and nfrac */
    if (mod == BLADERF_MODULE_RX) {
        status = bladerf_lms_write(dev, base + 5,
                                   freqsel << 2 | (freq < 1500000000 ? 1 : 2));
    } else {
        status = bladerf_lms_write(dev, base + 5, freqsel << 2);
    }

    if (status != 0) {
        goto lms_set_frequency_error;
    }

    data = nint >> 1;
    status = bladerf_lms_write(dev, base + 0, data);
    if (status != 0) {
        goto lms_set_frequency_error;
    }


    data = ((nint & 1) << 7) | ((nfrac >> 16) & 0x7f);
    status = bladerf_lms_write(dev, base + 1, data);
    if (status != 0) {
        goto lms_set_frequency_error;
    }

    data = ((nfrac >> 8) & 0xff);
    status = bladerf_lms_write(dev, base + 2, data);
    if (status != 0) {
        goto lms_set_frequency_error;
    }

    data = (nfrac & 0xff);
    status = bladerf_lms_write(dev, base + 3, data);
    if (status != 0) {
        goto lms_set_frequency_error;
    }

    /* Set the PLL Ichp, Iup and Idn currents */
    status = bladerf_lms_read(dev, base + 6, &data);
    if (status != 0) {
        goto lms_set_frequency_error;
    }

    data &= ~(0x1f);
    data |= 0x0c;

    status = bladerf_lms_write(dev, base + 6, data);
    if (status != 0) {
        goto lms_set_frequency_error;
    }

    status = bladerf_lms_read(dev, base + 7, &data);
    if (status != 0) {
        goto lms_set_frequency_error;
    }

    data &= ~(0x1f);
    data |= 3;

    status = bladerf_lms_write(dev, base + 7, data);
    if (status != 0) {
        goto lms_set_frequency_error;
    }

    status = bladerf_lms_read(dev, base + 8, &data);
    if (status != 0) {
        goto lms_set_frequency_error;
    }

    data &= ~(0x1f);
    status = bladerf_lms_write(dev, base + 8, data);
    if (status != 0) {
        goto lms_set_frequency_error;
    }

    /* Loop through the VCOCAP to figure out optimal values */
    status = tune_vcocap(dev, base, data);

lms_set_frequency_error:
    /* Turn off the DSMs */
    dsm_status = bladerf_lms_read(dev, 0x09, &data);
    if (dsm_status == 0) {
        data &= ~(0x05);
        dsm_status = bladerf_lms_write(dev, 0x09, data);
    }

    return (status == 0) ? dsm_status : status;
}

int lms_dump_registers(struct bladerf *dev)
{
    int status = 0;
    uint8_t data,i;
    const uint16_t num_reg = sizeof(lms_reg_dumpset);

    for (i = 0; i < num_reg; i++) {
        status = bladerf_lms_read(dev, lms_reg_dumpset[i], &data);
        if (status != 0) {
            log_debug("Failed to read LMS @ 0x%02x\n", lms_reg_dumpset[i]);
            return status;
        } else {
            log_debug("addr: %x data: %x\n", lms_reg_dumpset[i], data);
        }
    }

    return status;
}


int lms_config_init(struct bladerf *dev, struct lms_xcvr_config *config)
{
    int status;

    status = lms_soft_reset(dev);
    if (status != 0) {
        log_debug("Failed to perform LMS soft reset.\n");
        return status;
    }

    /* BYPVCOREG[7]     - VCO regulater bypassed
     * PDVCOREG[6]      - VCO regulator powered down
     * FSTVCOBG[5]      - Switch open
     * OFFUP[4:0]       - 30 uA
     */
    status = bladerf_lms_write(dev, 0x17, 0xC3);
    if (status != 0) {
        return status;
    }

    status = bladerf_lms_write(dev, 0x27, 0xC3);
    if (status != 0) {
        return status;
    }


    /* RX LPF Configuration:
     *  ADC/DAC modules enabled
     *  Decode ADC/DAC enable signals
     *  DAC Internal Output Load Resister = 100 Ohms
     *  DAC reference current resistor: Internal
     */
    status = bladerf_lms_write(dev, 0x57, 0x84);
    if (status != 0) {
        return status;
    }

    status = lms_tx_enable(dev, true);
    if (status != 0) {
        log_debug("Failed to enable LMS TX \n");
        return status;
    }

    status = lms_rx_enable(dev, true);
    if (status != 0) {
        log_debug("Failed to enable LMS RX \n");
        return status;
    }

    /* Set default value for mixer bias current */
    status = bladerf_lms_write(dev, 0x48, 20);
    if (status != 0) {
        return status;
    }

    /* Set default value for the bias current of the PAs */
    status = bladerf_lms_write(dev, 0x49, 20);
    if (status != 0) {
        return status;
    }

    status = lms_set_frequency(dev, BLADERF_MODULE_RX, config->rx_freq_hz);
    if (status != 0) {
        return status;
    }

    status = lms_set_frequency(dev, BLADERF_MODULE_TX, config->tx_freq_hz);
    if (status != 0) {
        return status;
    }

    status = lms_lna_select(dev, config->lna);
    if (status != 0) {
        return status;
    }

    lms_pa_enable(dev, config->pa, true);
    if (status != 0) {
        return status;
    }

    if (config->loopback_mode == BLADERF_LB_NONE) {
        status = lms_loopback_disable(dev, config->lna, config->tx_bw);
    } else {
        status = lms_loopback_enable(dev, config->loopback_mode);
    }

    return status;
}

/* Reference LMS6002D calibration guide, section 4.1 flow chart */
static int lms_dc_cal_loop(struct bladerf *dev, uint8_t base,
                           uint8_t cal_address, uint8_t *dc_regval)
{
    int status;
    uint8_t i, val, control;
    bool done = false;
    const unsigned int max_cal_count = 25;

    log_debug("Calibrating module %2.2x:%2.2x\n", base, cal_address);

    /* Set the calibration address for the block, and start it up */
    status = bladerf_lms_read(dev, base + 0x03, &val);
    if (status != 0) {
        return status;
    }

    val &= ~(0x07);
    val |= cal_address&0x07;

    status = bladerf_lms_write(dev, base + 0x03, val);
    if (status != 0) {
        return status;
    }

    /* Start the calibration by toggling DC_START_CLBR */
    val |= (1 << 5);
    status = bladerf_lms_write(dev, base + 0x03, val);
    if (status != 0) {
        return status;
    }

    val &= ~(1 << 5);
    status = bladerf_lms_write(dev, base + 0x03, val);
    if (status != 0) {
        return status;
    }

    control = val;

    /* Main loop checking the calibration */
    for (i = 0 ; i < max_cal_count; i++) {
        /* Read active low DC_CLBR_DONE */
        status = bladerf_lms_read(dev, base + 0x01, &val);
        if (status != 0) {
            return status;
        }

        if (((val >> 1) & 1) == 0) {
            /* We think we're done, but we need to check DC_LOCK */
            if (((val >> 2) & 7) != 0 && ((val >> 2) & 7) != 7) {
                log_debug("Converged in %d iterations for %2x:%2x\n", i + 1,
                          base, cal_address );
                done = true;
                break;
            } else {
                log_debug( "DC_CLBR_DONE but no DC_LOCK - rekicking\n" );

                control |= (1 << 5);
                status = bladerf_lms_write(dev, base + 0x03, control);
                if (status != 0) {
                    return status;
                }

                control &= ~(1 << 5);
                status =bladerf_lms_write(dev, base + 0x03, control);
                if (status != 0) {
                    return status;
                }
            }
        }
    }

    if (done == false) {
        log_warning("Never converged - DC_CLBR_DONE: %d DC_LOCK: %d\n",
                    (val >> 1) & 1, (val >> 2) & 7);
        status = BLADERF_ERR_UNEXPECTED;
    } else {
        /* See what the DC register value is and return it to the caller */
        status = bladerf_lms_read(dev, base, dc_regval);
        if (status == 0) {
            *dc_regval &= 0x3f;
            log_debug( "DC_REGVAL: %d\n", *dc_regval );
        }
    }

    return status;
}

int lms_calibrate_dc(struct bladerf *dev, bladerf_cal_module module)
{
    int status;

    /* Working variables */
    uint8_t cal_clock, base, addrs, i, val, dc_regval;

    /* Saved values that are to be restored */
    uint8_t clockenables, reg0x71, reg0x7c;
    bladerf_lna_gain lna_gain;
    int rxvga1, rxvga2;

    /* Save off the top level clock enables */
    status = bladerf_lms_read(dev, 0x09, &clockenables);
    if (status != 0) {
        return status;
    }

    val = clockenables;
    cal_clock = 0 ;
    switch (module) {
        case BLADERF_DC_CAL_LPF_TUNING:
            cal_clock = (1 << 5);  /* CLK_EN[5] - LPF CAL Clock */
            base = 0x00;
            addrs = 1;
            break;

        case BLADERF_DC_CAL_TX_LPF:
            cal_clock = (1 << 1);  /* CLK_EN[1] - TX LPF DCCAL Clock */
            base = 0x30;
            addrs = 2;
            break;

        case BLADERF_DC_CAL_RX_LPF:
            cal_clock = (1 << 3);  /* CLK_EN[3] - RX LPF DCCAL Clock */
            base = 0x50;
            addrs = 2;
            break;

        case BLADERF_DC_CAL_RXVGA2:
            cal_clock = (1 << 4);  /* CLK_EN[4] - RX VGA2 DCCAL Clock */
            base = 0x60;
            addrs = 5;
            break;

        default:
            return BLADERF_ERR_INVAL;
    }

    /* Enable the appropriate clock based on the module */
    status = bladerf_lms_write(dev, 0x09, clockenables | cal_clock);
    if (status != 0) {
        return status;
    }

    /* Special case for RX LPF or RX VGA2 */
    if (module == BLADERF_DC_CAL_RX_LPF || module == BLADERF_DC_CAL_RXVGA2) {

        /* Connect LNA to the external pads and interally terminate */
        status = bladerf_lms_read(dev, 0x71, &reg0x71);
        if (status != 0) {
            return status;
        }

        val = reg0x71;
        val &= ~(1 << 7);

        status = bladerf_lms_write(dev, 0x71, val);
        if (status != 0) {
            return status;
        }

        status = bladerf_lms_read(dev, 0x7c, &reg0x7c);
        if (status != 0) {
            return status;
        }

        val = reg0x7c;
        val |= (1 << 2);

        status = bladerf_lms_write(dev, 0x7c, val);
        if (status != 0) {
            return status;
        }

        /* Set maximum gain for everything, but save off current values */
        status = bladerf_get_lna_gain(dev, &lna_gain);
        if (status != 0) {
            return status;
        }

        status = bladerf_set_lna_gain(dev, BLADERF_LNA_GAIN_MAX);
        if (status != 0) {
            return status;
        }

        status = bladerf_get_rxvga1(dev, &rxvga1);
        if (status != 0) {
            return status;
        }

        status = bladerf_set_rxvga1(dev, 30);
        if (status != 0) {
            return status;
        }

        status = bladerf_get_rxvga2(dev, &rxvga2);
        if (status != 0) {
            return status;
        }

       status = bladerf_set_rxvga2(dev, 30);
       if (status != 0) {
           return status;
       }
    }

    /* Figure out number of addresses to calibrate based on module */
    for (i = 0; i < addrs ; i++) {
        status = lms_dc_cal_loop(dev, base, i, &dc_regval) ;
        if (status != 0) {
            return status;
        }
    }

    /* Special case for LPF tuning module where results are
     * written to TX/RX LPF DCCAL */
    if (module == BLADERF_DC_CAL_LPF_TUNING) {

        /* Set the DC level to RX and TX DCCAL modules */
        status = bladerf_lms_read(dev, 0x35, &val);
        if (status == 0) {
            val &= ~(0x3f);
            val |= dc_regval;
            status = bladerf_lms_write(dev, 0x35, val);
        }

        if (status != 0) {
            return status;
        }

        status = bladerf_lms_read(dev, 0x55, &val);
        if (status == 0) {
            val &= ~(0x3f);
            val |= dc_regval;
            status = bladerf_lms_write(dev, 0x55, val);
        }

        if (status != 0) {
            return status;
        }

    /* Special case for RX LPF or RX VGA2 */
    } else if (module == BLADERF_DC_CAL_RX_LPF ||
               module == BLADERF_DC_CAL_RXVGA2) {

        /* Restore previously saved LNA Gain, VGA1 gain and VGA2 gain */
        status = bladerf_set_rxvga2(dev, rxvga2);
        if (status != 0) {
            return status;
        }

        status = bladerf_set_rxvga1(dev, rxvga1);
        if (status != 0) {
            return status;
        }

        status = bladerf_set_lna_gain(dev, lna_gain);
        if (status != 0) {
            return status;
        }

        status = bladerf_lms_write(dev, 0x71, reg0x71);
        if (status != 0) {
            return status;
        }

        status = bladerf_lms_write(dev, 0x7c, reg0x7c);
        if (status != 0) {
            return status;
        }
    }

    /* Restore original clock enables */
    status = bladerf_lms_write(dev, 0x09, clockenables);
    return status;
}
