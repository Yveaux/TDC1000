#include "TDC1000.h"
#include <SPI.h>

#define TDC1000_SPI_CLK_MAX                               (int32_t(20000000))
#define TDC1000_SPI_MODE                                  (SPI_MODE0)
#define TDC1000_SPI_ORDER                                 (MSBFIRST)
#define TDC1000_SPI_REG_ADDR_MASK                         (0x1Fu)
#define TDC1000_SPI_REG_READ                              (0x00u)
#define TDC1000_SPI_REG_WRITE                             (0x40u)


#define TDC1000_REG_ADR_CONFIG_0                          (0x00u)
#define TDC1000_REG_ADR_CONFIG_1                          (0x01u)
#define TDC1000_REG_ADR_CONFIG_2                          (0x02u)
#define TDC1000_REG_ADR_CONFIG_3                          (0x03u)
#define TDC1000_REG_ADR_CONFIG_4                          (0x04u)
#define TDC1000_REG_ADR_TOF_1                             (0x05u)
#define TDC1000_REG_ADR_TOF_0                             (0x06u)
#define TDC1000_REG_ADR_ERROR_FLAGS                       (0x07u)
#define TDC1000_REG_ADR_TIMEOUT                           (0x08u)
#define TDC1000_REG_ADR_CLOCK_RATE                        (0x09u)

#define TDC1000_REG_SHIFT_CONFIG_0_TX_FREQ_DIV            (5)
#define TDC1000_REG_MASK_CONFIG_0_TX_FREQ_DIV             (0b111u)
#define TDC7200_REG_VAL_CONFIG_0_TX_FREQ_DIV(num)         (1 << (num+1))
#define TDC1000_REG_SHIFT_CONFIG_0_NUM_TX                 (0)
#define TDC7200_REG_VAL_CONFIG_0_NUM_TX(num)              (num)
#define TDC1000_REG_VAL_CONFIG_0_NUM_TX_MIN               (0)
#define TDC1000_REG_VAL_CONFIG_0_NUM_TX_MAX               (31)
#define TDC1000_REG_MASK_CONFIG_0_NUM_TX                  (0b11111u)
#define TDC1000_REG_DEFAULTS_CONFIG0                      (0x45u)


#define TDC1000_REG_SHIFT_CONFIG_1_NUM_AVG                (3)
#define TDC1000_REG_MASK_CONFIG_1_NUM_AVG                 (0b111u)
#define TDC7200_REG_VAL_CONFIG_1_NUM_AVG(num)             (1 << (num))
#define TDC1000_REG_SHIFT_CONFIG_1_NUM_RX                 (0)
#define TDC1000_REG_MASK_CONFIG_1_NUM_RX                  (0b111u)
#define TDC7200_REG_VAL_CONFIG_1_NUM_RX(num)              (num)
#define TDC1000_REG_VAL_CONFIG_1_NUM_RX_MIN               (0)
#define TDC1000_REG_VAL_CONFIG_1_NUM_RX_MAX               (7)
#define TDC1000_REG_DEFAULTS_CONFIG1                      (0x40u)


#define TDC1000_REG_SHIFT_CONFIG_2_VCOM_SEL               (7)
#define TDC1000_REG_MASK_CONFIG_2_VCOM_SEL                (0b1u)
#define TDC1000_REG_SHIFT_CONFIG_2_MEAS_MODE              (6)
#define TDC1000_REG_MASK_CONFIG_2_MEAS_MODE               (0b1u)
#define TDC1000_REG_VAL_CONFIG_2_MEAS_MODE_TOF            (0)
#define TDC1000_REG_VAL_CONFIG_2_MEAS_MODE_TEMP           (1)
#define TDC1000_REG_SHIFT_CONFIG_2_DAMPING                (5)
#define TDC1000_REG_MASK_CONFIG_2_DAMPING                 (0b1u)
#define TDC1000_REG_SHIFT_CONFIG_2_CH_SWP                 (4)
#define TDC1000_REG_MASK_CONFIG_2_CH_SWP                  (0b1u)
#define TDC1000_REG_SHIFT_CONFIG_2_EXT_CHSEL              (3)
#define TDC1000_REG_MASK_CONFIG_2_EXT_CHSEL               (0b1u)
#define TDC1000_REG_SHIFT_CONFIG_2_CH_SEL                 (2)
#define TDC1000_REG_MASK_CONFIG_2_CH_SEL                  (0b1u)
#define TDC1000_REG_VAL_CONFIG_2_CH_SEL(num)              ((num)+1)
#define TDC1000_REG_SHIFT_CONFIG_2_TOF_MEAS_MODE          (0)
#define TDC1000_REG_MASK_CONFIG_2_TOF_MEAS_MODE           (0x03u)
#define TDC1000_REG_VAL_CONFIG_2_TOF_MEAS_MODE_MIN        (0)
#define TDC1000_REG_VAL_CONFIG_2_TOF_MEAS_MODE_MAX        (2)
#define TDC7200_REG_VAL_CONFIG_2_TOF_MEAS_MODE(num)       (num)


#define TDC1000_REG_SHIFT_CONFIG_3_TEMP_MODE              (6)
#define TDC1000_REG_MASK_CONFIG_3_TEMP_MODE               (0b1u)
#define TDC1000_REG_SHIFT_CONFIG_3_TEMP_RTD_SEL           (5)
#define TDC1000_REG_MASK_CONFIG_3_TEMP_RTD_SEL            (0b1u)
#define TDC1000_REG_SHIFT_CONFIG_3_TEMP_CLK_DIV           (4)
#define TDC1000_REG_MASK_CONFIG_3_TEMP_CLK_DIV            (0b1u)
#define TDC1000_REG_SHIFT_CONFIG_3_BLANKING               (3)
#define TDC1000_REG_MASK_CONFIG_3_BLANKING                (0b1u)
#define TDC1000_REG_SHIFT_CONFIG_3_ECHO_QUAL_THLD         (0)
#define TDC1000_REG_MASK_CONFIG_3_ECHO_QUAL_THLD          (0b111u)
#define TDC1000_REG_VAL_CONFIG_3_ECHO_QUAL_THLD_MIN       (0)
#define TDC1000_REG_VAL_CONFIG_3_ECHO_QUAL_THLD_MAX       (7)


#define TDC1000_REG_SHIFT_CONFIG_4_RECEIVE_MODE           (6)
#define TDC1000_REG_MASK_CONFIG_4_RECEIVE_MODE            (0b1u)
#define TDC1000_REG_SHIFT_CONFIG_4_TRIG_EDGE_POLARITY     (5)
#define TDC1000_REG_MASK_CONFIG_4_TRIG_EDGE_POLARITY      (0b1u)
#define TDC1000_REG_SHIFT_CONFIG_4_TX_PH_SHIFT_POS        (0)
#define TDC1000_REG_MASK_CONFIG_4_TX_PH_SHIFT_POS         (0b11111u)
#define TDC1000_REG_VAL_CONFIG_4_TX_PH_SHIFT_POS(num)     (num)
#define TDC1000_REG_VAL_CONFIG_4_TX_PH_SHIFT_POS_MIN      (0)
#define TDC1000_REG_VAL_CONFIG_4_TX_PH_SHIFT_POS_MAX      (31)


#define TDC1000_REG_SHIFT_TOF_1_PGA_GAIN                  (5)
#define TDC1000_REG_MASK_TOF_1_PGA_GAIN                   (0b111u)
#define TDC1000_REG_VAL_TOF_1_PGA_GAIN(num)               (3*(num))
#define TDC1000_REG_SHIFT_TOF_1_PGA_CTRL                  (4)
#define TDC1000_REG_MASK_TOF_1_PGA_CTRL                   (0b1u)
#define TDC1000_REG_VAL_TOF_1_PGA_CTRL_ACTIVE             (0u)
#define TDC1000_REG_VAL_TOF_1_PGA_CTRL_BYPASSED           (1u)
#define TDC1000_REG_SHIFT_TOF_1_LNA_CTRL                  (3)
#define TDC1000_REG_MASK_TOF_1_LNA_CTRL                   (0b1u)
#define TDC1000_REG_VAL_TOF_1_LNA_CTRL_ACTIVE             (0u)
#define TDC1000_REG_VAL_TOF_1_LNA_CTRL_BYPASSED           (1u)
#define TDC1000_REG_SHIFT_TOF_1_LNA_FB                    (2)
#define TDC1000_REG_MASK_TOF_1_LNA_FB                     (0b1u)
#define TDC1000_REG_SHIFT_TOF_1_TIMING_REG                (0)
#define TDC1000_REG_MASK_TOF_1_TIMING_REG                 (0b11u)
#define TDC1000_REG_VAL_TOF_1_PGA_GAIN_MIN                (0)
#define TDC1000_REG_VAL_TOF_1_PGA_GAIN_MAX                (7)


#define TDC1000_REG_SHIFT_TOF_0_TIMING_REG                (0)
#define TDC1000_REG_MASK_TOF_0_TIMING_REG                 (0b11111111u)
#define TDC1000_REG_VAL_TOF_TIMING_REG_MIN                (0u)
#define TDC1000_REG_VAL_TOF_TIMING_REG_FORCE_SHORT_MAX    (29u)
#define TDC1000_REG_VAL_TOF_TIMING_REG_MAX                (1023u)
#define TDC1000_REG_SHIFT_ERROR_FLAGS_ERR_SIG_WEAK        (2)
#define TDC1000_REG_MASK_ERROR_FLAGS_ERR_SIG_WEAK         (0b1u)
#define TDC1000_REG_SHIFT_ERROR_FLAGS_ERR_NO_SIG          (1)
#define TDC1000_REG_MASK_ERROR_FLAGS_ERR_NO_SIG           (0b1u)
#define TDC1000_REG_SHIFT_ERROR_FLAGS_ERR_SIG_HIGH        (0)
#define TDC1000_REG_MASK_ERROR_FLAGS_ERR_SIG_HIGH         (0b1u)


#define TDC1000_REG_SHIFT_TIMEOUT_FORCE_SHORT_TOF         (6)
#define TDC1000_REG_MASK_TIMEOUT_FORCE_SHORT_TOF          (0b1u)
#define TDC1000_REG_VAL_TIMEOUT_FORCE_SHORT_TOF_DISABLED  (0u)
#define TDC1000_REG_VAL_TIMEOUT_FORCE_SHORT_TOF_ENABLED   (1u)
#define TDC1000_REG_SHIFT_TIMEOUT_SHORT_TOF_BLANK_PERIOD  (3)
#define TDC1000_REG_MASK_TIMEOUT_SHORT_TOF_BLANK_PERIOD   (0b111u)
#define TDC1000_REG_VAL_TIMEOUT_SHORT_TOF_BLANK_PERIOD(num)  (1u << ((num) + 3))
#define TDC1000_REG_SHIFT_TIMEOUT_ECHO_TIMEOUT            (2)
#define TDC1000_REG_MASK_TIMEOUT_ECHO_TIMEOUT             (0b1u)
#define TDC1000_REG_VAL_TIMEOUT_ECHO_TIMEOUT_ENABLED      (0u)
#define TDC1000_REG_VAL_TIMEOUT_ECHO_TIMEOUT_DISABLED     (1u)
#define TDC1000_REG_SHIFT_TIMEOUT_TOF_TIMEOUT_CTRL        (0)
#define TDC1000_REG_MASK_TIMEOUT_TOF_TIMEOUT_CTRL         (0b11u)
#define TDC1000_REG_VAL_TIMEOUT_TOF_TIMEOUT_CTRL(num)     (1u << ((num) + 7))


#define TDC1000_REG_SHIFT_CLOCKIN_DIV                     (2)
#define TDC1000_REG_MASK_CLOCKIN_DIV                      (0b1u)
#define TDC1000_REG_VAL_CLOCKIN_DIV(num)                  ((num)+1)
#define TDC1000_REG_SHIFT_AUTOZERO_PERIOD                 (0)
#define TDC1000_REG_MASK_AUTOZERO_PERIOD                  (0b11u)
#define TDC1000_REG_VAL_AUTOZERO_PERIOD(num)              (1u << ((num) + 6))

#define ARRAY_SIZE(x)   ((sizeof(x))/sizeof(x[0]))

TDC1000::TDC1000(const uint8_t pinCs, const uint8_t pinReset)
    :   m_pinCs(pinCs),
        m_pinReset(pinReset)
{
}

bool TDC1000::begin()
{
    // -- Configure SPI
    digitalWrite(m_pinCs, HIGH);
    pinMode(m_pinCs, OUTPUT);
    SPI.begin();

    // -- Reset TDC1000
    pinMode(m_pinReset, OUTPUT);
    digitalWrite(m_pinReset, HIGH);
    delay(10);
    digitalWrite(m_pinReset, LOW);
    delay(10);

    // -- Comms sanity check
    if (   (spiReadReg8(TDC1000_REG_ADR_CONFIG_0) != TDC1000_REG_DEFAULTS_CONFIG0)
        or (spiReadReg8(TDC1000_REG_ADR_CONFIG_1) != TDC1000_REG_DEFAULTS_CONFIG1) )
    {
        return false;
    }

    return true;
}

bool TDC1000::setTriggerEdge(const bool rising)
{
    spiRmwReg8( TDC1000_REG_ADR_CONFIG_4,
         TDC1000_REG_MASK_CONFIG_4_TRIG_EDGE_POLARITY << TDC1000_REG_SHIFT_CONFIG_4_TRIG_EDGE_POLARITY,
         (rising ? 0x00: 0x01) << TDC1000_REG_SHIFT_CONFIG_4_TRIG_EDGE_POLARITY );

    return true;
}

bool TDC1000::setTx(const TxFreqDivider div, const uint8_t pulses, const uint8_t shift,
                    const bool damping)
{
    if (pulses > TDC1000_REG_VAL_CONFIG_0_NUM_TX_MAX) return false;
    if (shift > TDC1000_REG_VAL_CONFIG_4_TX_PH_SHIFT_POS_MAX) return false;

    uint8_t val = spiReadReg8(TDC1000_REG_ADR_CONFIG_0);
    // Clear bits we are going to modify.
    val &= ~(   (TDC1000_REG_MASK_CONFIG_0_TX_FREQ_DIV << TDC1000_REG_SHIFT_CONFIG_0_TX_FREQ_DIV)
              | (TDC1000_REG_MASK_CONFIG_0_NUM_TX      << TDC1000_REG_SHIFT_CONFIG_0_NUM_TX) );
    
    val |= uint8_t(div) << TDC1000_REG_SHIFT_CONFIG_0_TX_FREQ_DIV;
    val |= pulses << TDC1000_REG_SHIFT_CONFIG_0_NUM_TX;
    spiWriteReg8(TDC1000_REG_ADR_CONFIG_0, val);

    spiRmwReg8( TDC1000_REG_ADR_CONFIG_4,
         TDC1000_REG_MASK_CONFIG_4_TX_PH_SHIFT_POS << TDC1000_REG_SHIFT_CONFIG_4_TX_PH_SHIFT_POS,
         shift << TDC1000_REG_SHIFT_CONFIG_4_TX_PH_SHIFT_POS );

    val = spiReadReg8(TDC1000_REG_ADR_CONFIG_2);
    // Clear bits we are going to modify.
    val &= ~( (TDC1000_REG_MASK_CONFIG_2_DAMPING << TDC1000_REG_SHIFT_CONFIG_2_DAMPING) );
    if (damping) val |= 1u << TDC1000_REG_SHIFT_CONFIG_2_DAMPING;
    spiWriteReg8(TDC1000_REG_ADR_CONFIG_2, val);

    return true;
}

bool TDC1000::setRx(const bool multiEcho)
{
    spiRmwReg8( TDC1000_REG_ADR_CONFIG_4,
         TDC1000_REG_MASK_CONFIG_4_RECEIVE_MODE << TDC1000_REG_SHIFT_CONFIG_4_RECEIVE_MODE,
         (multiEcho ? 1u : 0u) << TDC1000_REG_SHIFT_CONFIG_4_RECEIVE_MODE );

    return true;
}

bool TDC1000::setRxSensitivity(const RxDacEchoThreshold dacTh, const RxPgaGain pgaGain, const RxLnaFbMode lnaFb  )
{
    spiRmwReg8( TDC1000_REG_ADR_CONFIG_3,
         TDC1000_REG_MASK_CONFIG_3_ECHO_QUAL_THLD << TDC1000_REG_SHIFT_CONFIG_3_ECHO_QUAL_THLD,
         uint8_t(dacTh) << TDC1000_REG_SHIFT_CONFIG_3_ECHO_QUAL_THLD );
    
    uint8_t val = spiReadReg8(TDC1000_REG_ADR_TOF_1);
    // Clear bits we are going to modify.
    val &= ~(   (TDC1000_REG_MASK_TOF_1_PGA_GAIN << TDC1000_REG_SHIFT_TOF_1_PGA_GAIN)
              | (TDC1000_REG_MASK_TOF_1_PGA_CTRL << TDC1000_REG_SHIFT_TOF_1_PGA_CTRL)
              | (TDC1000_REG_MASK_TOF_1_LNA_CTRL << TDC1000_REG_SHIFT_TOF_1_LNA_CTRL)
              | (TDC1000_REG_MASK_TOF_1_LNA_FB   << TDC1000_REG_SHIFT_TOF_1_LNA_FB) );

    if (RxPgaGain::disabled == pgaGain)
    {
        val |= TDC1000_REG_VAL_TOF_1_PGA_CTRL_BYPASSED << TDC1000_REG_SHIFT_TOF_1_PGA_CTRL;
    }
    else
    {
        val |= TDC1000_REG_VAL_TOF_1_PGA_CTRL_ACTIVE << TDC1000_REG_SHIFT_TOF_1_PGA_CTRL;
        val |= uint8_t(pgaGain) << TDC1000_REG_SHIFT_TOF_1_PGA_GAIN;
    }
    if (RxLnaFbMode::disabled == lnaFb)
    {
        val |= TDC1000_REG_VAL_TOF_1_LNA_CTRL_BYPASSED << TDC1000_REG_SHIFT_TOF_1_LNA_CTRL;
    }
    else
    {
        val |= TDC1000_REG_VAL_TOF_1_LNA_CTRL_ACTIVE << TDC1000_REG_SHIFT_TOF_1_LNA_CTRL;
        val |= uint8_t(lnaFb) << TDC1000_REG_SHIFT_TOF_1_LNA_FB;
    }
    spiWriteReg8(TDC1000_REG_ADR_TOF_1, val);

    return true;
}

bool TDC1000::setRepeat(const TxRxCycles cycles, const uint8_t expectedEvents)
{
    if (expectedEvents > TDC1000_REG_VAL_CONFIG_1_NUM_RX_MAX) return false;

    uint8_t val = spiReadReg8(TDC1000_REG_ADR_CONFIG_1);
    // Clear bits we are going to modify.
    val &= ~(   (TDC1000_REG_MASK_CONFIG_1_NUM_AVG << TDC1000_REG_SHIFT_CONFIG_1_NUM_AVG)
              | (TDC1000_REG_MASK_CONFIG_1_NUM_RX << TDC1000_REG_SHIFT_CONFIG_1_NUM_RX) );

    val |= uint8_t(cycles) << TDC1000_REG_SHIFT_CONFIG_1_NUM_AVG;
    val |= expectedEvents << TDC1000_REG_SHIFT_CONFIG_1_NUM_RX;
    spiWriteReg8(TDC1000_REG_ADR_CONFIG_1, val);
 
    return true;
} 

bool TDC1000::setMeasureTOF(const TxRxChannel chanSel, const TofMode mode)
{
    uint8_t val = spiReadReg8(TDC1000_REG_ADR_CONFIG_2);
    // Clear bits we are going to modify.
    val &= ~(   (TDC1000_REG_MASK_CONFIG_2_CH_SWP        << TDC1000_REG_SHIFT_CONFIG_2_CH_SWP)
              | (TDC1000_REG_MASK_CONFIG_2_EXT_CHSEL     << TDC1000_REG_SHIFT_CONFIG_2_EXT_CHSEL)
              | (TDC1000_REG_MASK_CONFIG_2_CH_SEL        << TDC1000_REG_SHIFT_CONFIG_2_CH_SEL)
              | (TDC1000_REG_MASK_CONFIG_2_MEAS_MODE     << TDC1000_REG_SHIFT_CONFIG_2_MEAS_MODE)
              | (TDC1000_REG_MASK_CONFIG_2_TOF_MEAS_MODE << TDC1000_REG_SHIFT_CONFIG_2_TOF_MEAS_MODE) );

    if (TxRxChannel::External == chanSel)      val |= 1u << TDC1000_REG_SHIFT_CONFIG_2_EXT_CHSEL;
    else if (TxRxChannel::Swap == chanSel)     val |= 1u << TDC1000_REG_SHIFT_CONFIG_2_CH_SWP;
    else if (TxRxChannel::Channel2 == chanSel) val |= 1u << TDC1000_REG_SHIFT_CONFIG_2_CH_SEL;
    val |= uint8_t(mode) << TDC1000_REG_SHIFT_CONFIG_2_TOF_MEAS_MODE;
    val |= TDC1000_REG_VAL_CONFIG_2_MEAS_MODE_TOF << TDC1000_REG_SHIFT_CONFIG_2_MEAS_MODE;
    spiWriteReg8(TDC1000_REG_ADR_CONFIG_2, val);
    return true;
}

bool TDC1000::setMeasureTemp(const TempMode mode, const TempRtdSel rtd, const TempClkDiv div)
{
    spiRmwReg8( TDC1000_REG_ADR_CONFIG_2,
         TDC1000_REG_MASK_CONFIG_2_MEAS_MODE << TDC1000_REG_SHIFT_CONFIG_2_MEAS_MODE,
         TDC1000_REG_VAL_CONFIG_2_MEAS_MODE_TEMP << TDC1000_REG_SHIFT_CONFIG_2_MEAS_MODE );

    uint8_t val = spiReadReg8(TDC1000_REG_ADR_CONFIG_3);
    // Clear bits we are going to modify.
    val &= ~(   (TDC1000_REG_MASK_CONFIG_3_TEMP_MODE    << TDC1000_REG_SHIFT_CONFIG_3_TEMP_MODE)
              | (TDC1000_REG_MASK_CONFIG_3_TEMP_RTD_SEL << TDC1000_REG_SHIFT_CONFIG_3_TEMP_RTD_SEL)
              | (TDC1000_REG_MASK_CONFIG_3_TEMP_CLK_DIV << TDC1000_REG_SHIFT_CONFIG_3_TEMP_CLK_DIV) );

    val |= uint8_t(mode) << TDC1000_REG_SHIFT_CONFIG_3_TEMP_MODE;
    val |= uint8_t(rtd) << TDC1000_REG_SHIFT_CONFIG_3_TEMP_RTD_SEL;
    val |= uint8_t(div) << TDC1000_REG_SHIFT_CONFIG_3_TEMP_CLK_DIV;
    spiWriteReg8(TDC1000_REG_ADR_CONFIG_3, val);
    return true;
}

bool TDC1000::setTofMeasuementShort(const T0 clkDiv, const TxAutoZeroPeriod autoZero, const TxBlankPeriod blank,
                                    const TxEchoTimeoutPeriod echoTimeout)
{
    uint8_t val = spiReadReg8(TDC1000_REG_ADR_CLOCK_RATE);
    // Clear bits we are going to modify.
    val &= ~(   (TDC1000_REG_MASK_CLOCKIN_DIV     << TDC1000_REG_SHIFT_CLOCKIN_DIV)
              | (TDC1000_REG_MASK_AUTOZERO_PERIOD << TDC1000_REG_SHIFT_AUTOZERO_PERIOD) );
    val |= uint8_t(clkDiv)   << TDC1000_REG_SHIFT_CLOCKIN_DIV;
    val |= uint8_t(autoZero) << TDC1000_REG_SHIFT_AUTOZERO_PERIOD;
    spiWriteReg8(TDC1000_REG_ADR_CLOCK_RATE, val);

    val = spiReadReg8(TDC1000_REG_ADR_TIMEOUT);
    val &= ~(   (TDC1000_REG_MASK_TIMEOUT_FORCE_SHORT_TOF        << TDC1000_REG_SHIFT_TIMEOUT_FORCE_SHORT_TOF)
              | (TDC1000_REG_MASK_TIMEOUT_SHORT_TOF_BLANK_PERIOD << TDC1000_REG_SHIFT_TIMEOUT_SHORT_TOF_BLANK_PERIOD)
              | (TDC1000_REG_MASK_TIMEOUT_ECHO_TIMEOUT           << TDC1000_REG_SHIFT_TIMEOUT_ECHO_TIMEOUT)
              | (TDC1000_REG_MASK_TIMEOUT_TOF_TIMEOUT_CTRL       << TDC1000_REG_SHIFT_TIMEOUT_TOF_TIMEOUT_CTRL) );
    val |= TDC1000_REG_VAL_TIMEOUT_FORCE_SHORT_TOF_ENABLED << TDC1000_REG_SHIFT_TIMEOUT_FORCE_SHORT_TOF;
    val |= uint8_t(blank) << TDC1000_REG_SHIFT_TIMEOUT_SHORT_TOF_BLANK_PERIOD;
    if (TxEchoTimeoutPeriod::disabled == echoTimeout)
    {
        val |= TDC1000_REG_VAL_TIMEOUT_ECHO_TIMEOUT_DISABLED << TDC1000_REG_SHIFT_TIMEOUT_ECHO_TIMEOUT;
    }
    else
    {
        val |= TDC1000_REG_VAL_TIMEOUT_ECHO_TIMEOUT_ENABLED << TDC1000_REG_SHIFT_TIMEOUT_ECHO_TIMEOUT;
        val |= uint8_t(echoTimeout) << TDC1000_REG_SHIFT_TIMEOUT_TOF_TIMEOUT_CTRL;
    }
    spiWriteReg8(TDC1000_REG_ADR_TIMEOUT, val);
    return true;
}

bool TDC1000::setTofMeasuementStandard(const T0 clkDiv, const TxAutoZeroPeriod autoZero, const TxEchoTimeoutPeriod echoTimeout,
                                       const uint16_t timingReg, const bool blanking)
{
    if (timingReg <= TDC1000_REG_VAL_TOF_TIMING_REG_FORCE_SHORT_MAX) return false;  // Implicit short TOF mode, so wrong value.
    if (timingReg > TDC1000_REG_VAL_TOF_TIMING_REG_MAX) return false;

    uint8_t val = spiReadReg8(TDC1000_REG_ADR_CLOCK_RATE);
    // Clear bits we are going to modify.
    val &= ~(   (TDC1000_REG_MASK_CLOCKIN_DIV     << TDC1000_REG_SHIFT_CLOCKIN_DIV)
              | (TDC1000_REG_MASK_AUTOZERO_PERIOD << TDC1000_REG_SHIFT_AUTOZERO_PERIOD) );
    val |= uint8_t(clkDiv)   << TDC1000_REG_SHIFT_CLOCKIN_DIV;
    val |= uint8_t(autoZero) << TDC1000_REG_SHIFT_AUTOZERO_PERIOD;
    spiWriteReg8(TDC1000_REG_ADR_CLOCK_RATE, val);

    val = spiReadReg8(TDC1000_REG_ADR_TIMEOUT);
    val &= ~(   (TDC1000_REG_MASK_TIMEOUT_FORCE_SHORT_TOF        << TDC1000_REG_SHIFT_TIMEOUT_FORCE_SHORT_TOF)
              | (TDC1000_REG_MASK_TIMEOUT_ECHO_TIMEOUT           << TDC1000_REG_SHIFT_TIMEOUT_ECHO_TIMEOUT)
              | (TDC1000_REG_MASK_TIMEOUT_TOF_TIMEOUT_CTRL       << TDC1000_REG_SHIFT_TIMEOUT_TOF_TIMEOUT_CTRL) );
    val |= TDC1000_REG_VAL_TIMEOUT_FORCE_SHORT_TOF_DISABLED << TDC1000_REG_SHIFT_TIMEOUT_FORCE_SHORT_TOF;
    if (TxEchoTimeoutPeriod::disabled == echoTimeout)
    {
        val |= TDC1000_REG_VAL_TIMEOUT_ECHO_TIMEOUT_DISABLED << TDC1000_REG_SHIFT_TIMEOUT_ECHO_TIMEOUT;
    }
    else
    {
        val |= TDC1000_REG_VAL_TIMEOUT_ECHO_TIMEOUT_ENABLED << TDC1000_REG_SHIFT_TIMEOUT_ECHO_TIMEOUT;
        val |= uint8_t(echoTimeout) << TDC1000_REG_SHIFT_TIMEOUT_TOF_TIMEOUT_CTRL;
    }
    spiWriteReg8(TDC1000_REG_ADR_TIMEOUT, val);

    val = spiReadReg8(TDC1000_REG_ADR_TOF_1);
    // Clear bits we are going to modify.
    val &= ~(   (TDC1000_REG_MASK_TOF_1_TIMING_REG << TDC1000_REG_SHIFT_TOF_1_TIMING_REG) );
    val |= ((timingReg >> 8) & TDC1000_REG_MASK_TOF_1_TIMING_REG) << TDC1000_REG_SHIFT_TOF_1_TIMING_REG;
    spiWriteReg8(TDC1000_REG_ADR_TOF_1, val);

    val = (timingReg & TDC1000_REG_MASK_TOF_0_TIMING_REG) << TDC1000_REG_SHIFT_TOF_0_TIMING_REG;
    spiWriteReg8(TDC1000_REG_ADR_TOF_0, val);

    spiRmwReg8( TDC1000_REG_ADR_CONFIG_3,
         TDC1000_REG_MASK_CONFIG_3_BLANKING << TDC1000_REG_SHIFT_CONFIG_3_BLANKING,
         (blanking ? 1u : 0u) << TDC1000_REG_SHIFT_CONFIG_3_BLANKING );

    return true;
}

void TDC1000::getErrorFlags(bool &sigWeak, bool &noSig, bool &sigHigh)
{
    uint8_t val = spiReadReg8(TDC1000_REG_ADR_ERROR_FLAGS);
    sigWeak = val & (1u << TDC1000_REG_SHIFT_ERROR_FLAGS_ERR_SIG_WEAK);
    noSig   = val & (1u << TDC1000_REG_SHIFT_ERROR_FLAGS_ERR_NO_SIG);
    sigHigh = val & (1u << TDC1000_REG_SHIFT_ERROR_FLAGS_ERR_SIG_HIGH);
}

void TDC1000::clearErrorFlags()
{
    // Writing a 1 to ERR_SIG_HIGH will reset all the error flags and reset the ERRB pin to high.
    spiWriteReg8(TDC1000_REG_ADR_ERROR_FLAGS, 1u << TDC1000_REG_SHIFT_ERROR_FLAGS_ERR_SIG_HIGH);
}

void TDC1000::resetStatemachine()
{
    // Writing a 1 to this field resets the state machine, halts active measurements and returns
    // the device to the SLEEP or READY mode and resets the average counter and automatic channel
    // selection in measurement Mode 2.
    spiWriteReg8(TDC1000_REG_ADR_ERROR_FLAGS, 1u << TDC1000_REG_SHIFT_ERROR_FLAGS_ERR_NO_SIG);
}

/*
bool TDC1000::TEST()
{
    // Configure for resistive feedback!!! by setting the LNA_FB bit in the TOF_1 register to 1
 
    uint8_t config;
    config  = 0 << TDC1000_REG_SHIFT_CONFIG_0_TX_FREQ_DIV;   // 0 = divide by 2
    config |= 6 << TDC1000_REG_SHIFT_CONFIG_0_NUM_TX;        // 5 = 5 TX pulses
    spiWriteReg8(TDC1000_REG_ADR_CONFIG_0, config);
    
    // config  = 0 << TDC1000_REG_SHIFT_CONFIG_1_NUM_AVG;     // 0 = 1 measurement cycle
    // config |= 0 << TDC1000_REG_SHIFT_CONFIG_1_NUM_RX;      // 0 = do not count events
    // spiWriteReg8(TDC1000_REG_ADR_CONFIG_1, config);

    config  = 0 << TDC1000_REG_SHIFT_CONFIG_2_VCOM_SEL;      // 0 = internal voltage reference
    config |= 0 << TDC1000_REG_SHIFT_CONFIG_2_MEAS_MODE;     // 0 = TOF measurement
    config |= 1 << TDC1000_REG_SHIFT_CONFIG_2_DAMPING;       // 0 = disable damping
    config |= 0 << TDC1000_REG_SHIFT_CONFIG_2_CH_SWP;        // 0 = disable auto channel swap
    config |= 0 << TDC1000_REG_SHIFT_CONFIG_2_EXT_CHSEL;     // 0 = disable external channel swap
    config |= 0 << TDC1000_REG_SHIFT_CONFIG_2_CH_SEL;        // 0 = channel 1
    config |= 2 << TDC1000_REG_SHIFT_CONFIG_2_TOF_MEAS_MODE; // 2 = mode 2
    spiWriteReg8(TDC1000_REG_ADR_CONFIG_2, config);

    // *   The mode of the chip selects which transducers transmit and which 
    // *    transducer receives the signal.
    // *   Mode 0 uses each transducer as both sender and receiver, and maps the 
    // *    transmit1 channel with receive2, and transmit2 with receive1. In this 
    // *    mode, Channel selection is what tells the chip which transducer channel
    // *    to use: ChSel = 0, transmit1; ChSel = 1, transmit2.  In this mode,
    // *    the chip is used primarily for fluid level measurement.
    // *   Mode 1 also uses each transducer as both sender and receiver.  This 
    // *    mode varies with mode 0 by mapping the transmit1 channel with the 
    // *    receive1 channel, and transmit2 with receive2.  Otherwise it works 
    // *    almost exactly like mode 0.
    // *   Mode 2 is different in that it is for fluid flow measurement.  The 
    // *    channels are mapped in the same way as mode 1 (TX1-RX1 & TX2-RX2), but
    // *    when a signal is transmitted on one channel, the chip will be listening
    // *    for the echo on the other channel. It can be thought of as sending 
    // *    signals back-and-forth between the two transducers.  Mode 2 allows for
    // *    automatic channel swapping: as soon as the signal goes one way, the
    // *    chip can automatically send it through the second channel in the other
    // *    direction.  For automatic swapping, EXT_CHSEL needs to be off.
  

    config  = 0 << TDC1000_REG_SHIFT_CONFIG_3_TEMP_MODE;
    config |= 0 << TDC1000_REG_SHIFT_CONFIG_3_TEMP_RTD_SEL;
    config |= 0 << TDC1000_REG_SHIFT_CONFIG_3_TEMP_CLK_DIV;
    config |= 0 << TDC1000_REG_SHIFT_CONFIG_3_BLANKING;         // 0 = disable power blanking
//    config |= 3 << TDC1000_REG_SHIFT_CONFIG_3_ECHO_QUAL_THLD;   // 3 = -125mV Echo qualification DAC threshold level with respect to VCOM
    config |= 1 << TDC1000_REG_SHIFT_CONFIG_3_ECHO_QUAL_THLD;   // 3 = -125mV Echo qualification DAC threshold level with respect to VCOM
    spiWriteReg8(TDC1000_REG_ADR_CONFIG_3, config);

    config  = 0 << TDC1000_REG_SHIFT_CONFIG_4_RECEIVE_MODE;         // 0 = single echo
    config |= 0 << TDC1000_REG_SHIFT_CONFIG_4_TRIG_EDGE_POLARITY;   // 0 = rising
    config |= 31 << TDC1000_REG_SHIFT_CONFIG_4_TX_PH_SHIFT_POS;      // 2..31 for pulse, 31 = default
    spiWriteReg8(TDC1000_REG_ADR_CONFIG_4, config);


//    config  = 0 << TDC1000_REG_SHIFT_TOF_1_PGA_GAIN;            // 0 = 0dB gain
    config  = 7 << TDC1000_REG_SHIFT_TOF_1_PGA_GAIN;            // 7 = 21dB gain
    config |= 0 << TDC1000_REG_SHIFT_TOF_1_PGA_CTRL;            // 0 = active
    config |= 0 << TDC1000_REG_SHIFT_TOF_1_LNA_CTRL;            // 0 = active
    config |= 1 << TDC1000_REG_SHIFT_TOF_1_LNA_FB;              // 1 = resistive
    config |= 0 << TDC1000_REG_SHIFT_TOF_1_TIMING_REG;
    spiWriteReg8(TDC1000_REG_ADR_TOF_1, config);

    config  = 0 << TDC1000_REG_SHIFT_TOF_0_TIMING_REG;
    spiWriteReg8(TDC1000_REG_ADR_TOF_0, config);

    config  = 1 << TDC1000_REG_SHIFT_TIMEOUT_FORCE_SHORT_TOF;
    config |= 3 << TDC1000_REG_SHIFT_TIMEOUT_SHORT_TOF_BLANK_PERIOD;
    config |= 0 << TDC1000_REG_SHIFT_TIMEOUT_ECHO_TIMEOUT;
    config |= 1 << TDC1000_REG_SHIFT_TIMEOUT_TOF_TIMEOUT_CTRL;
    spiWriteReg8(TDC1000_REG_ADR_TIMEOUT, config);

    config  = 0 << TDC1000_REG_SHIFT_CLOCKIN_DIV;
    config |= 0 << TDC1000_REG_SHIFT_AUTOZERO_PERIOD;
    spiWriteReg8(TDC1000_REG_ADR_CLOCK_RATE, config);

    return true;     
}
*/

void TDC1000::dumpSettings(const uint32_t freqClkInHz)
{
    uint8_t reg, v;
    uint16_t v16;
    reg = spiReadReg8(TDC1000_REG_ADR_CLOCK_RATE);
    v = (reg >> TDC1000_REG_SHIFT_CLOCKIN_DIV) & TDC1000_REG_MASK_CLOCKIN_DIV;
    // T0 = 1 / CLKIN/2^CLKIN_DIV, scaled by 1000000 for [us]
    const float T0_us = 1000000.0 / (float(freqClkInHz) / float(TDC1000_REG_VAL_CLOCKIN_DIV(v)));

    reg = spiReadReg8(TDC1000_REG_ADR_CONFIG_0);
    v = (reg >> TDC1000_REG_SHIFT_CONFIG_0_TX_FREQ_DIV) & TDC1000_REG_MASK_CONFIG_0_TX_FREQ_DIV;
    // T1 = 1 / CLKIN/2^(TX_FREQ_DIV+1), scaled by 1000000 for [us]
    const float f1 = float(freqClkInHz) / float(TDC7200_REG_VAL_CONFIG_0_TX_FREQ_DIV(v));
    const float T1_us = 1000000.0 / f1;
    Serial.print(F("T0:\t\t\t\t")); Serial.print(T0_us); Serial.println(F(" us"));
    Serial.print(F("T1:\t\t\t\t")); Serial.print(T1_us); Serial.println(F(" us"));
    Serial.print(F("TX_CLOCK:\t\t\t")); Serial.print(f1); Serial.println(F(" Hz"));

    Serial.println(F("CONFIG_0"));
        Serial.print(F("\tRAW:\t\t\t0x")); Serial.println(reg, HEX);
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_0_TX_FREQ_DIV) & TDC1000_REG_MASK_CONFIG_0_TX_FREQ_DIV;
        Serial.print(F("\tTX_FREQ_DIV:\t\t")); Serial.println(TDC7200_REG_VAL_CONFIG_0_TX_FREQ_DIV(v));
        v = TDC7200_REG_VAL_CONFIG_0_NUM_TX(reg >> TDC1000_REG_SHIFT_CONFIG_0_NUM_TX) & TDC1000_REG_MASK_CONFIG_0_NUM_TX;
        const uint8_t numTx = v;
        Serial.print(F("\tNUM_TX:\t\t\t")); Serial.println(v);

    reg = spiReadReg8(TDC1000_REG_ADR_CONFIG_1);
    Serial.println(F("CONFIG_1"));
        Serial.print(F("\tRAW:\t\t\t0x")); Serial.println(reg, HEX);
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_1_NUM_AVG) & TDC1000_REG_MASK_CONFIG_1_NUM_AVG;
        Serial.print(F("\tNUM_AVG:\t\t")); Serial.println(TDC7200_REG_VAL_CONFIG_1_NUM_AVG(v));
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_1_NUM_RX) & TDC1000_REG_MASK_CONFIG_1_NUM_RX;
        Serial.print(F("\tNUM_RX:\t\t\t")); Serial.println(TDC7200_REG_VAL_CONFIG_1_NUM_RX(v));

    reg = spiReadReg8(TDC1000_REG_ADR_CONFIG_2);
    Serial.println(F("CONFIG_2"));
        Serial.print(F("\tRAW:\t\t\t0x")); Serial.println(reg, HEX);
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_2_VCOM_SEL) & 1u;
        Serial.print(F("\tVCOM_SEL:\t\t")); Serial.println(v ? F("EXT") : F("INT"));
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_2_MEAS_MODE) & 1u;
        Serial.print(F("\tMEAS_MODE:\t\t")); Serial.println(v ? F("TEMP") : F("TOF"));
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_2_DAMPING) & 1u;
        Serial.print(F("\tDAMPING:\t\t")); Serial.println(v ? F("ON") : F("OFF"));
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_2_CH_SWP) & 1u;
        Serial.print(F("\tCH_SWP:\t\t\t")); Serial.println(v ? F("ON") : F("OFF"));
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_2_EXT_CHSEL) & 1u;
        Serial.print(F("\tEXT_CHSEL:\t\t")); Serial.println(v ? F("ON") : F("OFF"));
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_2_CH_SEL) & 1u;
        Serial.print(F("\tCH_SEL:\t\t\t")); Serial.println(TDC1000_REG_VAL_CONFIG_2_CH_SEL(v));
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_2_TOF_MEAS_MODE) & TDC1000_REG_MASK_CONFIG_2_TOF_MEAS_MODE;
        Serial.print(F("\tTOF_MEAS_MODE:\t\t")); Serial.println(TDC7200_REG_VAL_CONFIG_2_TOF_MEAS_MODE(v));

    reg = spiReadReg8(TDC1000_REG_ADR_CONFIG_3);
    Serial.println(F("CONFIG_3"));
        Serial.print(F("\tRAW:\t\t\t0x")); Serial.println(reg, HEX);
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_3_TEMP_MODE) & 1u;
        Serial.print(F("\tTEMP_MODE:\t\t")); Serial.println(v ? F("REF, RTD1") : F("REF, RTD1, RTD2"));
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_3_TEMP_RTD_SEL) & 1u;
        Serial.print(F("\tTEMP_RTD_SEL:\t\t")); Serial.println(v ? F("PT500") : F("PT1000"));
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_3_TEMP_CLK_DIV) & 1u;
        Serial.print(F("\tTEMP_CLK_DIV:\t\t")); Serial.println(v ? F("TX_FREQ_DIV") : F("8"));
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_3_BLANKING) & 1u;
        const bool blanking = v;
        Serial.print(F("\tBLANKING:\t\t")); Serial.println(v ? F("ON") : F("OFF"));
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_3_ECHO_QUAL_THLD) & TDC1000_REG_MASK_CONFIG_3_ECHO_QUAL_THLD;
        int16_t th;
        switch(v)
        {
            case 0:  th = -35;   break;
            case 1:  th = -50;   break;
            case 2:  th = -75;   break;
            case 3:  th = -125;  break;
            case 4:  th = -220;  break;
            case 5:  th = -410;  break;
            case 6:  th = -775;  break;
            default: th = -1500; break;
        }
        Serial.print(F("\tECHO_QUAL_THLD:\t\t")); Serial.print(th); Serial.println(F(" mV"));

    reg = spiReadReg8(TDC1000_REG_ADR_CONFIG_4);
    Serial.println(F("CONFIG_4"));
        Serial.print(F("\tRAW:\t\t\t0x")); Serial.println(reg, HEX);
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_4_RECEIVE_MODE) & 1u;
        Serial.print(F("\tRECEIVE_MODE:\t\t")); Serial.println(v ? F("MULTI ECHO") : F("SINGLE ECHO"));
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_4_TRIG_EDGE_POLARITY) & 1u;
        Serial.print(F("\tTRIG_EDGE_POLARITY:\t")); Serial.println(v ? F("FALLING") : F("RISING"));
        v = (reg >> TDC1000_REG_SHIFT_CONFIG_4_TX_PH_SHIFT_POS) & TDC1000_REG_MASK_CONFIG_4_TX_PH_SHIFT_POS;
        Serial.print(F("\tTX_PH_SHIFT_POS:\t")); Serial.println(TDC1000_REG_VAL_CONFIG_4_TX_PH_SHIFT_POS(v));

    reg = spiReadReg8(TDC1000_REG_ADR_TOF_1);
    Serial.println(F("TOF_1"));
        Serial.print(F("\tRAW:\t\t\t0x")); Serial.println(reg, HEX);
        v = (reg >> TDC1000_REG_SHIFT_TOF_1_PGA_GAIN) & TDC1000_REG_MASK_TOF_1_PGA_GAIN;
        Serial.print(F("\tPGA_GAIN:\t\t")); Serial.print(TDC1000_REG_VAL_TOF_1_PGA_GAIN(v)); Serial.println(F(" dB"));
        v = (reg >> TDC1000_REG_SHIFT_TOF_1_PGA_CTRL) & 1u;
        Serial.print(F("\tPGA_CTRL:\t\t")); Serial.println(v ? F("OFF") : F("ON"));
        v = (reg >> TDC1000_REG_SHIFT_TOF_1_LNA_CTRL) & 1u;
        Serial.print(F("\tLNA_CTRL:\t\t")); Serial.println(v ? F("OFF") : F("ON"));
        v = (reg >> TDC1000_REG_SHIFT_TOF_1_LNA_FB) & 1u;
        Serial.print(F("\tLNA_FB:\t\t\t")); Serial.println(v ? F("RESISTIVE") : F("CAPACITIVE"));
        v = (reg >> TDC1000_REG_SHIFT_TOF_1_TIMING_REG) & TDC1000_REG_MASK_TOF_1_TIMING_REG;
        Serial.print(F("\tTIMING_REG[9:8]:\t")); Serial.println(v);
        v16 = v << 8;

    reg = spiReadReg8(TDC1000_REG_ADR_TOF_0);
    Serial.println(F("TOF_0"));
        Serial.print(F("\tRAW:\t\t\t0x")); Serial.println(reg, HEX);
        v16 |= reg;
        Serial.print(F("\tTIMING_REG[9:0]:\t")); Serial.println(v16);
        bool shortTOF = v16 < 30;
        const float tWait_us = (float(v16) - 30.0) * 8.0 * T0_us;

    reg = spiReadReg8(TDC1000_REG_ADR_ERROR_FLAGS);
    Serial.println(F("ERROR_FLAGS"));
        Serial.print(F("\tRAW:\t\t\t0x")); Serial.println(reg, HEX);
        v = (reg >> TDC1000_REG_SHIFT_ERROR_FLAGS_ERR_SIG_WEAK) & 1u;
        Serial.print(F("\tERR_SIG_WEAK:\t\t")); Serial.println(v);
        v = (reg >> TDC1000_REG_SHIFT_ERROR_FLAGS_ERR_NO_SIG) & 1u;
        Serial.print(F("\tERR_NO_SIG:\t\t")); Serial.println(v);
        v = (reg >> TDC1000_REG_SHIFT_ERROR_FLAGS_ERR_SIG_HIGH) & 1u;
        Serial.print(F("\tERR_SIG_HIGH:\t\t")); Serial.println(v);

    reg = spiReadReg8(TDC1000_REG_ADR_TIMEOUT);
    Serial.println(F("TIMEOUT"));
        Serial.print(F("\tRAW:\t\t\t0x")); Serial.println(reg, HEX);
        v = (reg >> TDC1000_REG_SHIFT_TIMEOUT_FORCE_SHORT_TOF) & 1u;
        shortTOF |= v;
        Serial.print(F("\tFORCE_SHORT_TOF:\t")); Serial.println(v ? F("ON") : F("OFF"));
        v16 = TDC1000_REG_VAL_TIMEOUT_SHORT_TOF_BLANK_PERIOD( (reg >> TDC1000_REG_SHIFT_TIMEOUT_SHORT_TOF_BLANK_PERIOD) & TDC1000_REG_MASK_TIMEOUT_SHORT_TOF_BLANK_PERIOD );
        const float tMask_us = float(v16) * T0_us;
        Serial.print(F("\tSHORT_TOF_BLANK_PERIOD:\t")); Serial.print(v16); Serial.print(F(" x T0 -> ")); Serial.print(tMask_us); Serial.println(F(" us"));
        v = (reg >> TDC1000_REG_SHIFT_TIMEOUT_ECHO_TIMEOUT) & 1u;
        Serial.print(F("\tECHO_TIMEOUT:\t\t")); Serial.println(v ? F("OFF") : F("ON"));
        v16 = TDC1000_REG_VAL_TIMEOUT_TOF_TIMEOUT_CTRL( (reg >> TDC1000_REG_SHIFT_TIMEOUT_TOF_TIMEOUT_CTRL) & TDC1000_REG_MASK_TIMEOUT_TOF_TIMEOUT_CTRL );
        const float tEchoListen_us = float(v16) * T0_us;
        Serial.print(F("\tTOF_TIMEOUT_CTRL:\t")); Serial.print(v16); Serial.print(F(" x T0 -> ")); Serial.print(tEchoListen_us); Serial.println(F(" us"));

    reg = spiReadReg8(TDC1000_REG_ADR_CLOCK_RATE);
    Serial.println(F("CLOCK_RATE"));
        Serial.print(F("\tRAW:\t\t\t0x")); Serial.println(reg, HEX);
        v = TDC1000_REG_VAL_CLOCKIN_DIV( (reg >> TDC1000_REG_SHIFT_CLOCKIN_DIV) & 1u);
        Serial.print(F("\tCLOCKIN_DIV:\t\t")); Serial.println(v);
        v16 = TDC1000_REG_VAL_AUTOZERO_PERIOD( (reg >> TDC1000_REG_SHIFT_AUTOZERO_PERIOD) & 1u);
        const float tAutozero_us = float(v16) * T0_us;
        Serial.print(F("\tAUTOZERO_PERIOD:\t")); Serial.print(v16); Serial.print(F(" x T0 -> ")); Serial.print(tAutozero_us); Serial.println(F(" us"));

    Serial.print(F("TOF Control:\t\t\t"));
    const float tCommonMode_us = 128 * T0_us;
    const float tTransmit_us = float(numTx) * T1_us;
    if (shortTOF)
    {
        Serial.println(F("Short TOF"));
    }
    else
    {
        Serial.println(F("Standard TOF"));
        if (blanking)
        {
            Serial.println(F(" with blanking"));
        }
    }
    if (not shortTOF) { Serial.print(F("Transmit:\t\t\t")); Serial.print(tTransmit_us); Serial.println(F(" us")); }
    if (blanking)     { Serial.print(F("Wait:\t\t\t\t")); Serial.print(tWait_us); Serial.println(F(" us")); }
    Serial.print(F("Common-mode:\t\t\t")); Serial.print(tCommonMode_us); Serial.println(F(" us"));
    Serial.print(F("Autozero:\t\t\t")); Serial.print(tAutozero_us); Serial.println(F(" us"));
    if (shortTOF)
    {
        Serial.print(F("Transmit:\t\t\t")); Serial.print(tTransmit_us); Serial.println(F(" us"));
        Serial.print(F("Mask/Blank:\t\t\t")); Serial.print(tMask_us); Serial.println(F(" us"));
    }
    else if (not blanking)
    {
        Serial.print(F("Wait/Echo listen:\t\t\t")); Serial.print(tWait_us); Serial.println(F(" us"));
    }
    Serial.print(F("Echo listen:\t\t\t")); Serial.print(tEchoListen_us); Serial.println(F(" us"));
    Serial.print(F("End:\t\t\t\t")); Serial.print(T1_us); Serial.println(F(" us"));
}


uint8_t TDC1000::spiReadReg8(const uint8_t addr)
{
    SPI.beginTransaction(SPISettings(TDC1000_SPI_CLK_MAX, TDC1000_SPI_ORDER, TDC1000_SPI_MODE));
    digitalWrite(m_pinCs, LOW);

    SPI.transfer((addr & TDC1000_SPI_REG_ADDR_MASK) | TDC1000_SPI_REG_READ);
    uint8_t val = SPI.transfer(0u);

    digitalWrite(m_pinCs, HIGH);
    SPI.endTransaction();

    return val;
}

void TDC1000::spiWriteReg8(const uint8_t addr, const uint8_t val)
{
    SPI.beginTransaction(SPISettings(TDC1000_SPI_CLK_MAX, TDC1000_SPI_ORDER, TDC1000_SPI_MODE));
    digitalWrite(m_pinCs, LOW);

    (void)SPI.transfer16((((addr & TDC1000_SPI_REG_ADDR_MASK) | TDC1000_SPI_REG_WRITE) << 8) | val);

    digitalWrite(m_pinCs, HIGH);
    SPI.endTransaction();
}

void TDC1000::spiRmwReg8(const uint8_t adr, const uint8_t clr, const uint8_t set)
{
    uint8_t val = spiReadReg8(adr);
    val &= ~clr;
    val |= set;
    spiWriteReg8(adr, val);
}
