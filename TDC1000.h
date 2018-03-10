#pragma once
#include <inttypes.h>

class TDC1000
{
    public:
        /**
         * Constructor.
         * @param[in] pinEnable   Mcu pin controlling TDC1000 enable input.
         * @param[in] pinCs       Mcu pin controlling TDC1000 SPI CSB input.
         */
        TDC1000(const uint8_t pinCs, const uint8_t pinReset);

        /**
         * Initialize TDC1000.
         */
        bool begin();

        /**
         * Set polarity of trigger input.
         * @param[in] rising  Set to true to configure for rising edges, false for falling edges.
         * @return True when setup was successful, false otherwise.
         */
        bool setTriggerEdge(const bool rising);

        enum class TxFreqDivider{ Div2 = 0, Div4 = 1, Div8 = 2, Div16 = 3, Div32 = 4, Div64 = 5, Div128 = 6, Div256 = 7 };
        /**
         * Configure transmit parameters.
         * @param[in] div      Frequency divider for Tx clock and T1.
         * @param[in] pulses   Number of Tx pulses in a burst, range [0..31].
         * @param[in] shift    Tx pulse 180 degree pulse shift position, range [0..31], recommended [2..31].
         * @param[in] damping  True to enable Tx burs damping, false to disable.
         * @return True when setup was successful, false otherwise.
         */
        bool setTx(const TxFreqDivider div, const uint8_t pulses, const uint8_t shift = 31, const bool damping = false);

        /**
         * Configure receive parameters.
         * @param[in] multiEcho  True enables multi-echo reception (multiple pulses merge into one),
         *                       false enables single pulse reception.
         * @return True when setup was successful, false otherwise.
         */
        bool setRx(const bool multiEcho);

        enum class RxDacEchoThreshold{ m35mV = 0, m50mV = 1, m75mV = 2, m125mV = 3, m220mV = 4, m410mV = 5, m775mV = 6, m1500mV = 7 };
        enum class RxPgaGain{ disabled = 8, g0dB = 0, g3dB = 1, g6dB = 2, g9dB = 3, g12dB = 4, g15dB = 5, g18dB = 6, g21dB = 7 };
        enum class RxLnaFbMode{ disabled = 2, capacitive = 0, resistive = 1 };
        /**
         * Configure Rx sensitivity, Pga and Lna.
         * @param[in] dacTh    Echo qualification DAC threshold level with respect to VCOM.
         * @param[in] pgaGain  Set PGA gain, or set to disabled to bypass PGA.
         * @param[in] lnaFb    Set LNA feedback mode, or set to disabled to bypass LNA.
         * @return True when setup was successful, false otherwise.
         */
        bool setRxSensitivity(const RxDacEchoThreshold dacTh, const RxPgaGain pgaGain, const RxLnaFbMode lnaFb);

        enum class TxRxCycles{ x1 = 0, x2 = 2, x4 = 2, x8 = 3, x16 = 4, x32 = 5, x64 = 6, x128 = 7};
        /**
         * Configure automatic pulse averaging.
         * @param[in] cycles          Number of measurement cycles to average.
         * @param[in] expectedEvents  Number of expected receive events per cycle. Range [0..7] stop pulses output.
         * @return True when setup was successful, false otherwise.
         */
        bool setRepeat(const TxRxCycles cycles, const uint8_t expectedEvents);

        enum class TxRxChannel{ Channel1 = 0, Channel2 = 1, Swap = 2, External = 3 };
        enum class TofMode{ Mode0 = 0, Mode1 = 1, Mode2 = 2 };
        /**
         * Configure TDC1000 for TOF measurement, setting measurement channel and TOF mode.
         * @param[in] chanSel  Channel to measure. Select channel 1 or 2, automatic swap after each measurement
         *                     or selcted externally through CHSEL pin.
         * @param[in] mode     TOF measurement mode:
         *                     - Mode0: Fluid level and identification. Transducer transmits then receives
         *                              its own signal.
         *                     - Mode1: Flow sensing. Transducer 1 transmits and transducer 2 receives.
         *                              Requires manual channel switching
         *                     - Mode2: Flow sensing (preferred). Transducer 1 transmits and transducer 2
         *                              receives. Allows automatic channel switching.
         * @return True when setup was successful, false otherwise.
         */
        bool setMeasureTOF(const TxRxChannel chanSel, const TofMode mode);

        enum class TempMode{ REF_RTD1_RTD2 = 0, REF_RTD1 = 1 };
        enum class TempRtdSel{ PT1000 = 0, PT500 = 1 };
        enum class TempClkDiv{ div8 = 0, useTX_FREQ_DIV = 1 };
        /**
         * Configure TDC1000 for temperature measurement.
         * @param[in] mode    Temperature measurement mode.
         * @param[in] rtd     Type of RTD connected.
         * @param[in] div     Clock divider for temperature measurement.
         * @return True when setup was successful, false otherwise.
         */
        bool setMeasureTemp(const TempMode mode, const TempRtdSel rtd, const TempClkDiv div);

        enum class T0{ ClkInDiv1 = 0, ClkInDiv2 = 1 };
        enum class TxAutoZeroPeriod{ T0x64 = 0, T0x128 = 1, T0x256 = 2, T0x512 = 3 };
        enum class TxBlankPeriod{ T0x8 = 0, T0x16 = 1, T0x32 = 2, T0x64 = 3, T0x128 = 4, T0x256 = 5, T0x512 = 6, T0x1024 = 7 };
        enum class TxEchoTimeoutPeriod{ disabled = 4, T0x128 = 0, T0x256 = 1, T0x512 = 2, T0x1024 = 3 };
        /**
         * Configure for short TOF measurement, ref datasheet 8.4.6.1
         * @param[in] clkDiv       CLKIN divider to generate T0.
         * @param[in] autoZero     AutoZero period.
         * @param[in] blank        Blank/mask period.
         * @param[in] echoTimeout  Echo listen period.
         * @return True when setup was successful, false otherwise.
         */
        bool setTofMeasuementShort(const T0 clkDiv, const TxAutoZeroPeriod autoZero, const TxBlankPeriod blank,
                                   const TxEchoTimeoutPeriod echoTimeout);

        /**
         * Configure for standard TOF measurement with optional blanking, ref datasheet 8.4.6.2 and 8.4.6.3
         * @param[in] clkDiv       CLKIN divider to generate T0.
         * @param[in] autoZero     AutoZero period.
         * @param[in] echoTimeout  Echo listen period.
         * @param[in] timingReg    Range [0..1023].
         * @param[in] blanking     True to enable power blanking, just after transmit pulses.
         * @return True when setup was successful, false otherwise.
         */
        bool setTofMeasuementStandard(const T0 clkDiv, const TxAutoZeroPeriod autoZero, const TxEchoTimeoutPeriod echoTimeout,
                                      const uint16_t timingReg, const bool blanking);


        /**
         * Retrieve status of error flags.
         * @param[out] sigWeak          When true, the number of received and qualified zero-crossings was
                                        less than the expected number set in NUM_RX field and a timeout occurred.
         * @param[out] noSig            When true, no signals were received and timeout occurred.
         * @param[out] sigHigh          When true, the received echo amplitude exceeds the largest echo
         *                              qualification threshold at the input of the comparators.
         *                              Only reported when RxDacEchoThreshold is set to m1500mV (-1500mV).
         */
        void getErrorFlags(bool &sigWeak, bool &noSig, bool &sigHigh);

        /**
         * Clear any error flags currently set.
         */
        void clearErrorFlags();

        /**
         * Reset the TDC1000 statemachine.
         * Also halts active measurements, returns the device to the SLEEP or READY mode and resets
         * the average counter and automatic channel selection in measurement Mode 2.
         */
        void resetStatemachine();

//        bool TEST();

        /**
         * Make a verbose dump to serial device of all TDC1000 settings.
         */
        void dumpSettings(const uint32_t freqClkInHz = 0ul);

    private:
        uint8_t  m_pinCs;           //< Mcu pin controlling TDC7200 SPI CSB input.
        uint8_t  m_pinReset;        //< Mcu pin controlling TDC7200 Reset input.

        /** Read a single byte through spi.
         * @param[in] addr   Register address to read. 
         * @return Register value at requested address.
         */
        uint8_t  spiReadReg8(const uint8_t addr);

        /** Write a single byte through spi.
         * @param[in] addr   Register address to write. 
         * @param[in] val    Value to write. 
         */
        void     spiWriteReg8(const uint8_t addr, const uint8_t val);

        /**
         * Read-modify-write bits in an 8bit spi register.
         * @param[in] addr   Register address to modify.
         * @param[in] clr    Bits to clear in register content.
         * @param[in] set    Bits to set in register content.
         */
        void     spiRmwReg8(const uint8_t addr, const uint8_t clr, const uint8_t set);
};

