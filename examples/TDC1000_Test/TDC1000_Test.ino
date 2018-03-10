#define USE_SI5351

#include "TDC1000.h"

#define PIN_TDC1000_START     (2)
#define PIN_TDC1000_STOP      (3)
#define PIN_TDC1000_TRIGGER   (4)
#define PIN_TDC1000_RESET     (5)
//#define PIN_TDC1000_CHSEL     (7)
//#define PIN_TDC1000_ERRB      (8)
#define PIN_TDC1000_SPI_CS    (9)
#ifndef USE_SI5351
#define PIN_TDC1000_CLKIN     (10)
#endif

//#define PIN_DEBUG1            (5)
//#define PIN_DEBUG2            (A0)

#define TDC1000_CLKIN_FREQ_HZ  (2*40000)
#define TDC1000_CLKIN_FREQ_DIV (TDC1000::TxFreqDivider::Div2)

#ifdef USE_SI5351
#include <si5351.h>
#else
#include <TimerOne.h>
#define PWM_CYCLE_US (1000000/TDC1000_CLKIN_FREQ_HZ)
#endif

#ifdef USE_SI5351
static Si5351 si5351;
#endif

static TDC1000 usafe(PIN_TDC1000_SPI_CS, PIN_TDC1000_RESET);

static volatile uint16_t tstart_us;
static volatile uint16_t tstop_us;
static void irqHandlerStart()
{
    if (not(tstart_us))
    {
        tstart_us = micros();
#ifdef PIN_DEBUG1
        digitalWrite(PIN_DEBUG1, HIGH);
#endif
    }
}
static void irqHandlerStop()
{
    if (not(tstop_us))
    {
        tstop_us = micros();
#ifdef PIN_DEBUG2
        digitalWrite(PIN_DEBUG2, HIGH);
#endif
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println(F("-- Starting TDC1000 test --"));
    while (not usafe.begin())
    {
        Serial.println(F("Failed to init TDC1000"));
        delay(1000);
    }
    Serial.println(F("TDC1000 init OK"));

    bool ok = true;
    ok &= usafe.setTriggerEdge(true);
    ok &= usafe.setTx(TDC1000_CLKIN_FREQ_DIV, 6 /*pulses*/, 31 /*shift*/, true /*damping*/);
    ok &= usafe.setRx(false /*multiEcho*/);
    ok &= usafe.setRxSensitivity(TDC1000::RxDacEchoThreshold::m220mV, TDC1000::RxPgaGain::g21dB, TDC1000:: RxLnaFbMode::resistive );
    ok &= usafe.setRepeat(TDC1000::TxRxCycles::x1, 0 /*expected pulses*/);
    ok &= usafe.setTofMeasuementShort(TDC1000::T0::ClkInDiv1, TDC1000::TxAutoZeroPeriod::T0x64,
                                    TDC1000::TxBlankPeriod::T0x16, TDC1000::TxEchoTimeoutPeriod::disabled);
    ok &= usafe.setMeasureTOF(TDC1000::TxRxChannel::Channel1, TDC1000::TofMode::Mode2);
    //ok &= usafe.setMeasureTOF(TDC1000::TxRxChannel::Swap, TDC1000::TofMode::Mode2);
    usafe.dumpSettings(TDC1000_CLKIN_FREQ_HZ);

    if (not ok)
    {
        Serial.println(F("Failed to configure TDC1000"));
        while(1) {};
    }

    pinMode(PIN_TDC1000_START, INPUT);
    pinMode(PIN_TDC1000_STOP, INPUT);

    digitalWrite(PIN_TDC1000_TRIGGER, LOW);
    pinMode(PIN_TDC1000_TRIGGER, OUTPUT);

//    digitalWrite(PIN_TDC1000_CHSEL, LOW);
//    pinMode(PIN_TDC1000_CHSEL, OUTPUT);

//    pinMode(PIN_TDC1000_ERRB, INPUT_PULLUP);    // open drain, active low

#ifdef USE_SI5351
    while (not si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0))
    {
        Serial.println(F("Failed to init Si5351"));
        delay(1000);
    }

    // Set CLK0 to output TDC1000_CLKIN_FREQ_HZ
    si5351.set_freq(TDC1000_CLKIN_FREQ_HZ * SI5351_FREQ_MULT, SI5351_CLK0);

    // Query a status update and wait a bit to let the Si5351 populate the
    // status flags correctly.
    si5351.update_status();
    delay(500);

    si5351.update_status();
    Serial.print("SYS_INIT: ");
    Serial.print(si5351.dev_status.SYS_INIT);
    Serial.print("  LOL_A: ");
    Serial.print(si5351.dev_status.LOL_A);
    Serial.print("  LOL_B: ");
    Serial.print(si5351.dev_status.LOL_B);
    Serial.print("  LOS: ");
    Serial.print(si5351.dev_status.LOS);
    Serial.print("  REVID: ");
    Serial.println(si5351.dev_status.REVID);
#else
    // Configure PWM to generate a pulse train of TDC1000_CLKIN_FREQ_HZ [Hz]
    // which is to be used as TDC1000 CLKIN.
    Timer1.initialize(PWM_CYCLE_US);
    Timer1.pwm(PIN_TDC1000_CLKIN, 1023);
    Timer1.setPwmDuty(PIN_TDC1000_CLKIN, 512);
#endif

#ifdef PIN_DEBUG1
    pinMode(PIN_DEBUG1, OUTPUT);
#endif
#ifdef PIN_DEBUG2
    pinMode(PIN_DEBUG2, OUTPUT);
#endif
    attachInterrupt(digitalPinToInterrupt(PIN_TDC1000_START), irqHandlerStart, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_TDC1000_STOP),  irqHandlerStop,  RISING);
}

inline uint16_t elapsedMicros(const uint16_t start_us)
{
    return uint16_t(micros()) - start_us;
}


void loop()
{
    tstart_us = 0u;
    tstop_us  = 0u;

    usafe.clearErrorFlags();
    usafe.resetStatemachine();

    // Trigger new measurement
    digitalWrite(PIN_TDC1000_TRIGGER, HIGH);
    digitalWrite(PIN_TDC1000_TRIGGER, LOW);

    bool timeout = false;
    unsigned long tstart = micros();
    for (;;)
    {
        if (tstop_us) break;
        if (micros() - tstart > 5000u)
        {
            timeout = true;
            break;
        }
    }
#ifdef PIN_DEBUG1
    digitalWrite(PIN_DEBUG1, LOW);
#endif
#ifdef PIN_DEBUG2
    digitalWrite(PIN_DEBUG2, LOW);
#endif

    bool sigWeak, noSig, sigHigh;
    usafe.getErrorFlags(sigWeak, noSig, sigHigh);

    if (timeout) Serial.print(F("timeout "));
    if (sigWeak) Serial.print(F("sigweak "));
    if (noSig)   Serial.print(F("nosig "));
    if (sigHigh) Serial.print(F("sighigh "));

    Serial.println(tstop_us - tstart_us);
}
