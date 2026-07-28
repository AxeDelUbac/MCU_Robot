#include "GlobalSpeed.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_DECLARE(g0b1re, LOG_LEVEL_INF);

/* ── Encodeur GPIO specs depuis le devicetree ───────────────────── */
static const struct gpio_dt_spec enc_specs[4] = {
    GPIO_DT_SPEC_GET(DT_NODELABEL(enc_fl), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(enc_fr), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(enc_rl), gpios),
    GPIO_DT_SPEC_GET(DT_NODELABEL(enc_rr), gpios),
};

static struct gpio_callback enc_cb_data[4];

/* Compteurs atomiques de pulses — incrémentés en ISR, lus et remis à 0 dans la tâche */
static atomic_t pulse_count[4] = {
    ATOMIC_INIT(0), ATOMIC_INIT(0), ATOMIC_INIT(0), ATOMIC_INIT(0)
};

/* Un callback par encodeur pour éviter toute ambiguïté */
static void enc_fl_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{ atomic_inc(&pulse_count[0]); }

static void enc_fr_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{ atomic_inc(&pulse_count[1]); }

static void enc_rl_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{ atomic_inc(&pulse_count[2]); }

static void enc_rr_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{ atomic_inc(&pulse_count[3]); }

static gpio_callback_handler_t const enc_handlers[4] = {
    enc_fl_isr, enc_fr_isr, enc_rl_isr, enc_rr_isr
};

static const char *const enc_names[4] = { "FL", "FR", "RL", "RR" };

/* ── Données globales ───────────────────────────────────────────── */
GlobalSpeed_t tGlobalSpeed;

/* ── Init ───────────────────────────────────────────────────────── */
void GlobalSpeed_init(void)
{
    for (int i = 0; i < 4; i++) {
        if (!device_is_ready(enc_specs[i].port)) {
            LOG_ERR("Encoder %s GPIO port not ready", enc_names[i]);
            continue;
        }
        gpio_pin_configure_dt(&enc_specs[i], GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&enc_specs[i], GPIO_INT_EDGE_TO_ACTIVE);
        gpio_init_callback(&enc_cb_data[i], enc_handlers[i], BIT(enc_specs[i].pin));
        gpio_add_callback(enc_specs[i].port, &enc_cb_data[i]);
        LOG_INF("Encoder %s ready on pin %d", enc_names[i], enc_specs[i].pin);
    }
}

/* ── Calcul vitesse ─────────────────────────────────────────────── */
float GlobalSpeed_getMeanSpeedInRPM(float *fSpeedEncodeRPM, float fPeriodMs)
{
    /* Lecture atomique et remise à zéro des compteurs */
    int pulses[4];
    for (int i = 0; i < 4; i++) {
        pulses[i] = (int)atomic_set(&pulse_count[i], 0);
    }

    /*
     * Mapping encodeur → roue (identique à l'Arduino original) :
     *   pulse_count[0]=FL  [1]=FR  [2]=RL  [3]=RR
     */
    tGlobalSpeed.fWheelSpeed[0] = rotaryEncoder_getSpeedRpm(
        &tGlobalSpeed.rotaryEncoderFrontLeft,  pulses[0], fPeriodMs);
    tGlobalSpeed.fWheelSpeed[1] = rotaryEncoder_getSpeedRpm(
        &tGlobalSpeed.rotaryEncoderFrontRight, pulses[1], fPeriodMs);
    tGlobalSpeed.fWheelSpeed[2] = rotaryEncoder_getSpeedRpm(
        &tGlobalSpeed.rotaryEncoderRearLeft,   pulses[2], fPeriodMs);
    tGlobalSpeed.fWheelSpeed[3] = rotaryEncoder_getSpeedRpm(
        &tGlobalSpeed.rotaryEncoderRearRight,  pulses[3], fPeriodMs);

    for (int i = 0; i < 4; i++) {
        fSpeedEncodeRPM[i] = tGlobalSpeed.fWheelSpeed[i];
    }

    tGlobalSpeed.fMeanRpm = (tGlobalSpeed.fWheelSpeed[0] + tGlobalSpeed.fWheelSpeed[1] +
                             tGlobalSpeed.fWheelSpeed[2] + tGlobalSpeed.fWheelSpeed[3]) / 4.0f;
    return tGlobalSpeed.fMeanRpm;
}

/* ── Debug ──────────────────────────────────────────────────────── */
void GlobalSpeed_debug(void)
{
    LOG_INF("RPM  FL=%d  FR=%d  RL=%d  RR=%d  mean=%d",
        (int)tGlobalSpeed.fWheelSpeed[0],
        (int)tGlobalSpeed.fWheelSpeed[1],
        (int)tGlobalSpeed.fWheelSpeed[2],
        (int)tGlobalSpeed.fWheelSpeed[3],
        (int)tGlobalSpeed.fMeanRpm);
}
