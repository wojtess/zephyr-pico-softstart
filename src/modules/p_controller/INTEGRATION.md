/**
 * @file INTEGRATION.md
 * @brief P-controller integration example
 *
 * This file shows how to integrate the p_controller module into main.c
 */

/*
 * STEP 1: Add include to main.c
 * ----------------------------------------
 */
#include "modules/p_controller/p_controller.h"


/*
 * STEP 2: Declare P-controller context
 * ----------------------------------------
 */
static struct p_ctrl_ctx p_ctrl_ctx;


/*
 * STEP 3: Initialize P-controller (in main())
 * ----------------------------------------
 */
int main(void)
{
    /* ... other init code ... */

    /* Initialize P-controller */
    ret = p_ctrl_init(&p_ctrl_ctx);
    if (ret != 0) {
        printk("Failed to init P-controller: %d\n", ret);
        return 0;
    }

    /* Set callbacks */
    p_ctrl_set_callbacks(&p_ctrl_ctx,
                        led_pwm_set_duty,      // PWM callback
                        adc_ctrl_read_channel0, // ADC callback
                        p_ctrl_stream_callback); // Stream callback

    /* Set initial parameters */
    p_ctrl_set_feed_forward(&p_ctrl_ctx, 10);  // 10% base PWM
    p_ctrl_set_gain(&p_ctrl_ctx, 100);         // Gain = 1.0

    /* ... rest of init ... */
}


/*
 * STEP 4: Implement stream callback
 * ----------------------------------------
 */
static void p_ctrl_stream_callback(uint16_t setpoint, uint16_t measured, uint8_t pwm)
{
    /*
     * Send streaming data via UART
     * Format: [0x10][SETPOINT_H][SETPOINT_L][MEASURED_H][MEASURED_L][PWM][CRC8]
     *
     * You can use tx_buffer_put_isr() to send data safely
     */

    uint8_t frame[7];
    frame[0] = 0x10;  // CMD_P_CTRL_STREAM
    frame[1] = (setpoint >> 8) & 0xFF;
    frame[2] = setpoint & 0xFF;
    frame[3] = (measured >> 8) & 0xFF;
    frame[4] = measured & 0xFF;
    frame[5] = pwm;
    frame[6] = crc8(frame, 6);  // Assuming you have crc8 function

    /* Send via TX buffer (ISR-safe) */
    tx_buffer_put_isr(frame, 7);
}


/*
 * STEP 5: Add protocol handlers
 * ----------------------------------------
 *
 * In your protocol handler (e.g., in protocol.c or main.c):
 */

/* Command 0x10: Set P-controller mode */
void protocol_handle_set_p_ctrl_mode(const uint8_t *frame)
{
    uint8_t mode = frame[1];
    p_ctrl_set_mode(&p_ctrl_ctx, mode);
    protocol_send_ack();
}

/* Command 0x11: Set setpoint */
void protocol_handle_set_setpoint(const uint8_t *frame)
{
    uint16_t setpoint = (frame[1] << 8) | frame[2];
    p_ctrl_set_setpoint(&p_ctrl_ctx, setpoint);
    protocol_send_ack();
}

/* Command 0x12: Set gain */
void protocol_handle_set_gain(const uint8_t *frame)
{
    uint16_t gain = (frame[1] << 8) | frame[2];
    p_ctrl_set_gain(&p_ctrl_ctx, gain);
    protocol_send_ack();
}

/* Command 0x13: Set feed-forward */
void protocol_handle_set_feed_forward(const uint8_t *frame)
{
    uint8_t ff = frame[1];
    p_ctrl_set_feed_forward(&p_ctrl_ctx, ff);
    protocol_send_ack();
}

/* Command 0x14: Start/stop streaming */
void protocol_handle_p_ctrl_stream(const uint8_t *frame)
{
    uint8_t enable = frame[1];
    uint8_t rate_hz = frame[2];

    if (enable) {
        p_ctrl_start_stream(&p_ctrl_ctx, rate_hz);
    } else {
        p_ctrl_stop_stream(&p_ctrl_ctx);
    }
    protocol_send_ack();
}

/* Command 0x15: Get current P-controller status */
void protocol_handle_get_p_ctrl_status(const uint8_t *frame)
{
    uint8_t response[8];
    response[0] = 0x15;  // CMD_GET_P_CTRL_STATUS
    response[1] = atomic_get(&p_ctrl_ctx.mode);
    response[2] = (atomic_get(&p_ctrl_ctx.setpoint) >> 8) & 0xFF;
    response[3] = atomic_get(&p_ctrl_ctx.setpoint) & 0xFF;
    response[4] = (atomic_get(&p_ctrl_ctx.gain) >> 8) & 0xFF;
    response[5] = atomic_get(&p_ctrl_ctx.gain) & 0xFF;
    response[6] = atomic_get(&p_ctrl_ctx.feed_forward);
    response[7] = crc8(response, 7);

    tx_buffer_put_isr(response, 8);
}


/*
 * STEP 6: Update CMakeLists.txt
 * ----------------------------------------
 *
 * Add p_controller.c to target_sources in CMakeLists.txt:
 *
 * target_sources(app PRIVATE
 *     src/main.c
 *     ...
 *     src/modules/p_controller/p_controller.c
 * )
 */
