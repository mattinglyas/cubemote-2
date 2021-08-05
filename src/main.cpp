
#include <stdio.h>
#include <string.h>
#include <esp_timer.h>

#include <driver/rmt.h>
#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#include <driver/gpio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "gamecube.h"

extern "C"
{
    void app_main(void);
}

/*************************************************************************
 * Definitions
 * 
 *************************************************************************/
#define NSI_FRAME_MAX 64
#define NSI_BIT_PERIOD_TICKS 8
#define RMT_RX_GPIO_NUM GPIO_NUM_16
#define RMT_TX_GPIO_NUM GPIO_NUM_17
#define RMT_RX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_CHANNEL RMT_CHANNEL_2
#define RMT_CLK_DIV 40
#define RMT_RX_IDLE_THRESHOLD_US 9500
#define RMT_CLOCK_SPEED 80000000

/*************************************************************************
 * Pins
 * 
 *************************************************************************/

/*************************************************************************
 * Global Vars 
 * 
 *************************************************************************/
static uint8_t nsi_frame[NSI_FRAME_MAX];
static uint8_t nsi_frame_size_bytes;

static uint8_t nsi_response[NSI_FRAME_MAX];
static uint16_t nsi_response_size_bits;

static rmt_config_t rx_config;
static rmt_config_t tx_config;

static rmt_item32_t items[NSI_FRAME_MAX * 8];

/*************************************************************************
 * Methods
 * 
 *************************************************************************/

static uint16_t IRAM_ATTR construct_response(uint8_t command, uint8_t *bytes)
{
    switch (command)
    {
    case CONSOLE_PROBE:
        bytes[0] = 0x09; // it is like this for some reason
        bytes[1] = 0x00;
        bytes[2] = 0x03;
        bytes[3] = 0x80; // stop bit
        return 25;
        break;
    case CONSOLE_PROBE_ORIGIN:
        bytes[0] = 0x00; // 0, 0, 0, start, y, x, b, a
        bytes[1] = 0x00; // l, r, z, d-up, d-down, d-r, d-l
        bytes[2] = 0x0F; // left stick x axis
        bytes[3] = 0x0F; // left stick y axis
        bytes[4] = 0x0F; // c stick x axis
        bytes[5] = 0x0F; // c stick y axis
        bytes[6] = 0x00; // left trigger
        bytes[7] = 0x00; // right trigger
        bytes[8] = 0x00; // two null bytes at end
        bytes[9] = 0x00;
        bytes[10] = 0x80; // stop bit
        return 73;
        break;
    case CONSOLE_POLL:
    case CONSOLE_POLL_RUMBLE:
        /* TODO put actual controller data here */
        bytes[0] = 0x00; // 0, 0, 0, start, y, x, b, a
        bytes[1] = 0x00; // l, r, z, d-up, d-down, d-r, d-l
        bytes[2] = 0x0F; // left stick x axis
        bytes[3] = 0x0F; // left stick y axis
        bytes[4] = 0x0F; // c stick x axis
        bytes[5] = 0x0F; // c stick y axis
        bytes[6] = 0x00; // left trigger
        bytes[7] = 0x00; // right trigger
        bytes[8] = 0x80; // stop bit
        return 65;
        break;
    default:
        // not good
        return 0;
        break;
    }
}

static uint8_t IRAM_ATTR bytes_to_console_command(uint8_t *bytes)
{
    /* POLL and POLL_RUMBLE will be a majority of the commands */
    if (memcmp(bytes, console_commands[CONSOLE_POLL], 2) == 0 && (bytes[3] == console_commands[CONSOLE_POLL][3]))
    {
        if (bytes[2] == 0x00 && bytes[3])
            return CONSOLE_POLL;
        else if (bytes[2] == 0x01)
            return CONSOLE_POLL_RUMBLE;
        else
            return CONSOLE_INVALID;
    }

    if (memcmp(bytes, console_commands[CONSOLE_PROBE], 4) == 0)
        return CONSOLE_PROBE;

    if (memcmp(bytes, console_commands[CONSOLE_PROBE_ORIGIN], 4) == 0)
        return CONSOLE_PROBE_ORIGIN;

    return CONSOLE_INVALID;
}

static uint16_t IRAM_ATTR bytes_to_nsi(uint8_t *bytes, rmt_item32_t *nsi, uint16_t length)
{
    uint8_t byte_idx = 0;
    uint16_t nsi_idx = 0;
    uint8_t nsi_mask = 0;

    while (nsi_idx < length)
    {
        byte_idx = nsi_idx >> 3;
        nsi_mask = 1 << (nsi_idx & 0x07);

        if ((bytes[byte_idx] & nsi_mask) == 0)
        {
            /* send zero */
            nsi[nsi_idx].duration0 = 7;
            nsi[nsi_idx].duration1 = 3;
            nsi[nsi_idx].level0 = 0;
            nsi[nsi_idx].level1 = 1;
        }
        else
        {
            /* send one */
            nsi[nsi_idx].duration0 = 3;
            nsi[nsi_idx].duration1 = 7;
            nsi[nsi_idx].level0 = 0;
            nsi[nsi_idx].level1 = 1;
        }

        nsi_idx++;
    }

    return nsi_idx;
}

static uint8_t IRAM_ATTR nsi_to_bytes(rmt_item32_t *nsi, uint8_t *bytes)
{
    uint8_t byte_idx = 0;
    uint16_t nsi_idx = 0;

    memset(bytes, 0, NSI_FRAME_MAX);

    while (nsi[nsi_idx].duration1 != 0)
    {
        byte_idx = nsi_idx >> 3;

        bytes[byte_idx] >>= 1;
        if (nsi[nsi_idx].duration1 > (NSI_BIT_PERIOD_TICKS / 2))
        {
            bytes[byte_idx] |= 0x80;
        }

        nsi_idx++;
    }

    /* Stop bit */
    byte_idx = nsi_idx >> 3;
    bytes[byte_idx] >>= 1;
    bytes[byte_idx] |= 0x80;

    return byte_idx + 1;
}

// ISR from https://github.com/darthcloud/esp32_rmt_nsi_sniffer
static void IRAM_ATTR rmt_isr(void *arg)
{
    const uint32_t intr_st = RMT.int_st.val;
    uint32_t status = intr_st;
    uint8_t i, channel, command;

    while (status != 0)
    {
        i = __builtin_ffs(status) - 1;
        status &= ~(1 << i);
        channel = i / 3;
        switch (i % 3)
        {
        /* RX End */
        case 1:
            if (channel == RMT_RX_CHANNEL)
            {
                nsi_frame_size_bytes = nsi_to_bytes((rmt_item32_t *)RMTMEM.chan[channel].data32, nsi_frame);
                command = bytes_to_console_command(nsi_frame);
                nsi_response_size_bits = construct_response(command, nsi_response);

                ets_printf("%02x: ", command);
                for (uint8_t i = 0; i < nsi_frame_size_bytes; i++)
                {
                    ets_printf("%02x ", nsi_frame[i]);
                }
                ets_printf("-> ");
                for (uint8_t i = 0; i < nsi_response_size_bits >> 3; i++)
                {
                    ets_printf("%02x ", nsi_response[i]);
                }
                ets_printf("\n");
            }

            RMT.conf_ch[channel].conf1.mem_wr_rst = 1;
            RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_RX;
            RMT.conf_ch[channel].conf1.rx_en = 1;

            break;
        /* ERR */
        case 2:
            ets_printf("RMT ERR\n");
            RMT.int_ena.val &= (~(BIT(i)));
            break;
        }
    }

    RMT.int_clr.val = intr_st;
}

void rmt_rx_init()
{
    rx_config.channel = RMT_RX_CHANNEL;
    rx_config.gpio_num = RMT_RX_GPIO_NUM;
    rx_config.clk_div = RMT_CLK_DIV;
    rx_config.mem_block_num = 4;
    rx_config.rmt_mode = RMT_MODE_RX;

    rx_config.rx_config.filter_en = 0;
    rx_config.rx_config.filter_ticks_thresh = 0;
    rx_config.rx_config.idle_threshold = (NSI_BIT_PERIOD_TICKS * 4); //RMT_RX_IDLE_THRESHOLD_US / 10 * (RMT_CLOCK_SPEED / RMT_CLK_DIV) / 100000;

    rmt_config(&rx_config);
}

void rmt_tx_init()
{
    tx_config.channel = RMT_TX_CHANNEL;
    tx_config.gpio_num = RMT_TX_GPIO_NUM;
    tx_config.mem_block_num = 1;
    tx_config.clk_div = RMT_CLK_DIV;
    tx_config.rmt_mode = RMT_MODE_TX;

    tx_config.tx_config.carrier_freq_hz = 24000000;
    tx_config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
    tx_config.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
    tx_config.tx_config.carrier_duty_percent = 50;
    tx_config.tx_config.carrier_en = false;
    tx_config.tx_config.loop_en = false;
    tx_config.tx_config.idle_output_en = true;

    rmt_config(&tx_config);
}

void app_main()
{
    //rmt_isr_register(rmt_isr, NULL, 0, NULL);

    rmt_tx_init();
    rmt_rx_init();

    rmt_driver_install(tx_config.channel, 0, 0);
    rmt_driver_install(rx_config.channel, 3000, 0);

    RingbufHandle_t rx_ring_buffer = NULL;
    rmt_get_ringbuf_handle(rx_config.channel, &rx_ring_buffer);

    rmt_rx_start(rx_config.channel, 1);

    for (;;)
    {
        //nsi_response_size_bits = construct_response(CONSOLE_PROBE, nsi_response);
        //bytes_to_nsi(nsi_response, items, nsi_response_size_bits);
        //rmt_write_items(tx_config.channel, items, nsi_response_size_bits, 1);

        size_t rx_size = 0;
        rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rx_ring_buffer, &rx_size, portMAX_DELAY);

        if (item)
        {
            nsi_frame_size_bytes = nsi_to_bytes(item, nsi_frame);
            vRingbufferReturnItem(rx_ring_buffer, (void *)item);

            //rmt_rx_stop(rx_config.channel);

            uint8_t command = bytes_to_console_command(nsi_frame);
            nsi_response_size_bits = construct_response(command, nsi_response);
            bytes_to_nsi(nsi_response, items, nsi_response_size_bits);
            
            ets_printf("%02x: ", command);
            for (uint8_t i = 0; i < nsi_frame_size_bytes; i++)
            {
                ets_printf("%02x ", nsi_frame[i]);
            }

            if (command != CONSOLE_INVALID) 
            {
                rmt_write_items(tx_config.channel, items, nsi_response_size_bits, 1);
                
                ets_printf("-> ");
                for (uint8_t i = 0; i < ((nsi_response_size_bits >> 3) + ((nsi_response_size_bits & 0x03) != 0)); i++)
                {
                    ets_printf("%02x ", nsi_response[i]);
                }
                ets_printf("\n");
            }   

            //rmt_rx_start(rx_config.channel, 0);
        }
    }
}