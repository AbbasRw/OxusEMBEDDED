/*
 * dwin.c
 */


#include "dwin.h"
#include <string.h>
#include "main.h"
#if DWIN_DEBUG_ENABLE
#include <stdio.h>
#endif

// PROTOCOL
#define DWIN_FRAME_HEADER_0         0x5A
#define DWIN_FRAME_HEADER_1         0xA5
#define DWIN_CMD_WRITE              0x82
#define DWIN_CMD_READ               0x83

// SYSTEM ADDRESSES
#define DWIN_VP_SYSTEM_RESET        0x0004
#define DWIN_VP_PAGE_CHANGE         0x0084
#define DWIN_VP_TOUCH_CONTROL       0x00B0

// GLOBAL
DWIN_t g_dwin;
DWIN_Stats_t g_dwin_stats;

// INTERNAL
static void dwin_rx_parse_message(void);
static void dwin_rx_push(DWIN_RX_t *rx_data);

extern void DebugPrint(const char *format, ...);


extern UART_HandleTypeDef huart1;

/**
 * @brief CRC16
 */
uint16_t dwin_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;

    while (length--) {
        crc ^= *data++;
        for (i = 0; i < 8; i++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }

    return (crc >> 8) | (crc << 8);
}

/**
 * @brief Initpage
 */
void dwin_init(void)
{
    memset(&g_dwin, 0, sizeof(DWIN_t));
    memset(&g_dwin_stats, 0, sizeof(DWIN_Stats_t));

    g_dwin.rx_parser.state = RX_STATE_WAIT_HEADER1;
}

/**
 * @brief Byte-by-byte RX (State Machine)
 */
void dwin_rx_byte_received(uint8_t byte)
{
    RX_Parser_t *p = &g_dwin.rx_parser;

    switch(p->state)
    {
        case RX_STATE_WAIT_HEADER1:  // STATE 1: HEADER1 BEKLENİYOR (5A)
            if(byte == 0x5A)
            {
                p->buffer[0] = byte;
                p->index = 1;
                p->state = RX_STATE_WAIT_HEADER2;
            }
            else if(byte == 0x00)
            {
                // 0x00 mesajı (power on/off)
                DWIN_RX_t rx_data;
                memset(&rx_data, 0, sizeof(DWIN_RX_t));
                rx_data.type = RX_TYPE_POWER_ON;
                dwin_rx_push(&rx_data);

            }
            break;

        case RX_STATE_WAIT_HEADER2:    // STATE 2: HEADER2 BEKLENİYOR (A5)
            if(byte == 0xA5)
            {
                p->buffer[1] = byte;
                p->index = 2;
                p->state = RX_STATE_WAIT_LENGTH;
            }
            else
            {
                // Hata! Başa dön
                p->state = RX_STATE_WAIT_HEADER1;

            }
            break;
        case RX_STATE_WAIT_LENGTH: // STATE 3: LENGTH BEKLENİYOR
            p->buffer[2] = byte;
            p->expected_len = byte + 3;  // Header(2) + Length(1) + Data
            p->index = 3;
            p->state = RX_STATE_WAIT_DATA;
            break;

        case RX_STATE_WAIT_DATA: // STATE 4: DATA + CRC BEKLENİYOR
            p->buffer[p->index++] = byte;

            if(p->index >= p->expected_len)
            {
                // TAM MESAJ GELDİ!
            	DebugPrint("2.Message Complete! Len=%d\r\n", p->expected_len);
                dwin_rx_parse_message();
                p->state = RX_STATE_WAIT_HEADER1;

            }
            break;

        default:
            p->state = RX_STATE_WAIT_HEADER1;
            break;
    }
}

/**
 * @brief Mesajı parse et
 */
static void dwin_rx_parse_message(void)
{
    RX_Parser_t *p = &g_dwin.rx_parser;

    // CRC kontrolü
    uint8_t len = p->buffer[2];
    uint16_t crc_rx = (p->buffer[len + 1] << 8) | p->buffer[len + 2];
    uint16_t crc_calc = dwin_crc16(&p->buffer[3], len - 2);

    if(crc_rx != crc_calc)
    {
    	// TODO buraya flash veya log için listeye at mesajı
        DebugPrint("3.CRC FAIL! RX=%04X CALC=%04X\r\n", crc_rx, crc_calc);  // ← Adım 3a

        return;
    }

    uint8_t cmd = p->buffer[3];
    DWIN_RX_t rx_data;
    memset(&rx_data, 0, sizeof(DWIN_RX_t));

    if(len == 0x05 && cmd == 0x82 &&// OK MESAJI
       p->buffer[4] == 0x4F && p->buffer[5] == 0x4B)
    {
        rx_data.type = RX_TYPE_OK;
        g_dwin_stats.total_received++;
        //dwin_rx_push(&rx_data);

    }
    else if(cmd == 0x83)  // 0x83: TOUCH veya READ RESPONSE
    {
        rx_data.vp_addr = (p->buffer[4] << 8) | p->buffer[5];
        rx_data.data_len = p->buffer[6];

        uint8_t byte_len = rx_data.data_len * 2;
        for(uint8_t i = 0; i < byte_len && i < 32; i++)
        {
            rx_data.data[i] = p->buffer[7 + i];
        }

        if(rx_data.data_len == 1)
        {
            rx_data.type = RX_TYPE_TOUCH;

//            #if DWIN_DEBUG_ENABLE
//            printf("[RX] Touch - VP:0x%04X Data:0x%02X%02X\n",
//                   rx_data.vp_addr, rx_data.data[0], rx_data.data[1]);
//            #endif
        }
        else
        {
            rx_data.type = RX_TYPE_READ_RESPONSE;

//            #if DWIN_DEBUG_ENABLE
//            printf("[RX] Read Response - VP:0x%04X Len:%d\n",
//                   rx_data.vp_addr, rx_data.data_len);
//            #endif
        }

        DebugPrint("3.CRC OK! VP=0x%04X Len=%d\r\n", rx_data.vp_addr, rx_data.data_len);  // ← Adım 3b

        dwin_rx_push(&rx_data);
    }




}

/**
 * @brief Listeye ekle
 */
static void dwin_rx_push(DWIN_RX_t *rx_data)
{
    if(g_dwin.rx_list.count >= DWIN_RX_LIST_SIZE)
    {

        return;
    }


    memcpy(&g_dwin.rx_list.list[g_dwin.rx_list.head], rx_data, sizeof(DWIN_RX_t));

    g_dwin.rx_list.head = (g_dwin.rx_list.head + 1) % DWIN_RX_LIST_SIZE;
    g_dwin.rx_list.count++;
}

/**
 * @brief Listeden al
 */
DWIN_RX_t* dwin_rx_get(void)
{
    if(g_dwin.rx_list.count == 0)
        return NULL;

    DWIN_RX_t *rx = &g_dwin.rx_list.list[g_dwin.rx_list.tail];

    g_dwin.rx_list.tail = (g_dwin.rx_list.tail + 1) % DWIN_RX_LIST_SIZE;
    g_dwin.rx_list.count--;

    return rx;
}

/**
 * @brief Liste boş mu?
 */
uint8_t dwin_rx_available(void)
{
    return g_dwin.rx_list.count;
}


/**
 * @brief Health check
 */
void dwin_check_health(void)
{
    static uint32_t last_check = 0;
    static uint16_t last_sent = 0;
    static uint16_t last_received = 0;

    uint32_t now = HAL_GetTick();
    // TODO burdaki zamalamayı kaldır daha sonra
    if((now - last_check) >= 500)
    {
        last_check = now;

        uint16_t sent = g_dwin_stats.total_sent - last_sent;
        uint16_t received = g_dwin_stats.total_received - last_received;

        if(sent != received)
        {
          //  printf("⚠️ %d mesaj, %d OK (%d kayıp)\n", sent, received, sent - received);
        }

        last_sent = g_dwin_stats.total_sent;
        last_received = g_dwin_stats.total_received;
    }
}
/**
 * @brief Komut gönder
 */
void dwin_cmd_write(uint16_t vp_addr, uint8_t *data, uint8_t data_len)
{
    uint8_t idx = 0;

    g_dwin.tx_arr[idx++] = DWIN_FRAME_HEADER_0;
    g_dwin.tx_arr[idx++] = DWIN_FRAME_HEADER_1;

    uint8_t total_len = 5 + data_len;
    g_dwin.tx_arr[idx++] = total_len;
    g_dwin.tx_arr[idx++] = DWIN_CMD_WRITE;

    g_dwin.tx_arr[idx++] = (vp_addr >> 8) & 0xFF;
    g_dwin.tx_arr[idx++] = vp_addr & 0xFF;

    for(uint8_t i = 0; i < data_len; i++)
    {
        g_dwin.tx_arr[idx++] = data[i];
    }

    uint16_t crc = dwin_crc16(&g_dwin.tx_arr[3], total_len - 2);
    g_dwin.tx_arr[idx++] = (crc >> 8) & 0xFF;
    g_dwin.tx_arr[idx++] = crc & 0xFF;

    #if DWIN_USE_DMA
        HAL_UART_Transmit_DMA(&DWIN_UART_HANDLE, g_dwin.tx_arr, idx);
    #else
        HAL_UART_Transmit(&huart1, g_dwin.tx_arr, total_len + 3, 100);
    #endif

        // --- DEBUG PRINT KISMI ---
            DebugPrint("TX: VP=0x%04X, Len=%d, Data=", vp_addr, data_len);
            for(uint8_t i = 0; i < idx; i++) {
                DebugPrint("%02X ", g_dwin.tx_arr[i]);
            }
            DebugPrint("\r\n");
    g_dwin_stats.total_sent++;
}

// SYSTEM
void dwin_system_reset(void)
{
    static const uint8_t reset_data[4] = {0x55, 0xAA, 0x5A, 0xA5};
    dwin_cmd_write(DWIN_VP_SYSTEM_RESET, (uint8_t*)reset_data, 4);
}

// PAGE
void dwin_set_page(uint8_t page_ID)
{
    uint8_t page_data[4] = {0x5a, 0x01,0x00, page_ID};
    dwin_cmd_write(DWIN_VP_PAGE_CHANGE, page_data, 4);
}

// ICON
void dwin_set_icon(uint16_t vp_address, uint8_t icon_id)
{
    uint8_t icon_data[2] = {0x00, icon_id};
    dwin_cmd_write(vp_address, icon_data, 2);
}

// BUTTON
void dwin_return_keycode_enable(uint8_t page_id, uint8_t control_id)
{
    uint8_t button_data[9];
    button_data[0] = 0x5A;
    button_data[1] = 0xA5;
    button_data[2] = 0x00;
    button_data[3] = page_id;
    button_data[4] = control_id;
    button_data[5] = 0x05;
    button_data[6] = 0x00;
    button_data[7] = 0x01;
    dwin_cmd_write(DWIN_VP_TOUCH_CONTROL, button_data, 9);
}

void dwin_return_keycode_disable(uint8_t page_id, uint8_t control_id)
{
    uint8_t button_data[9];
    button_data[0] = 0x5A;
    button_data[1] = 0xA5;
    button_data[2] = 0x00;
    button_data[3] = page_id;
    button_data[4] = control_id;
    button_data[5] = 0x05;
    button_data[6] = 0x00;
    button_data[7] = 0x00;
    dwin_cmd_write(DWIN_VP_TOUCH_CONTROL, button_data, 9);
}

// DATA VARIABLE
void dwin_data_variable_show(uint16_t sp_addr, uint16_t vp_addr)
{
    uint8_t data[2];
    data[0] = (vp_addr >> 8) & 0xFF;
    data[1] = vp_addr & 0xFF;
    dwin_cmd_write(sp_addr, data, 2);
}

void dwin_data_variable_hide(uint16_t sp_addr)
{
    uint8_t data[2] = {0xFF, 0xFF};
    dwin_cmd_write(sp_addr, data, 2);
}

void dwin_data_variable_set_integer_digits(uint16_t sp_addr, uint8_t digits)
{
    uint8_t data[2];
    data[0] = 0x80;
    data[1] = digits;
    dwin_cmd_write(sp_addr + 5, data, 2);
}

void dwin_data_variable_set_decimal_digits(uint16_t sp_addr, uint8_t digits)
{
    uint8_t data[2];
    data[0] = digits;
    data[1] = 0x07;
    dwin_cmd_write(sp_addr + 6, data, 2);
}

void dwin_data_variable_write_float(uint16_t vp_addr, float value)
{
    uint8_t float_data[4];
    uint32_t *ptr = (uint32_t*)&value;
    float_data[0] = (*ptr >> 24) & 0xFF;
    float_data[1] = (*ptr >> 16) & 0xFF;
    float_data[2] = (*ptr >> 8) & 0xFF;
    float_data[3] = *ptr & 0xFF;
    dwin_cmd_write(vp_addr, float_data, 4);
}

// PROGRESS BAR
void dwin_set_progressbar(uint16_t vp_addr, uint8_t value)
{
    uint8_t data[2];
    data[0] = 0x00;
    data[1] = value;
    dwin_cmd_write(vp_addr, data, 2);
}
