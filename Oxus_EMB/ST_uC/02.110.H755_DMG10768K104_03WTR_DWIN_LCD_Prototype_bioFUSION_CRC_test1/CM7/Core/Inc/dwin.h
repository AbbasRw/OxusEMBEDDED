/*
 * dwin.h
 */
#ifndef INC_DWIN_H_
#define INC_DWIN_H_

#include <stdint.h>
#include "main.h"

// ============================================
// USER CONFIGURATION
// ============================================

//#define DWIN_UART_HANDLE            huart1
//#define DWIN_UART_HANDLE		huart1
#define DWIN_USE_DMA                0
#define DWIN_DEBUG_ENABLE           1

// BUFFER SIZES
#define DWIN_MAX_TX_MESSAGE_LENGTH  256
#define DWIN_RX_LIST_SIZE           10

// RX STATES
typedef enum {
    RX_STATE_WAIT_HEADER1,
    RX_STATE_WAIT_HEADER2,
    RX_STATE_WAIT_LENGTH,
    RX_STATE_WAIT_DATA
} RX_State_t;

// RX MESSAGE TYPES
typedef enum {
    RX_TYPE_NONE,
    RX_TYPE_OK,
    RX_TYPE_POWER_ON,
    RX_TYPE_TOUCH,
    RX_TYPE_READ_RESPONSE
} RX_Type_t;

// RX PARSER
typedef struct {
    RX_State_t state;
    uint8_t buffer[64];
    uint8_t index;
    uint8_t expected_len;
} RX_Parser_t;

// RX DATA
typedef struct {
    RX_Type_t type;
    uint16_t vp_addr;
    uint8_t data_len;
    uint8_t data[32];
} DWIN_RX_t;

// RX LIST
typedef struct {
    DWIN_RX_t list[DWIN_RX_LIST_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} DWIN_RX_List_t;

// STATS
typedef struct {
    uint16_t total_sent;
    uint16_t total_received;
} DWIN_Stats_t;

// DWIN STRUCT
typedef struct {
    uint8_t tx_arr[DWIN_MAX_TX_MESSAGE_LENGTH];
    uint8_t tx_len;
    volatile uint8_t rx_byte;

    RX_Parser_t rx_parser;
    DWIN_RX_List_t rx_list;
} DWIN_t;

// GLOBAL
extern DWIN_t g_dwin;
extern DWIN_Stats_t g_dwin_stats;

// CRC
//uint16_t dwin_crc16(const uint8_t *data, uint16_t length);

// INIT
void dwin_init(void);

// RX
void dwin_rx_callback(void);
uint8_t dwin_rx_available(void);           // Liste boş mu?
DWIN_RX_t* dwin_rx_get(void);              // Listeden al

// STATS
void dwin_check_health(void);

// SYSTEM
void dwin_system_reset(void);

// PAGE
void dwin_set_page(uint8_t page_ID);

// ICON
void dwin_set_icon(uint16_t vp_address, uint8_t icon_id);

// BUTTON
void dwin_return_keycode_enable(uint8_t page_id, uint8_t control_id);
void dwin_return_keycode_disable(uint8_t page_id, uint8_t control_id);

// DATA VARIABLE
void dwin_data_variable_show(uint16_t sp_addr, uint16_t vp_addr);
void dwin_data_variable_hide(uint16_t sp_addr);
void dwin_data_variable_set_integer_digits(uint16_t sp_addr, uint8_t digits);
void dwin_data_variable_set_decimal_digits(uint16_t sp_addr, uint8_t digits);
void dwin_data_variable_write_float(uint16_t vp_addr, float value);


// PROGRESS BAR
void dwin_set_progressbar(uint16_t vp_addr, uint8_t value);

void dwin_cmd_write(uint16_t vp_addr, uint8_t *data, uint8_t data_len);

#endif
