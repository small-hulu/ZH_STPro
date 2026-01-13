#include "serial_frame.h"
#include <string.h>

/* ================= 内部校验函数 ================= */

static uint8_t serial_frame_calc_checksum(const uint8_t *buf, uint16_t len)
{
    uint8_t cs = 0;
    for (uint16_t i = 0; i < len + 1; i++)
    {
        cs ^= buf[i];   
    }
    return cs;
}


serial_frame_ret_t serial_frame_parse(
    const uint8_t *raw,
    uint16_t raw_len,
    serial_frame_t *out)
{
    if (raw == NULL || out == NULL)
        return SERIAL_FRAME_ERR_FORMAT;

    if (raw_len < 5)
        return SERIAL_FRAME_ERR_LEN;

    if (raw[0] != SERIAL_FRAME_HEAD)
        return SERIAL_FRAME_ERR_HEAD;

    if (raw[raw_len - 1] != SERIAL_FRAME_TAIL)
        return SERIAL_FRAME_ERR_TAIL;

    uint8_t len = raw[1];   

    if (len < 1 || len > (SERIAL_FRAME_MAX_DATA_LEN + 1))
        return SERIAL_FRAME_ERR_LEN;

    if (raw_len != (uint16_t)(len + 4))
        return SERIAL_FRAME_ERR_LEN;

    uint8_t calc = serial_frame_calc_checksum(&raw[1], len);
    uint8_t recv = raw[raw_len - 2];   // checksum

    if (calc != recv)
        return SERIAL_FRAME_ERR_CHECKSUM;

    out->cmd = raw[2];
    out->data_len = len - 1;

    if (out->data_len > 0)
    {
        memcpy(out->data, &raw[3], out->data_len);
    }

    return SERIAL_FRAME_OK;
}

uint16_t serial_frame_build(
    uint8_t cmd,
    const uint8_t *data,
    uint8_t data_len,
    uint8_t *out_buf,
    uint16_t out_buf_size)
{
    uint8_t len = 1 + data_len;  // CMD + DATA
    uint16_t frame_len = len + 4;

    if (out_buf == NULL || out_buf_size < frame_len)
        return 0;

    out_buf[0] = SERIAL_FRAME_HEAD;   // 0xAA
    out_buf[1] = len;
    out_buf[2] = cmd;

    if (data_len > 0 && data != NULL)
    {
        memcpy(&out_buf[3], data, data_len);
    }

    uint8_t chk = 0;
    chk += cmd;
    for (uint8_t i = 0; i < data_len; i++)
    {
        chk += data[i];
    }

    out_buf[3 + data_len] = chk;
    out_buf[4 + data_len] = SERIAL_FRAME_TAIL; // 0x0D

    return frame_len;
}


