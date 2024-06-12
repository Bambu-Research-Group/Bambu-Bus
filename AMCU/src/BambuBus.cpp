#include "BambuBus.h"
#include "CRC16.h"
#include "CRC8.h"
CRC16 crc_16;
CRC8 crc_8;

uint8_t BambuBus_data_buf[1000];
int BambuBus_have_data = 0;

serial_t use_Serial;
struct _filament
{
    char ID[8] = "GFG00";
    uint32_t color = 0xFFFFFFFF;
    int16_t temperature_min = 220;
    int16_t temperature_max = 240;
    char name[20] = "PETG";
};
#define max_filament_num 4
_filament filament[max_filament_num];

bool TX_IDLE = true;
int TX_IRQ(serial_t *obj)
{
    digitalWrite(BambuBus_pin_de, 0);
    TX_IDLE = true;
    return 0;
}

void send_uart(const unsigned char *data, uint16_t length)
{
    TX_IDLE = false;
    digitalWrite(BambuBus_pin_de, 1);

    /* Must disable interrupt to prevent handle lock contention */
    HAL_NVIC_DisableIRQ(use_Serial.irq);
    /* The following function will enable UART_IT_TXE and error interrupts */
    HAL_UART_Transmit_IT(&use_Serial.handle, data, length);
    /* Enable interrupt */
    HAL_NVIC_EnableIRQ(use_Serial.irq);
}

unsigned char _RX_IRQ_data;
void RX_IRQ(serial_t *obj)
{
    static int _index = 0;
    static int length = 500;
    static uint8_t data_length_index;
    static uint8_t data_CRC8_index;
    static CRC8 crcx(0x39, 0x66, 0x00, false, false);
    unsigned char data = _RX_IRQ_data;

    /* Must disable interrupt to prevent handle lock contention */
    HAL_NVIC_DisableIRQ(use_Serial.irq);

    HAL_UART_Receive_IT(&use_Serial.handle, &_RX_IRQ_data, 1);

    /* Enable interrupt */
    HAL_NVIC_EnableIRQ(use_Serial.irq);

    if (_index == 0)
    {
        if (data == 0x3D)
        {
            BambuBus_data_buf[0] = 0x3D;
            crcx.restart();
            crcx.add(0x3D);
            data_length_index = 4;
            length = data_CRC8_index = 6;
            _index = 1;
        }
        return;
    }
    else
    {
        BambuBus_data_buf[_index] = data;
        if (_index == 1)
        {
            if (data & 0x80)
            {
                data_length_index = 2;
                data_CRC8_index = 3;
            }
            else
            {
                data_length_index = 4;
                data_CRC8_index = 6;
            }
        }
        if (_index == data_length_index)
        {
            length = data;
        }
        if (_index < data_CRC8_index)
        {
            crcx.add(data);
        }
        else if (_index == data_CRC8_index)
        {
            if (data != crcx.calc())
            {
                _index = 0;
                return;
            }
        }
        ++_index;
        if (_index >= length)
        {
            _index = 0;
            BambuBus_have_data = length;
        }
        if (_index >= 999)
        {
            _index = 0;
        }
    }
}

uint8_t _rx_bufx[64];
uint8_t _tx_bufx[64];

void UART_Init(uint32_t baudrate)
{
    use_Serial.uart = BambuBus_uart;
    use_Serial.pin_tx = digitalPinToPinName(BambuBus_pin_tx);
    use_Serial.pin_rx = digitalPinToPinName(BambuBus_pin_rx);
    use_Serial.pin_rts = NC;
    use_Serial.pin_cts = NC;
    use_Serial.rx_buff = _rx_bufx;
    use_Serial.rx_head = 0;
    use_Serial.rx_tail = 0;
    use_Serial.tx_buff = _tx_bufx;
    use_Serial.tx_head = 0;
    use_Serial.tx_tail = 0;
    use_Serial.tx_callback = TX_IRQ;
    use_Serial.rx_callback = RX_IRQ;
    uart_init(&use_Serial, baudrate, UART_WORDLENGTH_9B, UART_PARITY_EVEN, UART_STOPBITS_1);
    /* Must disable interrupt to prevent handle lock contention */
    HAL_NVIC_DisableIRQ(use_Serial.irq);
    HAL_UART_Receive_IT(&use_Serial.handle, &_RX_IRQ_data, 1);
    HAL_NVIC_EnableIRQ(use_Serial.irq);
}

void BambuBus_init(bool if_lowspeed_debug)
{
    crc_8.reset(0x39, 0x66, 0, false, false);
    crc_16.reset(0x1021, 0x913D, 0, false, false);
    pinMode(BambuBus_pin_de, OUTPUT);
    if (if_lowspeed_debug)
        UART_Init(115200);
    else
        UART_Init(1228800);

    filament[0].color = 0xFF0000FF;
    filament[1].color = 0x00FF000FF;
    filament[2].color = 0x0000FFFF;
    filament[3].color = 0x888888FF;
}

bool packge_check_crc16(uint8_t *data, int data_length)
{
    crc_16.restart();
    data_length -= 2;
    for (auto i = 0; i < data_length; i++)
    {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    if ((data[(data_length)] == (num & 0xFF)) && (data[(data_length + 1)] == ((num >> 8) & 0xFF)))
        return true;
    return false;
}

void packge_send_with_crc(uint8_t *data, int data_length)
{

    crc_8.restart();
    if (data[1] & 0x80)
    {
        for (auto i = 0; i < 3; i++)
        {
            crc_8.add(data[i]);
        }
        data[3] = crc_8.calc();
    }
    else
    {
        for (auto i = 0; i < 6; i++)
        {
            crc_8.add(data[i]);
        }
        data[6] = crc_8.calc();
    }
    crc_16.restart();
    data_length -= 2;
    for (auto i = 0; i < data_length; i++)
    {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    data[(data_length)] = num & 0xFF;
    data[(data_length + 1)] = num >> 8;
    data_length += 2;
    send_uart(data, data_length);
}

enum packge_type
{
    BambuBus_packge_Cxx,
    BambuBus_packge_Dxx,
    BambuBus_packge_Fxx,
    BambuBus_packge_REQx6,
    BambuBus_packge_NFC_detect,
    BambuBus_packge_Set_filament,
    BambuBus_packge_X05_MC,
    BambuBus_packge_X05_AP,
    BambuBus_packge_X05_AP2,
    BambuBus_packge_PE,
    BambuBus_packge_ETC,
    BambuBus_packge_ERROR,
    __BambuBus_packge_packge_type_size
};

unsigned char buf_X[100];
packge_type get_packge_type(unsigned char *buf, int length)
{
    if (packge_check_crc16(buf, length) == false)
    {
        return BambuBus_packge_ERROR;
    }
    if (buf[1] == 0xC5)
    {

        switch (buf[4])
        {
        case 0x03:
            return BambuBus_packge_Cxx;
        case 0x04:
            return BambuBus_packge_Dxx;
        case 0x05:
            return BambuBus_packge_Fxx;
        case 0x06:
            return BambuBus_packge_REQx6;
        case 0x07:
            return BambuBus_packge_NFC_detect;
        case 0x08:
            return BambuBus_packge_Set_filament;
        case 0x20:
            return BambuBus_packge_PE;
        default:
            return BambuBus_packge_ETC;
        }
    }
    else
    {
        if (buf[8] == 0x12)
        {
            switch (buf[10])
            {
            case 0x03:
                return BambuBus_packge_X05_MC;
            case 0x06:
                return BambuBus_packge_X05_AP;
            case 0x09:
                return BambuBus_packge_X05_AP2;
            default:
                return BambuBus_packge_ETC;
            }
        }
        else
        {
            return BambuBus_packge_ETC;
        }
    }
    return BambuBus_packge_ERROR;
}
uint8_t packge_num = 0;
unsigned char Cxx_res[] = {0x3D, 0xE0, 0x2C, 0x1A, 0x03,
                           0x00, 0x00, 0x00, 0xFF, // 0x0C...
                           0x00, 0x00, 0x80, 0xBF,
                           0x00, 0x00, 0x00, 0xC0,
                           0x00, 0xC0, 0x5D, 0xFF,
                           0x00, 0x00, 0x00, 0x00, // 0xFE, 0xFF, 0xFE, 0xFF,
                           0x00, 0x44, 0x00, 0x00,
                           0x10,
                           0xC1, 0xC3, 0xEC, 0xBC,
                           0x01, 0x01, 0x01, 0x01,
                           0x00, 0x00, 0x00, 0x00,
                           0x90, 0xE4};
void send_for_Cxx(unsigned char *buf, int length)
{
    Cxx_res[1] = 0xC0 | (packge_num << 3);
    if (packge_num < 7)
        packge_num++;
    else
        packge_num = 0;
    packge_send_with_crc(Cxx_res, sizeof(Cxx_res));
}

unsigned char filament_flag_on = 0x0F;
unsigned char filament_flag_detected = 0x00;

unsigned char Dxx_res[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x04,
                           0x00, 0x00, 0x00, 0x00,
                           0x04, 0x04, 0x04, 0xFF, // flags
                           0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0xFF, // 0x0C...
                           0x00, 0x00, 0x80, 0xBF,
                           0x00, 0x00, 0x00, 0xC0,
                           0x00, 0xC0, 0x5D, 0xFF,
                           0x00, 0x00, 0x00, 0x00, // 0xFE, 0xFF, 0xFE, 0xFF,
                           0x00, 0x44, 0x00, 0x00,
                           0x10,
                           0xC1, 0xC3, 0xEC, 0xBC,
                           0x01, 0x01, 0x01, 0x01,
                           0x00, 0x00, 0x00, 0x00,
                           0x64, 0x64, 0x64, 0x64,
                           0x90, 0xE4};
unsigned char Dxx_res2[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x04,
                            0x00, 0x00, 0x00, 0x00,
                            0x0C, 0x04, 0x04, 0x03,
                            0x08, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x03, 0x03,
                            0x5F, 0x6E, 0xD7, 0xBE,
                            0x00, 0x00, 0x03, 0x00,
                            0x44, 0x00, 0x44, 0x00,
                            0xFF, 0xFF, 0xFF, 0xFF,
                            0x00, 0x00, 0x44, 0x00,
                            0x50,
                            0xC1, 0xC3, 0xED, 0xE9,
                            0x01, 0x01, 0x01, 0x01,
                            0x00, 0x00, 0x00, 0x00,
                            0x64, 0x64, 0x64, 0x64,
                            0xEC, 0xF0};
bool need_res_for_06 = false;
uint8_t res_for_06_num = 0xFF;
void send_for_Dxx(unsigned char *buf, int length)
{
    if (need_res_for_06)
    {
        Dxx_res2[1] = 0xC0 | (packge_num << 3);
        Dxx_res2[9] = filament_flag_on;
        Dxx_res2[10] = filament_flag_on - filament_flag_detected;
        Dxx_res2[11] = filament_flag_on - filament_flag_detected;
        Dxx_res2[12] = res_for_06_num;
        Dxx_res2[13] = filament_flag_detected;
        packge_send_with_crc(Dxx_res2, sizeof(Dxx_res2));
        need_res_for_06 = false;
    }
    else
    {
        Dxx_res[1] = 0xC0 | (packge_num << 3);
        Dxx_res[9] = filament_flag_on;
        Dxx_res[10] = filament_flag_on - filament_flag_detected;
        Dxx_res[11] = filament_flag_on - filament_flag_detected;
        Dxx_res[12] = buf[9];
        Dxx_res[13] = filament_flag_detected;
        packge_send_with_crc(Dxx_res, sizeof(Dxx_res));
    }
    if (packge_num < 7)
        packge_num++;
    else
        packge_num = 0;
}
unsigned char REQx6_res[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x06,
                             0x00, 0x00, 0x00, 0x00,
                             0x04, 0x04, 0x04, 0xFF, // flags
                             0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0xFF, // 0x0C...
                             0x00, 0x00, 0x80, 0xBF,
                             0x00, 0x00, 0x00, 0xC0,
                             0x00, 0xC0, 0x5D, 0xFF,
                             0x00, 0x00, 0x00, 0x00, // 0xFE, 0xFF, 0xFE, 0xFF,
                             0x00, 0x44, 0x00, 0x00,
                             0x10,
                             0xC1, 0xC3, 0xEC, 0xBC,
                             0x01, 0x01, 0x01, 0x01,
                             0x00, 0x00, 0x00, 0x00,
                             0x64, 0x64, 0x64, 0x64,
                             0x90, 0xE4};
void send_for_REQx6(unsigned char *buf, int length)
{
    REQx6_res[1] = 0xC0 | (packge_num << 3);
    res_for_06_num = buf[7];
    REQx6_res[9] = filament_flag_on;
    REQx6_res[10] = filament_flag_on - filament_flag_detected;
    REQx6_res[11] = filament_flag_on - filament_flag_detected;
    Dxx_res2[12] = res_for_06_num;
    Dxx_res2[12] = res_for_06_num;
    packge_send_with_crc(REQx6_res, sizeof(REQx6_res));
    need_res_for_06 = true;
    if (packge_num < 7)
        packge_num++;
    else
        packge_num = 0;
}

uint64_t last_detect = 0;
void NFC_detect_run()
{
    /*uint64_t time = GetTick();
    return;
    if (time > last_detect + 3000)
    {
        filament_flag_detected = 0;
    }*/
}

unsigned char F10_res[] = {0x3D, 0xC0, 0x1D, 0xB4, 0x05,
                           0x01, 0x00, 0x20, 0x9B,
                           0x31, 0x33, 0x34, 0x36,
                           0x35, 0x02, 0x00, 0x37,
                           0x39, 0x33, 0x38,
                           0xFF, 0xFF, 0xFF, 0xFF,
                           0x00, 0x00, 0x00,
                           0x00, 0x00};
void send_for_Fxx(unsigned char *buf, int length)
{
    if ((buf[5] == 0x01) && (buf[6] == 0x00))
    {
        memcpy(F10_res + 4, buf + 4, 23);
        packge_send_with_crc(F10_res, sizeof(F10_res));
    }
}

unsigned char NFC_detect_res[] = {0x3D, 0xC0, 0x0D, 0x6F, 0x07, 0x00, 0x03, 0x01, 0x00, 0x00, 0x00, 0xFC, 0xE8};
void send_for_NFC_detect(unsigned char *buf, int length)
{
    // last_detect = GetTick();
    // filament_flag_detected = 1 << buf[6];
    NFC_detect_res[6] = buf[6];
    NFC_detect_res[7] = buf[7];
    packge_send_with_crc(NFC_detect_res, sizeof(NFC_detect_res));
}

unsigned char X05_MC_res[] = {0x3D, 0x00, 0x00, 0x00, 0x15,
                              0x00, 0xF4, 0x00, 0x03, 0x00, 0x12, 0x1A, 0x02,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x57};
void send_for_X05_MC(unsigned char *buf, int length)
{
    packge_send_with_crc(X05_MC_res, sizeof(X05_MC_res));
}
unsigned char X05_MC_AP_Read_filament_res[] =
    {
        0x3D, 0x00, 0x00, 0x00, 0x92, 0x00, 0x2B,
        0x00, 0x06, 0x00, 0x12,
        0x11,
        0x02, 0x00,
        0x02, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x47, 0x46, 0x47, 0x39, 0x39, 0x00, 0x00, 0x00,
        0x50, 0x45, 0x54, 0x47, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x16, 0x16, 0x16, 0xFF,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x0E, 0x01,
        0xE6, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x2C, 0x71};
void send_for_X05_AP(unsigned char *buf, int length)
{
    if (buf[11] == 0x11)
    {
        uint8_t n = buf[14];

        X05_MC_AP_Read_filament_res[14] = n;
        uint32_t x = filament[n].color;
        X05_MC_AP_Read_filament_res[75] = x;
        X05_MC_AP_Read_filament_res[74] = x >> 8;
        X05_MC_AP_Read_filament_res[73] = x >> 16;
        X05_MC_AP_Read_filament_res[72] = x >> 24;

        x = filament[n].temperature_min;
        X05_MC_AP_Read_filament_res[92] = x;
        X05_MC_AP_Read_filament_res[93] = x >> 8;
        x = filament[n].temperature_min;
        X05_MC_AP_Read_filament_res[94] = x;
        X05_MC_AP_Read_filament_res[95] = x >> 8;

        memcpy(X05_MC_AP_Read_filament_res + 32, filament[n].ID, sizeof(filament[n].ID));
        memcpy(X05_MC_AP_Read_filament_res + 40, filament[n].name, sizeof(filament[n].name));
        packge_send_with_crc(X05_MC_AP_Read_filament_res, sizeof(X05_MC_AP_Read_filament_res));
    }
}

unsigned char X05_AP2_res_03[] = {0x3D, 0x00, 0x6A, 0x00, 0x48, 0x00, 0xC0, 0x00,
                                  0x09, 0x00, 0x12, 0x03, 0x01, 0x5C, 0x07, 0x00,
                                  0x00, 0x41, 0x4D, 0x53, 0x5F, 0x46, 0x31, 0x30,
                                  0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBD, 0x44};
unsigned char X05_AP2_res_02[] = {0x3D, 0x00, 0xB3, 0x00, 0x51, 0x00, 0x28, 0x00, 0x09, 0x00, 0x12, 0x02, 0x04,
                                  0x0F, 0x30, 0x33, 0x43, 0x31, 0x32, 0x41, 0x33,
                                  0x43, 0x30, 0x34, 0x30, 0x30, 0x35, 0x32, 0x39,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x9B, 0x31, 0x33, 0x34, 0x36, 0x35, 0x02,
                                  0x00, 0x37, 0x39, 0x33, 0x38, 0xFF, 0xFF, 0xFF,
                                  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                  0xFF, 0xBB, 0x44, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                  0xFF, 0x00, 0xDE, 0xEF};
void send_for_X05_AP2(unsigned char *buf, int length)
{
    switch (buf[11])
    {
    case 0x02:
        X05_AP2_res_02[2] = buf[2];
        packge_send_with_crc(X05_AP2_res_02, sizeof(X05_AP2_res_02));
        break;
    case 0x03:
        X05_AP2_res_03[2] = buf[2];
        packge_send_with_crc(X05_AP2_res_03, sizeof(X05_AP2_res_03));
        break;
    default:
        break;
    }
}
unsigned char s = 0x01;

unsigned char Set_filament_res[] = {0x3D, 0xC0, 0x08, 0xB2, 0x08, 0x60, 0xB4, 0x04};
void send_for_Set_filament(unsigned char *buf, int length)
{
    memcpy(filament[buf[5]].ID, buf + 7, sizeof(filament[buf[5]].ID));
    memcpy(filament[buf[5]].name, buf + 23, sizeof(filament[buf[5]].name));
    uint32_t x = buf[15];
    x <<= 8;
    x |= buf[16];
    x <<= 8;
    x |= buf[17];
    x <<= 8;
    x |= buf[18];
    filament[buf[5]].color = x;
    x = buf[20];
    x <<= 8;
    x |= buf[19];
    filament[buf[5]].temperature_min = x;
    x = buf[22];
    x <<= 8;
    x |= buf[21];
    filament[buf[5]].temperature_max = x;
    packge_send_with_crc(Set_filament_res, sizeof(Set_filament_res));
    // forwarding_Serial.print(filament[buf[5]].ID);
}

void BambuBus_run()
{
    if (BambuBus_have_data)
    {
        int data_length = BambuBus_have_data;
        uint8_t buf_X[1000];
        digitalToggle(13);
        BambuBus_have_data = 0;
        memcpy(buf_X, BambuBus_data_buf, data_length);
        packge_type type_n = get_packge_type(buf_X, data_length);

        delayMicroseconds(100);
        switch (type_n)
        {
        case BambuBus_packge_PE:
            break;
        case BambuBus_packge_Cxx:
            send_for_Cxx(buf_X, data_length);
            break;
        case BambuBus_packge_Dxx:
            send_for_Dxx(buf_X, data_length);
            break;
        case BambuBus_packge_Fxx:
            send_for_Fxx(buf_X, data_length);
            break;
        case BambuBus_packge_REQx6:
            send_for_REQx6(buf_X, data_length);
            break;
        case BambuBus_packge_X05_MC:
            send_for_X05_MC(buf_X, data_length);
            break;
        case BambuBus_packge_X05_AP:
            send_for_X05_AP(buf_X, data_length);
            break;
        case BambuBus_packge_X05_AP2:
            send_for_X05_AP2(buf_X, data_length);
            break;
        case BambuBus_packge_NFC_detect:
            send_for_NFC_detect(buf_X, data_length);
            break;
        case BambuBus_packge_Set_filament:
            send_for_Set_filament(buf_X, data_length);
            break;
        default:

            break;
        }
    }
    // HAL_UART_Transmit(&use_Serial.handle,&s,1,1000);

    NFC_detect_run();
}
#ifdef BambuBus_use_forwarding_Serial

void forwarding_Serial_hex(unsigned char *buf, int length)
{
    char x[10];
    for (int i = 0; i < length; ++i)
    {
        sprintf(x, "%02X ", buf[i]);
        forwarding_Serial.print(x);
    }
    forwarding_Serial.write('\n');
}

void BambuBus_run_forward()
{
    if (BambuBus_have_data)
    {
        int data_length = BambuBus_have_data;
        uint8_t buf_X[1000];
        
        BambuBus_have_data = 0;
        memcpy(buf_X, BambuBus_data_buf, data_length);

        digitalToggle(13);
        packge_type type_n = get_packge_type(buf_X, data_length);

        switch (type_n)
        {
        case BambuBus_packge_PE:
            forwarding_Serial.println("E");
            break;
        case BambuBus_packge_Fxx:
            forwarding_Serial.println("Fxx");
            break;
        
        case BambuBus_packge_Dxx:
            if((buf_X[7]==0xBF)||(buf_X[7]==0x3F))
            {
                digitalWrite(15,1);
            }
            else
                digitalWrite(15,0);
        case BambuBus_packge_Cxx:
        case BambuBus_packge_REQx6:
        case BambuBus_packge_X05_MC:
        case BambuBus_packge_X05_AP2:
        case BambuBus_packge_NFC_detect:
        case BambuBus_packge_Set_filament:

        default:
            forwarding_Serial_hex(buf_X, data_length);
            break;
        }
    }
    // HAL_UART_Transmit(&use_Serial.handle,&s,1,1000);
}
#endif