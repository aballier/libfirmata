#include <errno.h>

extern "C" {
#include "firmata/libfirmata.h"
}

#include "firmata/libfirmatapp.h"

FirmataConn::FirmataConn(const char* devname, int baudrate) :
    c(firmata_open(devname, baudrate))
{
    if(!c) throw errno;
}

FirmataConn::~FirmataConn() { firmata_close(c); }

#define WRAP_CXX0(type, f) type FirmataConn:: f ( ) { return firmata_##f (c); }
#define WRAP_CXX1(type, f, d1, v1) type FirmataConn:: f (d1) { return firmata_##f (c, v1); }
#define WRAP_CXX2(type, f, d1, v1, d2, v2) type FirmataConn:: f (d1, d2) { return firmata_##f (c, v1, v2); }
#define WRAP_CXX3(type, f, d1, v1, d2, v2, d3, v3) type FirmataConn:: f (d1, d2, d3) { return firmata_##f (c, v1, v2, v3); }
#define WRAP_CXX4(type, f, d1, v1, d2, v2, d3, v3, d4, v4) type FirmataConn:: f (d1, d2, d3, d4) { return firmata_##f (c, v1, v2, v3, v4); }
#define WRAP_CXX5(type, f, d1, v1, d2, v2, d3, v3, d4, v4, d5, v5) type FirmataConn:: f (d1, d2, d3, d4, d5) { return firmata_##f (c, v1, v2, v3, v4, v5); }

WRAP_CXX0(const struct firmata_global_data*, get_global_state);
WRAP_CXX1(int , put_global_state     , const struct firmata_global_data *s, s);
WRAP_CXX2(int , add_callback         , int code, code, void (*cb)(void* arg), cb);
WRAP_CXX2(void, del_callback         , int code, code, void (*cb)(void* arg), cb);
WRAP_CXX0(void, get_protocol_version   );
WRAP_CXX0(void, get_firmware_version   );
WRAP_CXX0(void, analog_mapping_query   );
WRAP_CXX0(void, capability_query       );
WRAP_CXX1(void, pin_state_query        , uint8_t pin, pin);
WRAP_CXX2(void, report_analog_channel  , uint8_t chan, chan, uint8_t val, val);
WRAP_CXX2(void, report_digital_port    , uint8_t port, port, uint8_t val, val);
WRAP_CXX2(void, set_pin_mode           , uint8_t pin, pin, uint8_t mode, mode);
WRAP_CXX2(int , set_pin_value          , uint8_t pin, pin, uint32_t value, value);
WRAP_CXX1(void, set_sampling_interval  , uint16_t ms, ms);
WRAP_CXX5(int , i2c_read_write         , uint16_t addr, addr, uint8_t command, command, int tenbits, tenbits, const uint8_t *data, data, ssize_t data_len, data_len);
WRAP_CXX1(void, i2c_config             , uint16_t delay, delay);
WRAP_CXX3(void, encoder_attach         , uint8_t encoder, encoder, uint8_t pin_a, pin_a, uint8_t pin_b, pin_b);
WRAP_CXX1(void, encoder_report_position, uint8_t encoder, encoder);
WRAP_CXX0(void, encoder_report_positions);
WRAP_CXX1(void, encoder_reset_position, uint8_t encoder, encoder);
WRAP_CXX1(void, encoder_set_reporting, uint8_t value, value);
WRAP_CXX1(void, encoder_detach, uint8_t encoder, encoder);
WRAP_CXX3(void, set_servo_config, uint8_t pin, pin, uint16_t minPulse, minPulse, uint16_t maxPulse, maxPulse);
