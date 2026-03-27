#include <hf_utils.h>

const int bufferSize = 256;

void printHex(std::vector<uint8_t> &v) {
  for (int i = 0; i < v.size(); ++i) {
    std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
              << (v[i] & 0xff) << " ";
  }
  return;
}

uint16_t CRC16_MODBUS(std::vector<uint8_t> v, uint8_t len) {
  uint16_t CRCin = 0xffff;
  uint16_t CRCret = 0;
  for (int i = 0; i < len; i++) {
    CRCin = v[i] ^ CRCin;
    for (int j = 0; j < 8; j++) {
      if (CRCin & 0x01) {
        CRCin = (CRCin >> 1) ^ 0xa001; // 0xa001是由0x8005高低位转换所得
      } else {
        CRCin = CRCin >> 1;
      }
    }
  }
  CRCret = (CRCin >> 8) | (CRCin << 8); // 低位在前
  return CRCret;
}
