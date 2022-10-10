#include <Arduino.h>
#include <M5Unified.h>
#include <esp_spi_flash.h>

#include "firmware.h"

std::uint32_t _crc_table[256];

static constexpr std::uint8_t CMD_UPDATE_BEGIN = 0xF0;
static constexpr std::uint8_t CMD_UPDATE_DATA  = 0xF1;
static constexpr std::uint8_t CMD_UPDATE_END   = 0xF2;
static constexpr std::uint8_t CMD_RESET        = 0xFF;
static constexpr std::uint8_t UPDATE_RESULT_BROKEN = 0x01;
static constexpr std::uint8_t UPDATE_RESULT_ERROR  = 0x00;
static constexpr std::uint8_t UPDATE_RESULT_OK     = 0xF1;
static constexpr std::uint8_t UPDATE_RESULT_BUSY   = 0xFF;
static constexpr std::uint8_t DEVICE_ID_0 = 0x31;
static constexpr std::uint8_t DEVICE_ID_1 = 0x85;
static constexpr std::uint8_t I2C_ADDR = 0x66;

static void initCRCtable(void)
{
  std::size_t i = 0;
  do
  {
    std::uint32_t c = i << 24;
    std::size_t j = 0;
    do
    {
        c = ( c << 1) ^ ( ( c & 0x80000000) ? 0x04C11DB7 : 0);
    } while ( ++j < 8 );
    _crc_table[i] = c;
  } while ( ++i < 256 );
}

static uint32_t crc32(const std::uint8_t *buf, std::size_t len) {
  std::uint32_t c = 0xffffffff;
  for (std::size_t i = 0; i < len; i++)    {
    c = (c << 8) ^ _crc_table[((c >> 24) ^ buf[i]) & 0xff];
  }
  return c;
}

bool update(void)
{
  M5.Display.fillScreen(TFT_WHITE);
  M5.Display.setCursor(0, 0);
  M5.Display.setFont(&fonts::Font4);
  M5.Display.setTextColor(TFT_BLACK, TFT_WHITE);
  M5.Display.drawString("Unit Kmeter", 0, 0);
  M5.Display.drawString("update", 0, 28);
  M5.Display.fillRect(10, 112, M5.Display.width() - 20, 17, TFT_BLACK);
  M5.Display.fillCircle(                   10, 120, 8, TFT_BLACK);
  M5.Display.fillCircle( M5.Display.width() - 10, 120, 8, TFT_BLACK);

  std::uint8_t readbuf[8] = { 0 };
  std::size_t length = sizeof(firmware);
  std::size_t block = (length + SPI_FLASH_SEC_SIZE - 1) / SPI_FLASH_SEC_SIZE;

  /// ファームウェア更新コマンド列
  std::uint8_t data[16] = { CMD_UPDATE_BEGIN, DEVICE_ID_0, DEVICE_ID_1, CMD_UPDATE_BEGIN };
  data[4] = length >> 24;
  data[5] = length >> 16;
  data[6] = length >>  8;
  data[7] = length >>  0;

  if (!M5.Ex_I2C.start(I2C_ADDR, false, 400000)
   || !M5.Ex_I2C.write(data, 8)
   || !M5.Ex_I2C.stop())
  {
    return false;
  }

  delay(500);
  data[3] = data[0] = CMD_UPDATE_DATA;

  /// セクタブロック単位(4096Byte) でデータ送信を繰り返す
  for (std::size_t b = 0; b < block; ++b)
  {
    M5.Display.fillCircle( 10 + (M5.Display.width() - 20) * b / block, 120, 4, TFT_GREEN );
    if (!M5.Display.displayBusy())
    {
      M5.Display.display();
    }

    auto len = std::min<std::size_t>(SPI_FLASH_SEC_SIZE, length);
    auto crc = crc32(&firmware[b * SPI_FLASH_SEC_SIZE], len);
    data[4] = crc >> 24;  /// 送信するデータのCRC32
    data[5] = crc >> 16;
    data[6] = crc >>  8;
    data[7] = crc >>  0;

    Serial.printf("block %d :", b);
    /// ヘッダおよびデータブロック送信
    if (!M5.Ex_I2C.start(I2C_ADDR, false, 400000)
     || !M5.Ex_I2C.write(data, 8)
     || !M5.Ex_I2C.write(&firmware[b * SPI_FLASH_SEC_SIZE], len)
     || !M5.Ex_I2C.stop())
    // if (lgfx::i2c::beginTransaction(cfg.i2c_port, cfg.i2c_addr, 400000).has_error()
    // || lgfx::i2c::writeBytes(cfg.i2c_port, data, 8).has_error()
    // || lgfx::i2c::writeBytes(cfg.i2c_port, &firmware[b * SPI_FLASH_SEC_SIZE], len).has_error()
    // || lgfx::i2c::endTransaction(cfg.i2c_port).has_error())
    {
      Serial.println("fail");
      return false;
    }
    Serial.println("ok");
    /// ビジーチェック
    int retry = 500;
    do
    {
      delay(10);
      readbuf[0] = UPDATE_RESULT_BUSY;
      if (!M5.Ex_I2C.start(I2C_ADDR, true, 400000)
       || !M5.Ex_I2C.read(readbuf, 1)
       || !M5.Ex_I2C.stop())
      {
        break;
      }
    } while (UPDATE_RESULT_OK != readbuf[0] && --retry);
    if (readbuf[0] != UPDATE_RESULT_OK)
    {
      Serial.printf("fail:%02x\r\n", readbuf[0]);
      return false;
    }
    length -= len;
  }

  data[3] = data[0] = CMD_UPDATE_END;

  M5.Ex_I2C.stop();
  // lgfx::i2c::endTransaction(cfg.i2c_port);

  if (!M5.Ex_I2C.start(I2C_ADDR, false, 400000)
   || !M5.Ex_I2C.write(data, 4)
   || !M5.Ex_I2C.stop())
  {
    return false;
  }

  return true;
}

bool searchUnit(void)
{
  // return (M5.Ex_I2C.start(I2C_ADDR, false, 400000) && M5.Ex_I2C.stop());

  bool res = false;
  std::uint8_t buf[4] = { 0x04 };
  if (M5.Ex_I2C.start(I2C_ADDR, false, 400000)
   && M5.Ex_I2C.write(buf, 1)
   && M5.Ex_I2C.restart(I2C_ADDR, true, 400000)
   && M5.Ex_I2C.read(buf, 4)
  )
  {
    Serial.printf("found : device_id : %02x %02x %02x %02x\r\n", buf[0], buf[1], buf[2], buf[3]);
    if (buf[0] == DEVICE_ID_0
     && buf[1] == DEVICE_ID_1)
    {
      if (buf[2] != MAJOR_VER
       || buf[3] != MINOR_VER)
      {
        res = true;
      }
      else
      {
        Serial.println("updated.");
        delay(512);
      }
    }
  }
  M5.Ex_I2C.stop();
  // if (display2.init(26, 32)) return true; // ATOM
  // if (display2.init( 4, 13)) return true; // TimerCam

  return res;
}

void setup(void)
{
  Serial.begin(115200);

  M5.begin();
  M5.Power.setExtPower(true);
  M5.Ex_I2C.begin();
  M5.Display.setEpdMode(lgfx::epd_mode_t::epd_fastest);

  initCRCtable();

  M5.Display.println("search Unit Kmeter.");
  Serial.println("search Unit Kmeter.");
}

void loop(void)
{
  while (!searchUnit())
  {
    delay(256);
  }

  if (update())
  {
    M5.Display.drawString("success", 0, 56);
    Serial.println("success");
  }
  else
  {
    M5.Display.drawString("fail", 0, 56);
    Serial.println("fail");
  }
  delay(8192);
}
