// SPDX-License-Identifier: MIT
//
// HIL sketch (.claude/skills/hil-contract/SKILL.md) and the restoration image:
// the board's resting firmware between runs, with no flash write path and no
// SoftDevice or InternalFileSystem use. One command per line on Serial (the
// DUT's USB CDC, served by the core's TinyUSB task), one reply line each
// carrying the command's nonce; anything else is answered REJECT why=<reason>.
//
// clang-format off
//   INFO <nonce>        HIL_INFO <nonce> stack=tinyusb chip=<nrf52833|nrf52840> f_cpu=<hz>
//   ECHO <nonce> <len>  then <len> raw bytes, each echoed back as it arrives, then
//                         HIL_RESULT <nonce> echo PASS len=<len> crc=<crc32>
// clang-format on

#include <Adafruit_TinyUSB.h> // Serial, the TinyUSB library's CDC
#include <Arduino.h>

#if defined(NRF52840_XXAA)
#define CHIP "nrf52840"
#elif defined(NRF52833_XXAA)
#define CHIP "nrf52833"
#else
#error "a HIL sketch needs USB: nRF52833 or nRF52840"
#endif

#define ECHO_MAX 4096u

static char nonce[24];
static uint32_t echo_left, echo_len, echo_crc;

static uint32_t crc32_update(uint32_t crc, uint8_t const *data, uint32_t len) {
  crc = ~crc;
  while (len--) {
    crc ^= *data++;
    for (int k = 0; k < 8; k++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return ~crc;
}

static void handle(char *line) {
  char *cmd = strtok(line, " ");
  char *n = strtok(NULL, " ");
  char *a = strtok(NULL, " ");
  char *extra = strtok(NULL, " ");
  if (!cmd || !n || strlen(n) >= sizeof(nonce) || extra) {
    Serial.println("HIL_RESULT - ? REJECT why=malformed");
    return;
  }
  if (!strcmp(cmd, "INFO") && !a) {
    Serial.printf("HIL_INFO %s stack=tinyusb chip=%s f_cpu=%lu\r\n", n, CHIP,
                  (unsigned long)F_CPU);
    return;
  }
  if (!strcmp(cmd, "ECHO") && a) {
    char *end;
    unsigned long len = strtoul(a, &end, 10);
    if (*end || a[0] == '-' || len == 0 || len > ECHO_MAX) {
      Serial.printf("HIL_RESULT %s echo REJECT why=length\r\n", n);
      return;
    }
    strcpy(nonce, n);
    echo_len = echo_left = len;
    echo_crc = 0;
    return;
  }
  Serial.printf("HIL_RESULT %s ? REJECT why=unknown\r\n", n);
}

void setup() { Serial.begin(115200); }

void loop() {
  static char line[64];
  static size_t n;
  // discarding an overlong line through its newline: its tail is no command
  static bool overlong;
  while (echo_left && Serial.available()) {
    uint8_t buf[64];
    uint32_t got = 0;
    while (got < sizeof(buf) && got < echo_left && Serial.available()) {
      buf[got++] = (uint8_t)Serial.read();
    }
    Serial.write(buf, got);
    echo_crc = crc32_update(echo_crc, buf, got);
    echo_left -= got;
    if (!echo_left) {
      Serial.printf("\r\nHIL_RESULT %s echo PASS len=%lu crc=%08lx\r\n", nonce,
                    (unsigned long)echo_len, (unsigned long)echo_crc);
    }
  }
  while (!echo_left && Serial.available()) {
    int c = Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      line[n] = 0;
      if (overlong) {
        Serial.println("HIL_RESULT - ? REJECT why=overlong");
      } else {
        handle(line);
      }
      n = 0;
      overlong = false;
    } else if (n < sizeof(line) - 1) {
      line[n++] = (char)c;
    } else {
      overlong = true;
    }
  }
}
