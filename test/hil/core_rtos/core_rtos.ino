// SPDX-License-Identifier: MIT
//
// HIL sketch (.claude/skills/hil-contract/SKILL.md): the core's FreeRTOS
// scheduling and time base, read by the host against its own monotonic clock.
// No SoftDevice or InternalFileSystem use. One command per line on Serial (the
// DUT's USB CDC), one reply line each carrying the command's nonce; anything
// else is answered REJECT why=<reason>.
//
// clang-format off
//   INFO <nonce>        HIL_INFO <nonce> stack=tinyusb chip=<nrf52833|nrf52840> f_cpu=<hz>
//   TIME <nonce>        HIL_RESULT <nonce> time PASS ms=<millis()> us=<micros()>
//   DELAY <nonce> <ms>  HIL_RESULT <nonce> delay PASS ms=<ms> us=<micros() across delay(ms)>
//   RTOS <nonce> <ms>   two worker tasks each queue a message every WORK_MS and a
//                       SoftwareTimer ticks every TIMER_MS while loop() drains the
//                       queue for <ms>, then all stop:
//                         HIL_RESULT <nonce> rtos PASS ms=<millis() elapsed>
//                         a=<messages> b=<messages> timer=<ticks>
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

#define DELAY_MAX 5000u
#define WORK_MS 10
#define TIMER_MS 50

static QueueHandle_t queue;
static SoftwareTimer timer;
static volatile uint32_t timer_ticks;
static volatile bool stopping;
static volatile uint8_t workers_done;

static void timer_cb(TimerHandle_t) { timer_ticks++; }

static void worker(void *arg) {
  uint8_t id = (uint8_t)(uintptr_t)arg;
  while (!stopping) {
    xQueueSend(queue, &id, portMAX_DELAY);
    vTaskDelay(ms2tick(WORK_MS));
  }
  taskENTER_CRITICAL();
  workers_done++;
  taskEXIT_CRITICAL();
  vTaskDelete(NULL);
}

// one queued message counted, or false after `wait` with none
static bool take(TickType_t wait, uint32_t count[2]) {
  uint8_t id;
  if (xQueueReceive(queue, &id, wait) != pdTRUE) {
    return false;
  }
  count[id & 1]++;
  return true;
}

// both workers stopped and their queue drained: false when a task could not
// be created
static bool rtos_run(unsigned long ms, uint32_t *elapsed, uint32_t count[2]) {
  stopping = false;
  workers_done = 0;
  timer_ticks = 0;
  count[0] = count[1] = 0;
  uint32_t t0 = millis();
  int started = 0;
  for (uintptr_t id = 0; id < 2; id++) {
    if (xTaskCreate(worker, id ? "hil_b" : "hil_a", 256, (void *)id,
                    TASK_PRIO_LOW, NULL) == pdPASS) {
      started++;
    }
  }
  timer.start();
  while (millis() - t0 < ms) {
    take(ms2tick(5), count);
  }
  timer.stop();
  *elapsed = millis() - t0;
  stopping = true;
  while (workers_done < started) {
    take(ms2tick(5), count);
  }
  while (take(0, count)) {
  }
  return started == 2;
}

// 1 to DELAY_MAX ms, or 0 after answering REJECT
static unsigned long parse_ms(char *a, char *n, char const *op) {
  char *end;
  unsigned long ms = strtoul(a, &end, 10);
  if (*end || a[0] == '-' || ms == 0 || ms > DELAY_MAX) {
    Serial.printf("HIL_RESULT %s %s REJECT why=length\r\n", n, op);
    return 0;
  }
  return ms;
}

static void handle(char *line) {
  char *cmd = strtok(line, " ");
  char *n = strtok(NULL, " ");
  char *a = strtok(NULL, " ");
  char *extra = strtok(NULL, " ");
  if (!cmd || !n || extra) {
    Serial.println("HIL_RESULT - ? REJECT why=malformed");
    return;
  }
  if (!strcmp(cmd, "INFO") && !a) {
    Serial.printf("HIL_INFO %s stack=tinyusb chip=%s f_cpu=%lu\r\n", n, CHIP,
                  (unsigned long)F_CPU);
    return;
  }
  if (!strcmp(cmd, "TIME") && !a) {
    uint32_t ms = millis();
    uint32_t us = micros();
    Serial.printf("HIL_RESULT %s time PASS ms=%lu us=%lu\r\n", n,
                  (unsigned long)ms, (unsigned long)us);
    return;
  }
  if (!strcmp(cmd, "DELAY") && a) {
    unsigned long ms = parse_ms(a, n, "delay");
    if (!ms) {
      return;
    }
    uint32_t t0 = micros();
    delay(ms);
    uint32_t t1 = micros();
    Serial.printf("HIL_RESULT %s delay PASS ms=%lu us=%lu\r\n", n, ms,
                  (unsigned long)(t1 - t0));
    return;
  }
  if (!strcmp(cmd, "RTOS") && a) {
    unsigned long ms = parse_ms(a, n, "rtos");
    if (!ms) {
      return;
    }
    uint32_t elapsed, count[2];
    if (!rtos_run(ms, &elapsed, count)) {
      Serial.printf("HIL_RESULT %s rtos REJECT why=task\r\n", n);
      return;
    }
    Serial.printf("HIL_RESULT %s rtos PASS ms=%lu a=%lu b=%lu timer=%lu\r\n", n,
                  (unsigned long)elapsed, (unsigned long)count[0],
                  (unsigned long)count[1], (unsigned long)timer_ticks);
    return;
  }
  Serial.printf("HIL_RESULT %s ? REJECT why=unknown\r\n", n);
}

void setup() {
  Serial.begin(115200);
  queue = xQueueCreate(16, sizeof(uint8_t));
  timer.begin(TIMER_MS, timer_cb);
}

void loop() {
  static char line[64];
  static size_t n;
  // discarding an overlong line through its newline: its tail is no command
  static bool overlong;
  while (Serial.available()) {
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
