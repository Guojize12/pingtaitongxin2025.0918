#include <Arduino.h>
#include "config.h"
#include "uart_utils.h"
#include "state_machine.h"
#include "platform_packet.h"
#include "at_commands.h"

volatile int g_monitorEventUploadFlag = 0;

void setup() {
  Serial.begin(DTU_BAUD);
  Serial2.begin(LOG_BAUD, SERIAL_8N1, RX2, TX2);

  log2("Boot");
  log2("UART0 for DTU, UART2 for log");
  log2Str("Server: ", SERVER_IP);
  log2Val("Port: ", SERVER_PORT);

  resetBackoff();
  gotoStep(STEP_IDLE);
}

void loop() {
  readDTU();
  driveStateMachine();
}