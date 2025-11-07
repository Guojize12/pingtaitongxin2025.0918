#include "at_commands.h"
#include "uart_utils.h"
#include "state_machine.h"
#include "config.h"
#include "comm_manager.h"
#include <Arduino.h>

void startATPing() {
  sendCmd("AT");
  gotoStep(STEP_AT_PING);
}

void queryCEREG() {
  sendCmd("AT+CEREG?");
  gotoStep(STEP_CEREG);
}

void setEncoding() {
  sendCmd("AT+MIPCFG=\"encoding\",0,1,0");
  gotoStep(STEP_ENCODING);
}

void closeCh0() {
  sendCmd("AT+MIPCLOSE=0");
  gotoStep(STEP_MIPCLOSE);
}

void openTCP() {
  char buf[128];
  snprintf(buf, sizeof(buf), "AT+MIPOPEN=0,\"TCP\",\"%s\",%d", SERVER_IP, SERVER_PORT);
  sendCmd(buf);
  gotoStep(STEP_MIPOPEN);
}

void pollMIPSTATE() {
  sendCmd("AT+MIPSTATE=0");
  scheduleStatePoll();
}

// Best-effort soft power-cycle for modem (try graceful via AT, fallback to nothing if no hw pins)
void modem_soft_power_cycle() {
    // Try graceful shutdown via AT if module responsive
    // Many modules accept AT+CFUN=0 (rf off) and AT+CPWROFF (power off) or AT+QPOWD
    sendCmd("AT+CFUN=0");
    delay(200);
    sendCmd("AT+CPWROFF");
    delay(300);
    // Some modules need extra poweroff command variants; try common ones
    sendCmd("AT+QPOWD");
    delay(300);

    // Bring it back up logically (if module supports)
    sendCmd("AT+CFUN=1");
    delay(500);

    // Note: if hardware power pin exists, the main recovery routine toggles it.
}