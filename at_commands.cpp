#include "at_commands.h"
#include "uart_utils.h"
#include "state_machine.h"
#include "config.h"
#include "comm_manager.h"

void startATPing() {
  log2("Send AT");
  sendCmd("AT");
  gotoStep(STEP_AT_PING);
}

void queryCEREG() {
  log2("Query CEREG");
  sendCmd("AT+CEREG?");
  gotoStep(STEP_CEREG);
}

void setEncoding() {
  log2("Set encoding ch0 HEX/ASCII");
  sendCmd("AT+MIPCFG=\"encoding\",0,1,0");
  gotoStep(STEP_ENCODING);
}

void closeCh0() {
  log2("Close ch0");
  sendCmd("AT+MIPCLOSE=0");
  gotoStep(STEP_MIPCLOSE);
}

void openTCP() {
  char buf[128];
  snprintf(buf, sizeof(buf), "AT+MIPOPEN=0,\"TCP\",\"%s\",%d", SERVER_IP, SERVER_PORT);
  log2("Open TCP ch0");
  log2Str("Server IP: ", SERVER_IP);
  log2Val("Server Port: ", SERVER_PORT);
  sendCmd(buf);
  gotoStep(STEP_MIPOPEN);
}

void pollMIPSTATE() {
  sendCmd("AT+MIPSTATE=0");
  scheduleStatePoll();
}