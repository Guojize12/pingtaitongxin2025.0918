#pragma once

void startATPing();
void queryCEREG();
void setEncoding();
void closeCh0();
void openTCP();
void pollMIPSTATE();

// Best-effort soft power-cycle / reset modem via AT
void modem_soft_power_cycle();