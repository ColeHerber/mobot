#pragma once

// Stub driver for UART1 (J4, IO41 RX / IO42 TX). Brings the bus up so a
// future consumer can call Serial1.* directly. Nothing reads or writes it
// yet.

void uart1Init();
