#pragma once

/**
 * Sends a control signal to a TCP control server.
 *
 * @param start  If non-zero, sends a START signal (0x10);
 *               otherwise sends a STOP signal (0x20).
 * @return       0 on success, -1 on failure.
 */
int send_control_signal(int start);
