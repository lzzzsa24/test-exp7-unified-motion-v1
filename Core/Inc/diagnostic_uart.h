#ifndef __DIAGNOSTIC_UART_H
#define __DIAGNOSTIC_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void DiagnosticUart_Init(void);
void DiagnosticUart_WriteString(const char *text);
void DiagnosticUart_WriteSigned(int32_t value);
void DiagnosticUart_WriteUnsigned(uint32_t value);
/* Returns -1 when no byte is available. */
int16_t DiagnosticUart_ReadChar(void);

#ifdef __cplusplus
}
#endif

#endif /* __DIAGNOSTIC_UART_H */
