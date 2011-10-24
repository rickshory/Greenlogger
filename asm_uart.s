;
; file: asm_uart.s
;
 .equ   __24FJ64, 1
 .include  "p24FJ64GA002.inc"

; .equ USART1_Buffer_Length, 0x800
 .equ USART1_Buffer_Length, 0x800

.bss
;.global _safe_ALCFGRPT
;.align 2
;_safe_ALCFGRPT: .space 2
;.global _machState
;.align 2
;_machState: .space 2

.global _USART1_inputbuffer_head
.global _USART1_inputbuffer_tail
.global _USART1_inputbuffer
.global _USART1_outputbuffer_head
.global _USART1_outputbuffer_tail
.global _USART1_outputbuffer

;
;LogData_Buffer_head: .space 2 ;use for collecting data string bytes, before logging to SD card
;LogData_Buffer: .space LogData_Buffer_Length
;
;Commandbuffer_head: .space 2 ;use for collecting command string bytes, before parsing
;Commandbuffer:   .space Command_Buffer_Length ;allocating 32 bytes of buffer space
;
_USART1_inputbuffer_head: .space 2
_USART1_inputbuffer_tail: .space 2
_USART1_inputbuffer:   .space USART1_Buffer_Length ;allocating 1024 bytes of buffer space 
;
_USART1_outputbuffer_head: .space 2
_USART1_outputbuffer_tail: .space 2
_USART1_outputbuffer:   .space USART1_Buffer_Length ;allocating 1024 bytes of buffer space
;
; Re. all these Head and Tail buffers:
;  in between work on buffers, if Head = Tail, the buffer is empty.
;  if Head is 1 more than Tail, Tail points to the 1 data byte and Head points one address ahead.
;  if Head is n more than Tail, the n addresses starting where Tail points contain the data, and
;   Head points one address in front of them.
;  "Ahead" of course takes into account rollover from the end of the buffer to the beginning.
;  If, during testing, we increment Head and (allowing for rollover) it = Tail, the buffer is full.
;

.text
; the following "__xxxInterrupt" symbols are standard names defined in the *.gld file
;  To enable the interrupt, write a function of this name, and 
;  set configuration bits to enable the corresponding interrupt.  The
;  function will serve as the interrupt service routine.
; .global  __RTCCInterrupt   
; .global  __SPI1Interrupt
 .global  __U1TXInterrupt 
 .global  __U1RXInterrupt
; .global  __T1Interrupt
; .global  __T2Interrupt  
; .global  __T3Interrupt
; .global  __CompInterrupt  


;***************************************
;
; Function:  __U1TXInterrupt
;  Overview:        
;  PreCondition: none
;  Input:          none
;  Output:         none
;  Side Effects:   none
; Changes:  none
;  Calls:          none
;
; * * * * *
; from AN774, page 13, in Asynchronous Communications with the PICmicro® USART
;TRANSMIT INTERRUPT OPERATION
;  The TXIF bit is cleared when data is written to TXREG and gets set when this data moves into the Transmit
; Shift Register to get transmitted. This means that the interrupt occurs when new data can be written to TXREG.
;
;  Whenever TXREG is empty, the TXIF bit will be set and an interrupt will occur if the interrupt is enabled. This
; provides a useful way to transmit data as fast as possible, but it is necessary to have the data available when
; the interrupt occurs. It is common to use a buffer that is read by the interrupt routine, one byte being written to
; TXREG each time the interrupt occurs. When the last byte of data (from the buffer) has been written to
; TXREG, the TXIE bit must be cleared to stop further interrupts from occurring. The interrupt can be enabled
; again later when new data needs to be transmitted and this will immediately cause an interrupt. If the code disables
; transmit interrupts and any other interrupts can occur, the interrupt routine must test for both the interrupt
; flag and the enable bit, because the interrupt flag can be set regardless of whether the interrupt is
; enabled.
; * * * * *
;  Therefore, this function will put at most one byte into the Transmit register on each call, though
; it may be called repeatedly in quick succession
;***************************************
; 
__U1TXInterrupt:
  push w0 ; save registers used here
  push w1
  push w2
  push w3
  ; test if any data to send
  mov  _USART1_outputbuffer_head, W0
  mov  _USART1_outputbuffer_tail, W1
  cpseq W0, W1
  bra  __U1TXInterrupt_Send
;
  bclr IEC0, #U1TXIE ; buffer is empty, disable interrupts
  bra  __U1TXInterrupt_Exit
;
__U1TXInterrupt_Send:
  mov.b [w1], w2 ; get byte we are going to send
; diagnostics: write character "O" to Output location we are done with
;mov.b #'O', w3
;mov.b w3, [w1]
  inc  W1, W1 ; generate new Tail address, start by incrementing
  mov  #_USART1_outputbuffer+USART1_Buffer_Length, W0
  cpseq W0, W1 ; test if rollover
  bra  __U1TXInterrupt_Send_NotEndOfBuffer
;
__U1TXInterrupt_Send_EndOfBuffer:
  mov  #_USART1_outputbuffer, W1 ; rollover new Tail address to start of buffer
;
__U1TXInterrupt_Send_NotEndOfBuffer:
; following 3 lines are old way
;  mov  USART1_outputbuffer_tail, W0 ; get current Tail pointer
;  mov.b [W0], w0 ; from pointer get data byte
;  mov  w0, U1TXREG ; put byte in the Transmit register
; new way
mov  w2, U1TXREG ; put byte in the Transmit register

  mov  W1, _USART1_outputbuffer_tail ; write new Tail pointer
  ; also, if starting from USART idle, this allows a cycle for U1TXIF to be properly set
  bclr IFS0, #U1TXIF  ; if problems, try rearranging this code; clear U1TXIF early, and put DISI here
;  
__U1TXInterrupt_Exit:

  pop w3
  pop w2
  pop w1
  pop w0 ; restore registers
  retfie

;***************************************
;
; Function:  __U1RXInterrupt
;  Overview:        
;  PreCondition: none
;  Input:          none
;  Output:         none
;  Side Effects:   none
;  Changes:  none     
;  Calls:          none
;
;***************************************
; 
__U1RXInterrupt:
  push.d W0      ; save registers that will be used here; 
;
__U1RXInterrupt1:
  mov _USART1_inputbuffer_head, W0
  inc  W0, W0 ; get tentative next head position
  mov  #_USART1_inputbuffer+USART1_Buffer_Length, W1 ; check end of buffer
  cpseq W0, W1    ; skip next if end of buffer
  bra  __U1RXInterrupt_NotEndOfBuffer
;
__U1RXInterrupt_EndOfBuffer:
  mov  #_USART1_inputbuffer, W0 ; head wraps around to start of buffer
;
__U1RXInterrupt_NotEndOfBuffer:  
  mov _USART1_inputbuffer_tail, W1 ; if calculated next head = tail, buffer is full
  cpseq W0, W1    ; skip next if buffer full
  bra  __U1RXInterrupt_MoveToBuffer
;
__U1RXInterrupt_NoRoomInBuffer:  
  mov  U1RXREG, W0    ; toss out the USART data
  bra  __U1RXInterrupt_Chk4More
; 
__U1RXInterrupt_MoveToBuffer: 
  mov  _USART1_inputbuffer_head, W1 ;re-read prev head, which will be write position
  mov  W0, _USART1_inputbuffer_head ;write calcd head, which has passed tests; points one past valid data
  mov  U1RXREG, W0    ; save the USART data
  mov.b W0, [W1]
;
__U1RXInterrupt_Chk4More:  
  btss U1STA, #URXDA
  bra  __U1RXInterrupt_Exit
;
  bra  __U1RXInterrupt1
;
__U1RXInterrupt_Exit:
  pop.d W0      ; replace registers that were used here
  bclr IFS0, #U1RXIF  ; clear interrupt flag
  retfie
;

.end
