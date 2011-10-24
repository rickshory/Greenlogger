;
; file: AssemblerCode.s
;
 .equ   __24FJ64, 1
 .include  "p24FJ64GA002.inc"

.bss
.global _safe_ALCFGRPT
.align 2
_safe_ALCFGRPT: .space 2

.text

.global _safely_set_ALCFGRPT

_safely_set_ALCFGRPT:
 ; on entry, the passed int value will be in w0
 
 ; set up ALCFGRPT using workaround to avoid glitches
 ; 1. Read ALCFGRPT into a RAM location.
 mov ALCFGRPT, w1
 mov w1, _safe_ALCFGRPT
 ; 2. Modify the ALCFGRPT data, as required, in RAM.
 ; temporarily disable alarm
 bclr _safe_ALCFGRPT, #15
  ; 3. Move the RAM value into ALCFGRPT, and a dummy location, in back-to-back instructions.
 mov _safe_ALCFGRPT, w1
 mov w1, ALCFGRPT
 ;dummy write prevents ALCFGRPT bit corruption from desynchronization between CPU & RTCC clock domains
 mov w1, _safe_ALCFGRPT
 ;do it again to update the value
 ; 1. Read ALCFGRPT into a RAM location.
 mov ALCFGRPT, w1
 mov w1, _safe_ALCFGRPT
 ; 2. Modify the ALCFGRPT data, as required, in RAM.
 ; use the passed value
 mov w0, _safe_ALCFGRPT
  ; 3. Move the RAM value into ALCFGRPT, and a dummy location, in back-to-back instructions.
 mov _safe_ALCFGRPT, w1
 mov w1, ALCFGRPT
 ;dummy write prevents ALCFGRPT bit corruption from desynchronization between CPU & RTCC clock domains
 mov w1, _safe_ALCFGRPT
return

.end
