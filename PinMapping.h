#define  PPSUnLock __builtin_write_OSCCONL(OSCCON & 0xbf)
#define  PPSLock __builtin_write_OSCCONL(OSCCON | 0x40)

#if defined(_INT0R) 
/// Maps INT0 to a remappable pin; 
/// see CONFIG_xxx_TO_RP(pin) for more information
#define CONFIG_INT0_TO_RP(pin) _INT0R = pin
#else
#define CONFIG_INT0_TO_RP(pin)
#endif

#if defined(_INT1R) 
/// Maps INT1 to a remappable pin; 
/// see CONFIG_xxx_TO_RP(pin) for more information
#define CONFIG_INT1_TO_RP(pin) _INT1R = pin
#else
#define CONFIG_INT1_TO_RP(pin)
#endif

#if defined(_INT2R) 
#define CONFIG_INT2_TO_RP(pin) _INT2R = pin
#else
#define CONFIG_INT2_TO_RP(pin)
#endif

#if defined(_T2CKR) 
#define CONFIG_T2CK_TO_RP(pin) _T2CKR = pin
#else
#define CONFIG_T2CK_TO_RP(pin)
#endif

#if defined(_T3CKR) 
#define CONFIG_T3CK_TO_RP(pin) _T3CKR = pin
#else
#define CONFIG_T3CK_TO_RP(pin)
#endif

#if defined(_T4CKR) 
#define CONFIG_T4CK_TO_RP(pin) _T4CKR = pin
#else
#define CONFIG_T4CK_TO_RP(pin)
#endif

#if defined(_T5CKR) 
#define CONFIG_T5CK_TO_RP(pin) _T5CKR = pin
#else
#define CONFIG_T5CK_TO_RP(pin)
#endif

#if defined(_IC1R) 
#define CONFIG_IC1_TO_RP(pin) _IC1R = pin
#else
#define CONFIG_IC1_TO_RP(pin)
#endif

#if defined(_IC2R) 
#define CONFIG_IC2_TO_RP(pin) _IC2R = pin
#else
#define CONFIG_IC2_TO_RP(pin)
#endif

#if defined(_IC3R) 
#define CONFIG_IC3_TO_RP(pin) _IC3R = pin
#else
#define CONFIG_IC3_TO_RP(pin)
#endif

#if defined(_IC4R) 
#define CONFIG_IC4_TO_RP(pin) _IC4R = pin
#else
#define CONFIG_IC4_TO_RP(pin)
#endif

#if defined(_IC5R) 
#define CONFIG_IC5_TO_RP(pin) _IC5R = pin
#else
#define CONFIG_IC5_TO_RP(pin)
#endif

#if defined(_IC6R) 
#define CONFIG_IC6_TO_RP(pin) _IC6R = pin
#else
#define CONFIG_IC6_TO_RP(pin)
#endif

#if defined(_IC7R) 
#define CONFIG_IC7_TO_RP(pin) _IC7R = pin
#else
#define CONFIG_IC7_TO_RP(pin)
#endif

#if defined(_IC8R) 
#define CONFIG_IC8_TO_RP(pin) _IC8R = pin
#else
#define CONFIG_IC8_TO_RP(pin)
#endif

#if defined(_OCFAR) 
#define CONFIG_OCFA_TO_RP(pin) _OCFAR = pin
#else
#define CONFIG_OCFA_TO_RP(pin)
#endif

#if defined(_OCFBR) 
#define CONFIG_OCFB_TO_RP(pin) _OCFBR = pin
#else
#define CONFIG_OCFB_TO_RP(pin)
#endif

#if defined(_U1RXR) 
#define CONFIG_U1RX_TO_RP(pin) _U1RXR = pin
#else
#define CONFIG_U1RX_TO_RP(pin)
#endif

#if defined(_U1CTSR) 
#define CONFIG_U1CTS_TO_RP(pin) _U1CTSR = pin
#else
#define CONFIG_U1CTS_TO_RP(pin)
#endif

#if defined(_U2RXR) 
#define CONFIG_U2RX_TO_RP(pin) _U2RXR = pin
#else
#define CONFIG_U2RX_TO_RP(pin)
#endif

#if defined(_U2CTSR) 
#define CONFIG_U2CTS_TO_RP(pin) _U2CTSR = pin
#else
#define CONFIG_U2CTS_TO_RP(pin)
#endif

#if defined(_SDI1R) 
#define CONFIG_SDI1_TO_RP(pin) _SDI1R = pin
#else
#define CONFIG_SDI1_TO_RP(pin)
#endif

#if defined(_SCK1R) 
#define CONFIG_SCK1IN_TO_RP(pin) _SCK1R = pin
#else
#define CONFIG_SCK1IN_TO_RP(pin)
#endif

#if defined(_SS1R) 
#define CONFIG_SS1IN_TO_RP(pin) _SS1R = pin
#else
#define CONFIG_SS1IN_TO_RP(pin)
#endif

#if defined(_SDI2R) 
#define CONFIG_SDI2_TO_RP(pin) _SDI2R = pin
#else
#define CONFIG_SDI2_TO_RP(pin)
#endif

#if defined(_SCK2R) 
#define CONFIG_SCK2IN_TO_RP(pin) _SCK2R = pin
#else
#define CONFIG_SCK2IN_TO_RP(pin)
#endif

#if defined(_SS2R) 
#define CONFIG_SS2IN_TO_RP(pin) _SS2R = pin
#else
#define CONFIG_SS2IN_TO_RP(pin)
#endif

#if defined(_C1RXR) 
#define CONFIG_C1RXR_TO_RP(pin) _C1RXR = pin
#else
#define CONFIG_C1RXR_TO_RP(pin)
#endif

#if defined(_C2RXR) 
#define CONFIG_C2RXR_TO_RP(pin) _C2RXR = pin
#else
#define CONFIG_C2RXR_TO_RP(pin)
#endif
//@}


//end RP input mapping
//Your device may not have all of these peripherals!
//start RP output mapping


#if defined(_RP0R) 
/// Maps C1OUT to a remappable pin; 
/// see CONFIG_yyy_TO_RP(pin) for more information.
#define CONFIG_C1OUT_TO_RP(pin) _RP##pin##R = 1
#else
#define CONFIG_C1OUT_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_C2OUT_TO_RP(pin) _RP##pin##R = 2
#else
#define CONFIG_C2OUT_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_U1TX_TO_RP(pin) _RP##pin##R = 3
#else
#define CONFIG_U1TX_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_U1RTS_TO_RP(pin) _RP##pin##R = 4
#else
#define CONFIG_U1RTS_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_U2TX_TO_RP(pin) _RP##pin##R = 5
#else
#define CONFIG_U2TX_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_U2RTS_TO_RP(pin) _RP##pin##R = 6
#else
#define CONFIG_U2RTS_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_SDO1_TO_RP(pin) _RP##pin##R = 7
#else
#define CONFIG_SDO1_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_SCK1OUT_TO_RP(pin) _RP##pin##R = 8
#else
#define CONFIG_SCK1OUT_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_SS1OUT_TO_RP(pin) _RP##pin##R = 9
#else
#define CONFIG_SS1OUT_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_SDO2_TO_RP(pin) _RP##pin##R = 10
#else
#define CONFIG_SDO2_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_SCK2OUT_TO_RP(pin) _RP##pin##R = 11
#else
#define CONFIG_SCK2OUT_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_SS2OUT_TO_RP(pin) _RP##pin##R = 12
#else
#define CONFIG_SS2OUT_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_C1TX_TO_RP(pin) _RP##pin##R = 16
#else
#define CONFIG_C1TX_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_OC1_TO_RP(pin) _RP##pin##R = 18
#else
#define CONFIG_OC1_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_OC2_TO_RP(pin) _RP##pin##R = 19
#else
#define CONFIG_OC2_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_OC3_TO_RP(pin) _RP##pin##R = 20
#else
#define CONFIG_OC3_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_OC4_TO_RP(pin) _RP##pin##R = 21
#else
#define CONFIG_OC4_TO_RP(pin)
#endif

#if defined(_RP0R) 
#define CONFIG_OC5_TO_RP(pin) _RP##pin##R = 22
#else
#define CONFIG_OC5_TO_RP(pin)
#endif
