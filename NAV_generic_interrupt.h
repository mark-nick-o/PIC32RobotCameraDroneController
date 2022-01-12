void NAV_interrupt() iv UANAV_VECTOR ilevel 7 ics ICS_AUTO                      /* interrupt on UART and store data in receive buffer */
{
   asm EI;                                                                      /* Reenable global interrupts */
#ifndef NAVLINK_USE_UART1                                                       /* UART1 doesnt seem to have this ???  */
   if((UANAVSTA & 0x0Eu) | UANAVEIF_Bit)                                        /* Error check with UART buffer */
   {
      if (OERR_UANAV_bit==1u)                                                   /* Overrun error (bit 1) */
      {
        OERR_UANAV_bit = 0u;
        NavOBject.Buffer_Overrun=++NavOBject.Buffer_Overrun % UINT16_MAX;      /* Count overruns  */
      }
      else if (FERR_UANAV_bit==1u)                                              /* Framing error (bit 2) */
      {
        FERR_UANAV_bit = 0u;
        NavOBject.Framing_Error=++NavOBject.Framing_Error % UINT16_MAX;
      }
      else if (PERR_UANAV_bit==1u)                                              /* Parity error (bit 3)    */
      {
        PERR_UANAV_bit = 0u;
        NavOBject.Parity_Error=++NavOBject.Parity_Error % UINT16_MAX;
      }
      NavOBject.Index = 0;                                                           /* Set the index to zero to recollect by looking for a new message start */
      UANAVSTA &= 0xFFFFFFF1UL;                                                 /* Clear the Error Status 0001 no longer 1101 (D) just the overrun , clears frame and parity  */
      UANAVEIF_Bit =CLEAR;                                                      /* Clear the Error IF */
      UANAVRXIF_bit=CLEAR;                                                      /* Clear the UART IF */
   }
#endif

  if (UANAVRXIF_bit)            /* a byte was received by the UART and we didnt have a full frame unprocessed */
  {
#ifndef NAVLINK_USE_UART1
     while(URXDA_UANAV_bit)                                                     /* while there is a byte to receive */
     {
#endif  /* ============================= end if_UART1 */
        NavOBject.buf[NavOBject.Index] = UANAVRXREG;                            /* put the byte from the UART into the buffer */
        NavOBject.Index=++NavOBject.Index % UINT16_MAX;                         /* increment and fill the buffer with data from the payload */
#ifndef NAVLINK_USE_UART1
     }
#endif  /* ============================= end if_UART1  */
  }  /* End U_RXIFBIT  */
  NavOBject.buf[NavOBject.Index] = '\0';
  navilockInterruptStateManager( &NavOBject );                                  /* call the interrupt state manager */

  UANAVRXIF_bit=CLEAR;                                                          /* clear the interrupt flag check when packet has been receievd and not processed if we need to remove then or not we want to keep in buffer */
}