#include "camera.h"
//
//  Return true if a datachange occurred for XStream camera
//
uint8_t XSChangeDetectedOneShot( XS_Hmi_Slider_t *ptr_HMIobject )               // Function to detect a data change by more than X (one shot pulse)
{
   uint8_t changeMade;                                                          // return code 1 for change 0 for no-change
   changeMade = (uint8_t) (abs(ptr_HMIobject->value - ptr_HMIobject->last)>ptr_HMIobject->delta);   // 1 if changed by more than delta 0 for no-change
   ptr_HMIobject->last = ptr_HMIobject->value;                                  // store the new last value for next comparison
   return changeMade;                                                           // return to calling level
}
//
//  Return true if a datachange occurred for XStream camera
//
uint8_t XSChangeDetected( XS_Hmi_Slider_t *ptr_HMIobject )                      // Function to detect a data change by more than X
{
   return (uint8_t) (abs(ptr_HMIobject->value - ptr_HMIobject->last)>=ptr_HMIobject->delta);   // 1 if changed by more than delta 0 for no-change
}