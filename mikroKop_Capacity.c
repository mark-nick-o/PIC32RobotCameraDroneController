// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + www.MikroKopter.com
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Software Nutzungsbedingungen (english version: see below)
// + der Fa. HiSystems GmbH, Flachsmeerstrasse 2, 26802 Moormerland - nachfolgend Lizenzgeber genannt -
// + Der Lizenzgeber räumt dem Kunden ein nicht-ausschließliches, zeitlich und räumlich* unbeschränktes Recht ein, die im den
// + Mikrocontroller verwendete Firmware für die Hardware Flight-Ctrl, Navi-Ctrl, BL-Ctrl, MK3Mag & PC-Programm MikroKopter-Tool 
// + - nachfolgend Software genannt - nur für private Zwecke zu nutzen.
// + Der Einsatz dieser Software ist nur auf oder mit Produkten des Lizenzgebers zulässig.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Die vom Lizenzgeber gelieferte Software ist urheberrechtlich geschützt. Alle Rechte an der Software sowie an sonstigen im
// + Rahmen der Vertragsanbahnung und Vertragsdurchführung überlassenen Unterlagen stehen im Verhältnis der Vertragspartner ausschließlich dem Lizenzgeber zu.
// + Die in der Software enthaltenen Copyright-Vermerke, Markenzeichen, andere Rechtsvorbehalte, Seriennummern sowie
// + sonstige der Programmidentifikation dienenden Merkmale dürfen vom Kunden nicht verändert oder unkenntlich gemacht werden.
// + Der Kunde trifft angemessene Vorkehrungen für den sicheren Einsatz der Software. Er wird die Software gründlich auf deren
// + Verwendbarkeit zu dem von ihm beabsichtigten Zweck testen, bevor er diese operativ einsetzt.
// + Die Haftung des Lizenzgebers wird - soweit gesetzlich zulässig - begrenzt in Höhe des typischen und vorhersehbaren
// + Schadens. Die gesetzliche Haftung bei Personenschäden und nach dem Produkthaftungsgesetz bleibt unberührt. Dem Lizenzgeber steht jedoch der Einwand 
// + des Mitverschuldens offen.
// + Der Kunde trifft angemessene Vorkehrungen für den Fall, dass die Software ganz oder teilweise nicht ordnungsgemäß arbeitet.
// + Er wird die Software gründlich auf deren Verwendbarkeit zu dem von ihm beabsichtigten Zweck testen, bevor er diese operativ einsetzt.
// + Der Kunde wird er seine Daten vor Einsatz der Software nach dem Stand der Technik sichern.
// + Der Kunde ist darüber unterrichtet, dass der Lizenzgeber seine Daten im zur Vertragsdurchführung erforderlichen Umfang
// + und auf Grundlage der Datenschutzvorschriften erhebt, speichert, verarbeitet und, sofern notwendig, an Dritte übermittelt.
// + *) Die räumliche Nutzung bezieht sich nur auf den Einsatzort, nicht auf die Reichweite der programmierten Software.
// + #### ENDE DER NUTZUNGSBEDINGUNGEN ####'
// +  Hinweis: Informationen über erweiterte Nutzungsrechte (wie z.B. Nutzung für nicht-private Zwecke) sind auf Anfrage per Email an info(@)hisystems.de verfügbar.
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + Software LICENSING TERMS
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + of HiSystems GmbH, Flachsmeerstrasse 2, 26802 Moormerland, Germany - the Licensor -
// + The Licensor grants the customer a non-exclusive license to use the microcontroller firmware of the Flight-Ctrl, Navi-Ctrl, BL-Ctrl, and MK3Mag hardware 
// + (the Software) exclusively for private purposes. The License is unrestricted with respect to time and territory*.
// + The Software may only be used with the Licensor's products.
// + The Software provided by the Licensor is protected by copyright. With respect to the relationship between the parties to this
// + agreement, all rights pertaining to the Software and other documents provided during the preparation and execution of this
// + agreement shall be the property of the Licensor.
// + The information contained in the Software copyright notices, trademarks, other legal reservations, serial numbers and other
// + features that can be used to identify the program may not be altered or defaced by the customer.
// + The customer shall be responsible for taking reasonable precautions
// + for the safe use of the Software. The customer shall test the Software thoroughly regarding its suitability for the
// + intended purpose before implementing it for actual operation. The Licensor's liability shall be limited to the extent of typical and
// + foreseeable damage to the extent permitted by law, notwithstanding statutory liability for bodily injury and product
// + liability. However, the Licensor shall be entitled to the defense of contributory negligence.
// + The customer will take adequate precautions in the case, that the software is not working properly. The customer will test
// + the software for his purpose before any operational usage. The customer will backup his data before using the software.
// + The customer understands that the Licensor collects, stores and processes, and, where required, forwards, customer data
// + to third parties to the extent necessary for executing the agreement, subject to applicable data protection and privacy regulations.
// + *) The territory aspect only refers to the place where the Software is used, not its programmed range.
// + #### END OF LICENSING TERMS ####
// + Note: For information on license extensions (e.g. commercial use), please contact us at info(@)hisystems.de.
//
// + Ported to the pic32/ft900 (mikroE C) by ACP Aviation
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <stdlib.h>
#include "definitions.h"                                                        // global defines
#include "mikroKop_Capacity.h"
#include "gc_events.h"
/*#include "main.h"
#include "timer0.h"
#include "analog.h" */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//#define CAPACITY_UPDATE_INTERVAL 10 // 10 ms
#define CAPACITY_UPDATE_INTERVAL 50u                                            // 50 ms  = 20Hz
#define FC_OFFSET_CURRENT 5u                                                    // calculate with a current of 0.5A
#define BL_OFFSET_CURRENT 2u                                                    // calculate with a current of 0.2A

int32_t update_timer;
uint32_t update_last;                                             
MkCapacity_t Capacity;

void Capacity_Init(void);
uint32_t BL3_Current(uint8_t who);
void Capacity_Update(void);

/*-----------------------------------------------------------------------------
 *      Capacity_Init():  initialize capacity calculation
 *
 *  Parameters: void
 *  Return: void
 *----------------------------------------------------------------------------*/
void Capacity_Init(void)
{
	Capacity.ActualCurrent = 0u;
	Capacity.UsedCapacity = 0u;
	Capacity.ActualPower = 0u;
	Capacity.MinOfMaxPWM = 0u;
	/* ---- TODO update_timer = SetDelay(CAPACITY_UPDATE_INTERVAL); ---- */
	update_timer = -1;
	calculateTick2Now( &update_timer, &update_last );
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// + extended Current measurement -> 200 = 20A    201 = 21A    255 = 75A (20+55)
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*-----------------------------------------------------------------------------
 *  BL3_Current():  extended Current measurement
 *
 *  Parameters: uint8_t who
 *  Return: uint32_t
 *----------------------------------------------------------------------------*/
uint32_t BL3_Current(uint8_t who)                                               // in 0,1A
{
    if(Motor[who].Current == 255u) return(0u);                                  // invalid
    if(Motor[who].Current <= 200u) return(Motor[who].Current);
    else
    {
      if(Motor[who].Version & MOTOR_STATE_BL30) return(200u + 10u * ((uint32_t)Motor[who].Current-200u));
      else return(Motor[who].Current);
    }
}

/*-----------------------------------------------------------------------------
 *  Capacity_Update():  called in main loop at a regular interval
 *
 *  Parameters: void
 *  Return: void
 *----------------------------------------------------------------------------*/
void Capacity_Update(void)
{
	uint32_t Current, SetSum;                                               // max value will be 255 * 12 = 3060
	uint16_t SubCounter = 0u;
	uint16_t CurrentOffset = 0u;
	uint64_t SumCurrentOffset = 0u;
	uint8_t i, NumOfMotors, MinOfMaxPWM;

/*	if(CheckDelay(update_timer))
	{ TODO */
	calculateTick2Now( &update_timer, &update_last );
	if (update_timer >= CAPACITY_UPDATE_INTERVAL)
	{ 
		// update_timer += CAPACITY_UPDATE_INTERVAL;                    do not use SetDelay to avoid timing leaks
		                                                                // determine sum of all present BL currents and setpoints
		Current = 0u;
		SetSum = 0u;
		NumOfMotors = 0u;
		MinOfMaxPWM = 255u;
		if(Capacity.MinOfMaxPWM == 254u) FC_StatusFlags3 |= FC_STATUS3_REDUNDANCE_AKTIVE; 
		else if(Capacity.MinOfMaxPWM == 255u) 	FC_StatusFlags3 &= ~FC_STATUS3_REDUNDANCE_AKTIVE;  
		
		for(i = 0u; i < MAX_MOTORS; i++)
		{
			if(Motor[i].State & MOTOR_STATE_PRESENT_MASK /* && Mixer.Motor[i][MIX_GAS]*/ )
			{
				NumOfMotors++;
				if(Motor[i].Current > 200u)
			        {
				   Current += BL3_Current(i);                   // extended Current measurement -> 200 = 20A    201 = 21A    255 = 75A (20+55)
				}
				else Current += (uint32_t)(Motor[i].Current);
				SetSum +=  (uint32_t)(Motor[i].SetPoint);
				if(Motor[i].MaxPWM <= MinOfMaxPWM) MinOfMaxPWM = Motor[i].MaxPWM; 
				else 
				if(Motor[i].MaxPWM == 255u) FC_StatusFlags3 &= ~FC_STATUS3_REDUNDANCE_AKTIVE; 
			}
		}
		Capacity.MinOfMaxPWM = MinOfMaxPWM;
		if(SetSum == 0u)                                                // if all setpoints are 0
		{                                                               // determine offsets of motor currents
			#define CURRENT_AVERAGE 8u                              // 8bit = 256 * 10 ms = 2.56s average time
			CurrentOffset = (uint32_t)(SumCurrentOffset>>CURRENT_AVERAGE);
			SumCurrentOffset -= CurrentOffset;
			SumCurrentOffset += Current;
			
			Current = FC_OFFSET_CURRENT;                            // after averaging set current to static offset
			FC_StatusFlags3 &= ~FC_STATUS3_REDUNDANCE_AKTIVE;	
		}
		else                                                            // some motors are running, includes also motor test condition, where "MotorRunning" is false
		{                                                               // subtract offset
			if(Current > CurrentOffset) Current -= CurrentOffset;
			else Current = 0u;
			
			Current += FC_OFFSET_CURRENT + NumOfMotors * BL_OFFSET_CURRENT; // add the FC and BL Offsets
		}


		Capacity.ActualCurrent = Current;                               // update actual Current
		if(Current < 255u) Capacity.ActualPower = (UBat * Current) / 100u; // update actual Power in W higher resolution
		else Capacity.ActualPower = (UBat * (Current/4u)) / 25u;        // update actual Power in W
		SubCounter += Current;                                          // update used capacity

		// 100mA * 1ms * CAPACITY_UPDATE_INTERVAL = 1 mA * 100 ms * CAPACITY_UPDATE_INTERVAL
		// = 1mA * 0.1s * CAPACITY_UPDATE_INTERVAL = 1mA * 1min / (600 / CAPACITY_UPDATE_INTERVAL)
		// = 1mAh / (36000 / CAPACITY_UPDATE_INTERVAL)
		#define SUB_COUNTER_LIMIT (36000ul / CAPACITY_UPDATE_INTERVAL)
		while(SubCounter > SUB_COUNTER_LIMIT)
		{
			Capacity.UsedCapacity++;			        // we have one mAh more
			SubCounter -= SUB_COUNTER_LIMIT;	                // keep the remaining sub part
		}
		calculateTickDiff( &update_timer, &update_last );               // reset the reference time to START again from now
	}                                                                       // EOF check delay update timer
}

#ifdef __cplusplus
}
#endif
