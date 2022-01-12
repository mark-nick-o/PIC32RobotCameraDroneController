#include <stdint.h>

#define F_ONE 0xff
#define F_ZERO 0x00
#define F_OR(a,b) ((a) > (b) ? (a) : (b))
#define F_AND(a,b) ((a) < (b) ? (a) : (b))
#define F_NOT(a) (F_ONE+F_ZERO-a)

char __IDOM[2u];
 int16_t OldError,SumError;
 int16_t process;
/* LINGUISTIC Error TYPE int MIN -90 MAX 90 */
/* { */
int Error ;
/* consequence */
int16_t ManVar ;
int32_t fa_ConsName, fc_ConsName, ConsVol, ConsPoint;
/* MEMBER LNegative { -90, -90, -20, 0 } */
/*

 1-| .............
 | . .
 | . .
 | . .
 0-| . .................
 ----------------------------------
 -90 -45 0 45 90

*/
char Error_LNegative (int __CRISP)
 {
 {
 if (__CRISP <= -20) return(255);
 else
 {
 if (__CRISP <= 0)
 return(( - __CRISP * 12) + 7);
 else
 return(0);
 }
 }
 }
/* MEMBER normal { -20, 0, 20 } */
/*

 1-| .
 | . .
 | . .
 | . .
 0-| ............. ..............
 ----------------------------------
 -90 -45 0 45 90

*/
char Error_normal (int __CRISP)
 {
 if (__CRISP < -20) return(0);
 else
 {
 if (__CRISP <= 0) return(((__CRISP + 20) * 12) + 7);
 else
 {
 {
 if (__CRISP <= 20)
 return((( + 20 - __CRISP) * 12) + 7);
 else
 return(0);
 }
 }
 }
 }
/* MEMBER close { -3, 0, 3 } */
/*

 1-| .
 | .
 | .
 | ..
 0-| .................................
 ----------------------------------
 -90 -45 0 45 90

*/
char Error_close (int __CRISP)
 {
 if (__CRISP < -3) return(0);
 else
 {
 if (__CRISP <= 0) return((__CRISP + 3) * 85);
 else
 {
 {
 if (__CRISP <= 3)
 return(( + 3 - __CRISP) * 85);
 else
 return(0);
 }
 }
 }
 }
/* MEMBER LPositive { 0, 20, 90, 90 } */
/*

 1-| ..............
 | . .
 | . .
 | . .
 0-| ................. .
 ----------------------------------
 -90 -45 0 45 90

*/
char Error_LPositive (int __CRISP)
 {
 if (__CRISP < 0) return(0);
 else
 {
 if (__CRISP <= 20) return((__CRISP * 12) + 7);
 else
 {
 return(255);
 }
 }
 }
/* } */
/*

 Fuzzy Sets for Error

 1-| ............. . ..............
 | . . ... . .
 | . . . . .
 | . . ... . .
 0-| ............. .. ..............
 ----------------------------------
 -90 -45 0 45 90


*/
/* LINGUISTIC DeltaError TYPE int MIN -90 MAX 90 */
/* { */
int DeltaError ;
/* MEMBER Negative { -90, -90, -10, 0 } */
/*

 1-| ...............
 | . .
 | . .
 | . .
 0-| . .................
 ----------------------------------
 -90 -45 0 45 90

*/
char DeltaError_Negative (int __CRISP)
 {
 {
 if (__CRISP <= -10) return(255);
 else
 {
 if (__CRISP <= 0)
 return(( - __CRISP * 25) + 2);
 else
 return(0);
 }
 }
 }
/* MEMBER Positive { 0, 10, 90, 90 } */
/*

 1-| ................
 | . .
 | . .
 | . .
 0-| ................. .
 ----------------------------------
 -90 -45 0 45 90

*/
char DeltaError_Positive (int __CRISP)
 {
 if (__CRISP < 0) return(0);
 else
 {
 if (__CRISP <= 10) return((__CRISP * 25) + 2);
 else
 {
 return(255);
 }
 }
 }
/* } */
/*

 Fuzzy Sets for DeltaError

 1-| ............... ................
 | . . . .
 | . . . .
 | . . .
 0-| .................................
 ----------------------------------
 -90 -45 0 45 90


*/
/* LINGUISTIC SumError TYPE int MIN -90 MAX 90 */
/* { */
/* MEMBER LNeg { -90, -90, -5, 0 } */
/*

 1-| ................
 | . .
 | . .
 | . .
 0-| . .................
 ----------------------------------
 -90 -45 0 45 90

*/
char SumError_LNeg (int __CRISP)
 {
 {
 if (__CRISP <= -5) return(255);
 else
 {
 if (__CRISP <= 0)
 return( - __CRISP * 51);
 else
 return(0);
 }
 }
 }
/* MEMBER LPos { 0, 5, 90, 90 } */
/*

 1-| .................
 | . .
 | . .
 | . .
 0-| ................. .
 ----------------------------------
 -90 -45 0 45 90

*/
char SumError_LPos (int __CRISP)
 {
 if (__CRISP < 0) return(0);
 else
 {
 if (__CRISP <= 5) return(__CRISP * 51);
 else
 {
 return(255);
 }
 }
 }
/* } */
/*

 Fuzzy Sets for SumError

 1-| .................................
 | . .. .
 | . . .
 | . . .
 0-| .................................
 ----------------------------------
 -90 -45 0 45 90


*/
/* CONSEQUENCE ManVar TYPE int MIN -20 MAX 20 DEFUZZ cg */
/* { */
/* MEMBER LNegative { -18 } */
/*

 1-| .
 | .
 | .
 | .
 0-| .*...............................
 ----------------------------------
 -20 -10 0 10 20


*/
void ManVar_LNegative (int __DOM)
 {
 fc_ConsName += ConsVol;
 fa_ConsName += (ConsVol * (ConsPoint));
 }
/* MEMBER SNegative { -6 } */
/*

 1-| .
 | .
 | .
 | .
 0-| ...........*.....................
 ----------------------------------
 -20 -10 0 10 20


*/
void ManVar_SNegative (int __DOM)
 {
 fc_ConsName += ConsVol;
 fa_ConsName += (ConsVol * (ConsPoint));
 }
/* MEMBER SPositive { 6 } */
/*

 1-| .
 | .
 | .
 | .
 0-| ....................*............
 ----------------------------------
 -20 -10 0 10 20


*/
void ManVar_SPositive (int __DOM)
 {
 fc_ConsName += ConsVol;
 fa_ConsName += (ConsVol * (ConsPoint));
 }
/* MEMBER LPositive { 18 } */
/*

 1-| .
 | .
 | .
 | .
 0-| ..............................*..
 ----------------------------------
 -20 -10 0 10 20


*/
void ManVar_LPositive (int __DOM)
 {
 fc_ConsName += ConsVol;
 fa_ConsName += (ConsVol * (ConsPoint));
 }
/* } */
/* FUZZY pid */
void pid (void)
 {
 fa_ConsName = 0;
 fc_ConsName = 0;
/* { */
/* IF Error IS LNegative THEN ManVar IS LPositive */
 ManVar_LPositive( Error_LNegative(Error) );
/* IF Error IS LPositive THEN ManVar IS LNegative */
 ManVar_LNegative( Error_LPositive(Error) );
/* IF Error IS normal AND DeltaError IS Positive */
/* THEN ManVar IS SNegative */
 __IDOM[1u] = Error_normal(Error) ;
 __IDOM[0u] = DeltaError_Positive(DeltaError) ;
 __IDOM[0u] = F_AND(__IDOM[1u],__IDOM[0u]);
 ManVar_SNegative( __IDOM[0u] );
/* IF Error IS normal AND DeltaError IS Negative */
/* THEN ManVar IS SPositive */
 __IDOM[1u] = Error_normal(Error) ;
 __IDOM[0u] = DeltaError_Negative(DeltaError) ;
 __IDOM[0u] = F_AND(__IDOM[1u],__IDOM[0u]);
 ManVar_SPositive( __IDOM[0u] );
/* IF Error IS close AND SumError IS LPos */
/* THEN ManVar IS SNegative */
 __IDOM[1u] = Error_close(Error) ;
 __IDOM[0u] = SumError_LPos(SumError) ;
 __IDOM[0u] = F_AND(__IDOM[1u],__IDOM[0u]);
 ManVar_SNegative( __IDOM[0u] );
/* IF Error IS close AND SumError IS LNeg */
/* THEN ManVar IS SPositive */
 __IDOM[1u] = Error_close(Error) ;
 __IDOM[0u] = SumError_LNeg(SumError) ;
 __IDOM[0u] = F_AND(__IDOM[1],__IDOM[0]);
 ManVar_SPositive( __IDOM[0u] );
/* } */
 ConsName = fa_ConsName / fc_ConsName;
 }

 void RunFuzzyLogic()
 {
   OldError = Error;
   Error = Setpoint - Process();
   DeltaError = Error - OldError;
   SumError := SumError + Error;
   pid();
 }