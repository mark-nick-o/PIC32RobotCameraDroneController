#ifndef _terrain_follow_H
#define _terrain_follow_H

#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define TERFOLLOWPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define TERFOLLOWPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define TERFOLLOWPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define TERFOLLOWPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define TERFOLLOWPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

#define TER_V_CONST 0.95f                                                       /* weighting factor on deviation in verticle direction */
#define TER_F_CONST 0.96f                                                       /* weighting factor on deviation in front direction */
#define TER_B_CONST 0.87f                                                       /* weighting factor on deviation in back direction */
#define TER_L_CONST 0.96f                                                       /* weighting factor on deviation in left direction */
#define TER_R_CONST 0.97f                                                       /* weighting factor on deviation in right direction */

#define TER_NUM_OF_MTRS 10u                                                     /* define the maximum number of motors in the craft */

typedef enum {
  TER_ADD = 1u,
  TER_SUB
} instructSpt_e;                                                                /* instruction to setpoint */

typedef enum {
  TER_FRONT_AND_BACK = 1u,
  TER_LEFT_AND_RIGHT,
  TER_ACTION_DONE
} controlAction_e;                                                              /* calculated control action  */

typedef enum {
  TER_OBJ_GO_DOWN = 1u,
  TER_OBJ_GO_UP,
  TER_OBJ_GO_LEFT,
  TER_OBJ_GO_RIGHT,
  TER_OBJ_GO_FWD,
  TER_OBJ_GO_BACK,
  TER_OBJ_REMAIN
} terrainState_e;                                                               /* defines a terrain following expert object */

#if defined(D_FT900)
typedef struct TERFOLLOWPACKED {
        float32_t vert_pos;                                                     /* verticle height from liddar or altitude or both */
        float32_t vert_spt_pos;                                                 /* required height from the ground */
        terrainState_e vert_state :8u;                                          /* requested state for the verticle position of the craft */
        float32_t vdelta;                                                       /* deadband in verticle direction */
        float32_t vChange;                                                      /* ammount to change vertical setpoint */
        float32_t front_pos;                                                    /* front distance from liddar */
        float32_t front_spt_pos;                                                /* required distance from the front */
        terrainState_e fwd_state :8u;                                           /* requested state for the forward position of the craft */
        float32_t fdelta;                                                       /* deadband in front direction */
        float32_t fChange;                                                      /* ammount to change front setpoint */
        float32_t left_pos;                                                     /* left distance from liddar or position sensor */
        float32_t left_spt_pos;                                                 /* required distance from the left */
        terrainState_e left_state :8u;                                          /* requested state for the left position of the craft */
        float32_t ldelta;                                                       /* deadband in left direction */
        float32_t lChange;                                                      /* ammount to change left setpoint */
        float32_t right_pos;                                                    /* right distance from liddar or position sensor */
        float32_t right_spt_pos;                                                /* required distance from the right */
        terrainState_e right_state :8u;                                         /* requested state for the right position of the craft */
        float32_t rdelta;                                                       /* deadband in left direction */
        float32_t rChange;                                                      /* ammount to change right setpoint */
        float32_t back_pos;                                                     /* back distance from liddar or position sensor (object is deemed too close) */
        float32_t back_spt_pos;                                                 /* required distance from the back */
        terrainState_e back_state :8u;                                          /* requested state for the back position of the craft */
        float32_t bdelta;                                                       /* deadband in back direction */
        float32_t bChange;                                                      /* ammount to change back setpoint */
} terrainObj_t;
#else
TERFOLLOWPACKED(
typedef struct {
        float32_t vert_pos;                                                     /* verticle height from liddar or altitude or both */
        float32_t vert_spt_pos;                                                 /* required height from the ground */
        terrainState_e vert_state :8u;                                          /* requested state for the verticle position of the craft */
        float32_t vdelta;                                                       /* deadband in verticle direction */
        float32_t vChange;                                                      /* ammount to change vertical setpoint */
        float32_t front_pos;                                                    /* front distance from liddar */
        float32_t front_spt_pos;                                                /* required distance from the front */
        terrainState_e fwd_state :8u;                                           /* requested state for the forward position of the craft */
        float32_t fdelta;                                                       /* deadband in front direction */
        float32_t fChange;                                                      /* ammount to change front setpoint */
        float32_t left_pos;                                                     /* left distance from liddar or position sensor */
        float32_t left_spt_pos;                                                 /* required distance from the left */
        terrainState_e left_state :8u;                                          /* requested state for the left position of the craft */
        float32_t ldelta;                                                       /* deadband in left direction */
        float32_t lChange;                                                      /* ammount to change left setpoint */
        float32_t right_pos;                                                    /* right distance from liddar or position sensor */
        float32_t right_spt_pos;                                                /* required distance from the right */
        terrainState_e right_state :8u;                                         /* requested state for the right position of the craft */
        float32_t rdelta;                                                       /* deadband in left direction */
        float32_t rChange;                                                      /* ammount to change right setpoint */
        float32_t back_pos;                                                     /* back distance from liddar or position sensor (object is deemed too close) */
        float32_t back_spt_pos;                                                 /* required distance from the back */
        terrainState_e back_state :8u;                                          /* requested state for the back position of the craft */
        float32_t bdelta;                                                       /* deadband in back direction */
        float32_t bChange;                                                      /* ammount to change back setpoint */
}) terrainObj_t;
#endif

void followTerrain( terrainObj_t *terObj );
void modifyMotorSetpoints( float32_t sptChange, instructSpt_e action, pidBlock_t *pidBlk );
controlAction_e chooseSetpoints( uint8_t motorsUp[CRAFT_NUM_OF_MTRS], uint8_t motorsDwn[CRAFT_NUM_OF_MTRS], uint8_t motorsRight[CRAFT_NUM_OF_MTRS], uint8_t motorsLeft[CRAFT_NUM_OF_MTRS], uint8_t mtrsFwd[CRAFT_NUM_OF_MTRS], uint8_t mtrsBack[CRAFT_NUM_OF_MTRS], terrainObj_t *terObj, pidBlock_t *pidBlk[CRAFT_NUM_OF_MTRS] );

/*-----------------------------------------------------------------------------
 *      followTerrain :  Look at position and decide on action for movement
 *  Parameters: terrainObj_t *terObj
 *
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void followTerrain( terrainObj_t *terObj )
{
   /* control vertical position follow terrain on ground */
   if ( (terObj->vert_pos+terObj->vdelta) > terObj->vert_spt_pos )              /* current position is above desired */
   {
      terObj->vert_state = TER_OBJ_GO_DOWN;                                     /* set the state to go down faster on top */
      terObj->vChange = TER_V_CONST*(terObj->vert_spt_pos - (terObj->vert_pos+terObj->vdelta)); /* calculate deviation */
   }
   else if ( (terObj->vert_pos-terObj->vdelta) < terObj->vert_spt_pos )         /* current position is below desired */
   {
       terObj->vert_state = TER_OBJ_GO_UP;                                      /* set the state to go down faster on bottom */
      terObj->vChange = TER_V_CONST*(terObj->vert_spt_pos - (terObj->vert_pos-terObj->vdelta)); /* calculate deviation */
   }
   else                                                                         /* current position is within desired band */
   {
       terObj->vert_state = TER_OBJ_REMAIN;                                     /* set the state to reamin at the current settings */
   }
   
   /* control forward position follow object */
   if ( (terObj->front_pos+terObj->fdelta) > terObj->front_spt_pos )            /* current position is closer than desired */
   {
      terObj->fwd_state = TER_OBJ_GO_BACK;                                      /* set the state to go back slow down */
      terObj->fChange = TER_F_CONST*((terObj->front_pos+terObj->fdelta) - terObj->front_spt_pos); /* calculate deviation */
   }
   else if ( (terObj->front_pos) < (terObj->front_spt_pos-terObj->fdelta) )     /* current position is further than desired */
   {
      terObj->fwd_state = TER_OBJ_GO_FWD;                                       /* set the state to go foward speed up */
      terObj->fChange = TER_F_CONST*((terObj->front_spt_pos-terObj->fdelta) - (terObj->front_pos)); /* calculate deviation */
   }
   else                                                                         /* current position is within desired band */
   {
       terObj->fwd_state = TER_OBJ_REMAIN;                                      /* set the state to reamin at the current settings */
   }

   /* control back position avoid obstacle approaching */
   if ( (terObj->back_pos) < (terObj->back_spt_pos+terObj->bdelta) )            /* @ current position it is closer than desired */
   {
       terObj->back_state = TER_OBJ_GO_FWD;                                      /* set the state to go foward speed up */
      terObj->fChange = TER_B_CONST*((terObj->back_spt_pos+terObj->bdelta)-(terObj->back_pos)); /* calculate deviation */
   }
   else                                                                         /* current position is within desired band */
   {
       terObj->back_state = TER_OBJ_REMAIN;                                     /* set the state to reamin at the current settings */
   }
   
   /* control left side position avoid obstacle at side */
   if ( (terObj->left_pos) < (terObj->left_spt_pos+terObj->ldelta) )            /* current position is closer to left object than desired */
   {
      terObj->left_state = TER_OBJ_GO_RIGHT;                                    /* set the state to go right faster on left */
      terObj->lChange = TER_L_CONST*( (terObj->left_spt_pos+terObj->ldelta) - (terObj->left_pos) ); /* calculate deviation */
   }
   else if ( (terObj->left_pos) > (terObj->left_spt_pos+(2*terObj->ldelta)) )   /* current position is further right than desired */
   {
      terObj->left_state = TER_OBJ_GO_LEFT;                                     /* set the state to go left (faster on right) */
      terObj->lChange = TER_L_CONST*( (terObj->left_pos) - (terObj->left_spt_pos+(2*terObj->ldelta)) ); /* calculate deviation */
   }
   else                                                                         /* current position is within desired band */
   {
      terObj->left_state = TER_OBJ_REMAIN;                                      /* set the state to reamin at the current settings */
   }
   
   /* control right side position avoid obstacle at side */
   if ( (terObj->right_pos+terObj->ldelta) > terObj->right_spt_pos )            /* current position is closer to right object than desired */
   {
      terObj->right_state = TER_OBJ_GO_LEFT;                                    /* set the state to go left (further away) faster on right */
      terObj->rChange = TER_R_CONST*( (terObj->right_pos+terObj->ldelta) - terObj->right_spt_pos ); /* calculate deviation */
   }
   else if ( (terObj->right_pos) < (terObj->right_spt_pos-terObj->ldelta) )     /* current position is further left than desired */
   {
      terObj->right_state = TER_OBJ_GO_RIGHT;                                   /* set the state to go right (closer) faster on the left */
      terObj->rChange = TER_R_CONST*( (terObj->right_spt_pos-terObj->ldelta) - (terObj->right_pos) ); /* calculate deviation */
   }
   else                                                                         /* current position is within desired band */
   {
       terObj->right_state = TER_OBJ_REMAIN;                                    /* set the state to reamin at the current settings */
   }
   
}

/*-----------------------------------------------------------------------------
 *      modifyMotorSetpoints :  Change setpoint of motor No by ammount in 
 *                              direction specified
 *  Parameters: float32_t sptChange, instructSpt_e action, pidBlock_t *pidBlk
 *
 *  Return:
 *         none
 *----------------------------------------------------------------------------*/
void modifyMotorSetpoints( float32_t sptChange, instructSpt_e action, pidBlock_t *pidBlk )
{
     switch (action)
     {
        case TER_ADD:
        pidBlk->rem_setpoint = pidBlk->rem_setpoint + ((int32_t)sptChange);
        break;
        
        case TER_SUB:
        pidBlk->rem_setpoint = pidBlk->rem_setpoint - ((int32_t)sptChange);
        break;
        
        default:
        break;
     }
}

/*-----------------------------------------------------------------------------
 *  chooseSetpoints :  Choose the motors in the craft to have their spt modified
 *                         depending on the terrain object
 *  Parameters: uint8_t motorsUp[10u],uint8_t motorsDwn[10u],uint8_t motorsRight[10u],
 *              uint8_t motorsLeft[10u],uint8_t mtrsFwd[10u],uint8_t mtrsBack[10u],
 *              terrainObj_t *terObj
 *
 *  Each array contains the motor numbers that are related to it
 *  e.g. motors on top or bottom on right or left on front or back
 *       there can be motors not in the group but each group must be mutually
 *       exclusive
 *       {top and bottom} {left and right} {front and back}
 *
 *  Return:
 *         controlAction_e : enum representing the action taken
 *----------------------------------------------------------------------------*/
controlAction_e chooseSetpoints( uint8_t motorsUp[CRAFT_NUM_OF_MTRS], uint8_t motorsDwn[CRAFT_NUM_OF_MTRS], uint8_t motorsRight[CRAFT_NUM_OF_MTRS], uint8_t motorsLeft[CRAFT_NUM_OF_MTRS], uint8_t mtrsFwd[CRAFT_NUM_OF_MTRS], uint8_t mtrsBack[CRAFT_NUM_OF_MTRS], terrainObj_t *terObj, pidBlock_t *pidBlk[CRAFT_NUM_OF_MTRS] )
{
   uint8_t current_motor;                                                       /* motor number we are looking at */
   
   /* first check that we dont have an error with the sensors or an impossible situation */
   if ((terObj->back_state == TER_OBJ_GO_FWD) && (terObj->fwd_state == TER_OBJ_GO_BACK)) /* something at back and front then do nothing and alarm it */
   {
      return TER_FRONT_AND_BACK;                                                /* do nothing but consider looking at magnitude and deciding which is worse */
   }
   else if ((terObj->left_state == TER_OBJ_GO_LEFT) && (terObj->right_state == TER_OBJ_GO_RIGHT)) /* something left and right do nothing and alarm it */
   {
      return TER_LEFT_AND_RIGHT;                                                /* do nothing but consider looking at magnitude and deciding which is worse */
   }
   else if ((terObj->left_state == TER_OBJ_GO_RIGHT) && (terObj->right_state == TER_OBJ_GO_LEFT)) /* something left and right do nothing and alarm it */
   {
      return TER_LEFT_AND_RIGHT;                                                /* do nothing but consider looking at magnitude and deciding which is worse */
   }
   else
   {
        switch(terObj->vert_state)                                              /* check the vertical state of the craft */
        {
            case TER_OBJ_GO_DOWN:
            for (current_motor=1u;current_motor<=TER_NUM_OF_MTRS;current_motor++)
            {
                if (motorsDwn[current_motor] == current_motor)                  /* motor number matches down list (on bottom) */
                {
                   modifyMotorSetpoints( terObj->vChange, TER_SUB, pidBlk[current_motor-1u] );   /* modify the motor setpoints on bottom to reduce speed */
                }
                if (motorsUp[current_motor] == current_motor)                   /* motor number matches up list (on top) */
                {
                   modifyMotorSetpoints( terObj->vChange, TER_ADD, pidBlk[current_motor-1u] );   /* modify the motor setpoints on top to increase speed */
                }
            }
            break;
            
            case TER_OBJ_GO_UP:
            for (current_motor=1u;current_motor<=TER_NUM_OF_MTRS;current_motor++)
            {
                if (motorsUp[current_motor] == current_motor)                   /* motor number matches up list (on top) */
                {
                   modifyMotorSetpoints( terObj->vChange, TER_SUB, pidBlk[current_motor-1u] );   /* modify the motor setpoints on top to reduce speed */
                }
                if (motorsDwn[current_motor] == current_motor)                  /* motor number matches down list (on bottom) */
                {
                   modifyMotorSetpoints( terObj->vChange, TER_ADD, pidBlk[current_motor-1u] );   /* modify the motor setpoints on bottom to increase speed */
                }
            }
            break;
            
            default:                                                            /* No Action */
            break;
        }

        switch(terObj->left_state)                                              /* check the left state of the craft */
        {
            case TER_OBJ_GO_LEFT:
            for (current_motor=1u;current_motor<=TER_NUM_OF_MTRS;current_motor++)
            {
                if (motorsLeft[current_motor] == current_motor)                 /* motor number matches left */
                {
                   modifyMotorSetpoints( terObj->lChange, TER_SUB, pidBlk[current_motor-1u] );   /* modify the motor setpoints on left to reduce speed */
                }
                if (motorsRight[current_motor] == current_motor)                /* motor number matches right list (on right side) */
                {
                   modifyMotorSetpoints( terObj->lChange, TER_ADD, pidBlk[current_motor-1u] );   /* modify the motor setpoints on right to increase speed */
                }
            }
            break;

            case TER_OBJ_GO_RIGHT:
            for (current_motor=1u;current_motor<=TER_NUM_OF_MTRS;current_motor++)
            {
                if (motorsLeft[current_motor] == current_motor)                 /* motor number matches left (on left side) */
                {
                   modifyMotorSetpoints( terObj->lChange, TER_ADD, pidBlk[current_motor-1u] );   /* modify the motor setpoints on left to increase speed */
                }
                if (motorsRight[current_motor] == current_motor)                /* motor number matches right list (on right side) */
                {
                   modifyMotorSetpoints( terObj->lChange, TER_SUB, pidBlk[current_motor-1u] );   /* modify the motor setpoints on right to decrease speed */
                }
            }
            break;

            default:                                                            /* No Action */
            break;
        }

        switch(terObj->right_state)                                             /* check the right state of the craft */
        {
            case TER_OBJ_GO_LEFT:
            for (current_motor=1u;current_motor<=TER_NUM_OF_MTRS;current_motor++)
            {
                if (motorsLeft[current_motor] == current_motor)                 /* motor number matches left */
                {
                   modifyMotorSetpoints( terObj->rChange, TER_SUB, pidBlk[current_motor-1u] );   /* modify the motor setpoints on left to reduce speed */
                }
                if (motorsRight[current_motor] == current_motor)                /* motor number matches right list (on right side) */
                {
                   modifyMotorSetpoints( terObj->rChange, TER_ADD, pidBlk[current_motor-1u] );   /* modify the motor setpoints on right to increase speed */
                }
            }
            break;

            case TER_OBJ_GO_RIGHT:
            for (current_motor=1u;current_motor<=TER_NUM_OF_MTRS;current_motor++)
            {
                if (motorsLeft[current_motor] == current_motor)                 /* motor number matches left (on left side) */
                {
                   modifyMotorSetpoints( terObj->rChange, TER_ADD, pidBlk[current_motor-1u] );   /* modify the motor setpoints on left to increase speed */
                }
                if (motorsRight[current_motor] == current_motor)                /* motor number matches right list (on right side) */
                {
                   modifyMotorSetpoints( terObj->rChange, TER_SUB, pidBlk[current_motor-1u] );   /* modify the motor setpoints on right to decrease speed */
                }
            }
            break;

            default:                                                            /* No Action */
            break;
        }

        switch(terObj->fwd_state)                                               /* check the forward state of the craft */
        {
            case TER_OBJ_GO_FWD:
            for (current_motor=1u;current_motor<=TER_NUM_OF_MTRS;current_motor++)
            {
                if (mtrsFwd[current_motor] == current_motor)                    /* motor number matches forward on front of craft */
                {
                   modifyMotorSetpoints( terObj->fChange, TER_SUB, pidBlk[current_motor-1u] );   /* modify the motor setpoints on front to reduce speed */
                }
                if (mtrsBack[current_motor] == current_motor)                   /* motor number matches back list (on rear side) */
                {
                   modifyMotorSetpoints( terObj->fChange, TER_ADD, pidBlk[current_motor-1u] );   /* modify the motor setpoints on back to increase speed */
                }
            }
            break;

            case TER_OBJ_GO_BACK:
            for (current_motor=1u;current_motor<=TER_NUM_OF_MTRS;current_motor++)
            {
                if (mtrsFwd[current_motor] == current_motor)                    /* motor number matches forward on front of craft */
                {
                   modifyMotorSetpoints( terObj->fChange, TER_ADD, pidBlk[current_motor-1u] );   /* modify the motor setpoints on front to increase speed */
                }
                if (mtrsBack[current_motor] == current_motor)                   /* motor number matches back list (on rear side) */
                {
                   modifyMotorSetpoints( terObj->fChange, TER_SUB, pidBlk[current_motor-1u] );   /* modify the motor setpoints on back to decrease speed */
                }
            }
            break;

            default:                                                            /* No Action */
            break;
        }

        switch(terObj->back_state)                                              /* check the back state of the craft */
        {
            case TER_OBJ_GO_FWD:
            for (current_motor=1u;current_motor<=TER_NUM_OF_MTRS;current_motor++)
            {
                if (mtrsFwd[current_motor] == current_motor)                    /* motor number matches forward on front of craft */
                {
                   modifyMotorSetpoints( terObj->bChange, TER_SUB, pidBlk[current_motor-1u] );   /* modify the motor setpoints on front to reduce speed */
                }
                if (mtrsBack[current_motor] == current_motor)                   /* motor number matches back list (on rear side) */
                {
                   modifyMotorSetpoints( terObj->bChange, TER_ADD, pidBlk[current_motor-1u] );   /* modify the motor setpoints on back to increase speed */
                }
            }
            break;
            
            default:
            break;
        }
   
   }
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* end library */