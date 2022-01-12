#include "definitions.h"
#if defined(ML_AI_NEEDED)                                                       /* ==== machine learning is required */
#if defined(UNIVARIATE_SMOOTH)                                                  /* ==== univariate smooth method ========== */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "univariate_smooth.h"

/* ====================== function definitions ============================== */
float64_t US_func1(float64_t x, uint8_t *valid);
float64_t US_func2(float64_t x, uint8_t *valid);
float64_t US_func3(float64_t x, uint8_t *valid);
float64_t US_func4(float64_t x, uint8_t *valid);
float64_t US_func5(float64_t x, uint8_t *valid);
float64_t US_Diff(US_Cost_func_e cost_func, float64_t x);
void US_RFM(US_Cost_func_e cost_func, float64_t *a, float64_t *b, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid);
void US_Secant(US_Cost_func_e cost_func, float64_t *x1, float64_t *x0, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid);
void US_Newton(US_Cost_func_e cost_func, float64_t *x, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid);
void US_Bisection(US_Cost_func_e cost_func, float64_t *a, float64_t *b, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid);
void US_doUnivariateSmooth(US_method_e meth, US_Cost_func_e cost_func, float64_t *a, float64_t *b, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid);

/*
   Thanks to Young-Chul Yoon for skeleton C++ code
   Ported and written for PIC32 by ACP Aviation
   Smooth univariate methods 
   Bisection
   Newton
   Secant
   RFM
*/
/*-----------------------------------------------------------------------------
 *   US_func1 :  Function1
 *  
 *  Parameters: float64_t x, uint8_t *valid
 *  Return: float64_t  
 *----------------------------------------------------------------------------*/
float64_t US_func1(float64_t x, uint8_t *valid) 
{
        float64_t f = log(x) + sin(x);
        *valid = IsaNan(f);
        return f;
}
/*-----------------------------------------------------------------------------
 *   US_func2 :  Function2
 *  
 *  Parameters: float64_t x, uint8_t *valid
 *  Return: float64_t  
 *----------------------------------------------------------------------------*/
float64_t US_func2(float64_t x, uint8_t *valid) 
{
        float64_t f = pow(x, 2.0f) + 3.0f * x + 1.0f;
        *valid = IsaNan(f);
        return f;
}
/*-----------------------------------------------------------------------------
 *   US_func3 :  Function3
 *  
 *  Parameters: float64_t x, uint8_t *valid
 *  Return: float64_t  
 *----------------------------------------------------------------------------*/
float64_t US_func3(float64_t x, uint8_t *valid) 
{
        float64_t f = pow(x, 5.0f) + 2.0f * pow(x, 3.0f) + exp(pow(x, 2.0f) + x);
        *valid = IsaNan(f);
        return f;
}
/*-----------------------------------------------------------------------------
 *   US_func4 :  Function4
 *  
 *  Parameters: float64_t x, uint8_t *valid
 *  Return: float64_t  
 *----------------------------------------------------------------------------*/
float64_t US_func4(float64_t x, uint8_t *valid) 
{
        float64_t f = cos(pow(x, 2.0f) + x);
        *valid = IsaNan(f);
        return f;
}
/*-----------------------------------------------------------------------------
 *   US_func5 :  Function5
 *  
 *  Parameters: float64_t x, uint8_t *valid
 *  Return: float64_t  
 *----------------------------------------------------------------------------*/
float64_t US_func5(float64_t x, uint8_t *valid) 
{
        float64_t f = pow(x, 3.0f) + log(pow(x, 3.0f)) + 1.0f;
        *valid = IsaNan(f);
        return f;
}

/*-----------------------------------------------------------------------------
 *   US_Diff :  Differentiation function
 *  
 *  Parameters: US_Cost_func_e cost_func, float64_t x, uint8_t *isValid
 *  Return: float64_t  
 *----------------------------------------------------------------------------*/
float64_t US_Diff(US_Cost_func_e cost_func, float64_t x, uint8_t *isValid) 
{
   float64_t eps = 0.0000001f;
   float64_t resVal = -1.0f;

   switch (cost_func)
   {
        case US_1:
            resVal = (US_func1((x + eps), isValid) - US_func1(x, isValid)) / eps;
                
        case US_2:
            resVal = (US_func2((x + eps), isValid) - US_func2(x,isValid)) / eps;                

        case US_3:
            resVal = (US_func3((x + eps), isValid) - US_func3(x,isValid)) / eps;
                
        case US_4:
            resVal = (US_func4((x + eps), isValid) - US_func4(x,isValid)) / eps;        

        case US_5:
            resVal = (US_func5((x + eps), isValid) - US_func5(x,isValid)) / eps;        

        default:
            resVal = 0.0f;
    }
    return resVal;                
}

/*                         Regula Falsi method
        ========================================================
        *   f = function
        *   a = recent_x
        *   b = prev_x
        *   iter = number of iteration
        *   recall = set true means call again recursive
*/
/*-----------------------------------------------------------------------------
 *   US_RFM :  Regula Falsi method
 *  
 *  Parameters: US_Cost_func_e cost_func, float64_t *a, float64_t *b, int16_t *iter, 
 *              int8_t *recall, float64_t *outVal, uint8_t *isValid
 *
 *  Return: void  
 *----------------------------------------------------------------------------*/
void US_RFM(US_Cost_func_e cost_func, float64_t *a, float64_t *b, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid) 
{
    float64_t new_x;
        
    switch (cost_func)
    {
          case US_1:        
          if ((*iter == 0) && ((US_func1(*a,isValid)*US_func1(*b,isValid)) > 0)) 
          {
                *recall = false;                                                /* do not call this function again */
                *outVal = -1.0f;                                                /* error */
          }
          new_x = *a - (US_func1(*a,isValid) / ((US_func1(*a,isValid) - US_func1(*b,isValid)) / (*a - *b)));
          if (US_func1(new_x,isValid) == 0 || abs(*a - new_x) < 0.0001f) 
          {
                *recall = false;                                                /* do not call this function again */
                *outVal = new_x;
          }
          else if ((US_func1(*a,isValid)*US_func1(new_x,isValid)) < 0.0f)
          {
                *iter+=1;
                *a = new_x;
                *b = *a;
                *recall = true;                                                 /* call this function again */
          }
          else
          {
                *iter+=1;
                *a = new_x;
                *recall = true;                                                 /* call this function again */
          }
          
          case US_2:        
          if ((*iter == 0) && ((US_func2(*a,isValid)*US_func2(*b,isValid)) > 0)) 
          {
                *recall = false;                                                /* do not call this function again */
                *outVal = -1.0f;                                                /* error */
          }
          new_x = *a - (US_func2(*a,isValid) / ((US_func2(*a,isValid) - US_func2(*b,isValid)) / (*a - *b)));
          if (US_func2(new_x,isValid) == 0 || abs(*a - new_x) < 0.0001f) 
          {
                *recall = false;                                                /* do not call this function again */
                *outVal = new_x;
          }
          else if ((US_func2(*a,isValid)*US_func2(new_x,isValid)) < 0.0f)
          {
                *iter+=1;
                *a = new_x;
                *b = *a;
                *recall = true;                                                 /* call this function again */
          }
          else
          {
                *iter+=1;
                *a = new_x;
                *recall = true;                                                 /* call this function again */
          }
          
          case US_3:        
          if ((*iter == 0) && ((US_func3(*a,isValid)*US_func3(*b,isValid)) > 0)) 
          {
                *recall = false;                                                /* do not call this function again */
                *outVal = -1.0f;                                                /* error */
          }
          new_x = *a - (US_func3(*a,isValid) / ((US_func3(*a,isValid) - US_func3(*b,isValid)) / (*a - *b)));
          if (US_func3(new_x,isValid) == 0 || abs(*a - new_x) < 0.0001f) 
          {
                *recall = false;                                                /* do not call this function again */
                *outVal = new_x;
          }
          else if ((US_func3(*a,isValid)*US_func3(new_x,isValid)) < 0.0f)
          {
                *iter+=1;
                *a = new_x;
                *b = *a;
                *recall = true;                                                 /* call this function again */
          }
          else
          {
                *iter+=1;
                *a = new_x;
                *recall = true;                                                 /* call this function again */
          }
          
          case US_4:        
          if ((*iter == 0) && ((US_func4(*a,isValid)*US_func4(*b,isValid)) > 0)) 
          {
                *recall = false;                                                /* do not call this function again */
                *outVal = -1.0f;                                                /* error */
          }
          new_x = *a - (US_func4(*a,isValid) / ((US_func4(*a,isValid) - US_func4(*b,isValid)) / (*a - *b)));
          if (US_func4(new_x,isValid) == 0 || abs(*a - new_x) < 0.0001f) 
          {
                *recall = false;                                                /* do not call this function again */
                *outVal = new_x;
          }
          else if ((US_func4(*a,isValid)*US_func4(new_x,isValid)) < 0.0f)
          {
                *iter+=1;
                *a = new_x;
                *b = *a;
                *recall = true;                                                 /* call this function again */
          }
          else
          {
                *iter+=1;
                *a = new_x;
                *recall = true;                                                 /* call this function again */
          }
          
          case US_5:        
          if ((*iter == 0) && ((US_func5(*a,isValid)*US_func5(*b,isValid)) > 0)) 
          {
                *recall = false;                                                /* do not call this function again */
                *outVal = -1.0f;                                                /* error */
          }
          new_x = *a - (US_func5(*a,isValid) / ((US_func5(*a,isValid) - US_func5(*b,isValid)) / (*a - *b)));
          if (US_func5(new_x,isValid) == 0 || abs(*a - new_x) < 0.0001f) 
          {
                *recall = false;                                                /* do not call this function again */
                *outVal = new_x;
          }
          else if ((US_func5(*a,isValid)*US_func5(new_x,isValid)) < 0.0f)
          {
                *iter+=1;
                *a = new_x;
                *b = *a;
                *recall = true;                                                 /* call this function again */
          }
          else
          {
                *iter+=1;
                *a = new_x;
                *recall = true;                                                 /* call this function again */
          }
          
          default:
          break;
    }
    return;
}

/* Secant method
        *        f = function
        *        x1 = recent_x
        *        x0 = prev_x
        *        iter = number of iteration
        *   recall = true pass through iteration again
        *   outVal = result
        *   isValid = false reading is not a NAN
*/
/*-----------------------------------------------------------------------------
 *   US_Secant:  Secant method
 *  
 *  Parameters: US_Cost_func_e cost_func, float64_t *x1, float64_t *x0, int16_t *iter,  
 *              int8_t *recall, float64_t *outVal, uint8_t *isValid
 *
 *  Return: void  
 *----------------------------------------------------------------------------*/
void US_Secant(US_Cost_func_e cost_func, float64_t *x1, float64_t *x0, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid) 
{
        float64_t new_x = *x1 - (US_func1(*x1,isValid) / ((US_func1(*x1,isValid) - US_func1(*x0,isValid)) / (*x1 - *x0)));
        
        switch (cost_func)
        {
            case US_1:
            if (US_func1(new_x,isValid) == 0 || abs(*x1 - new_x) < 0.0001f) 
            {
                  *outVal = new_x;
                  *recall = false;
            }
            else
            {
                   *x1 = new_x;
                   *x0 = *x1;
                   *recall = true;
                   *iter+=1;
            }
            break;
            
            case US_2:
            if (US_func2(new_x,isValid) == 0 || abs(*x1 - new_x) < 0.0001f) 
            {
                  *outVal = new_x;
                  *recall = false;
            }
            else
            {
                   *x1 = new_x;
                   *x0 = *x1;
                   *recall = true;
                   *iter+=1;
            }
            break;
            
            case US_3:
            if (US_func3(new_x,isValid) == 0 || abs(*x1 - new_x) < 0.0001f) 
            {
                  *outVal = new_x;
                  *recall = false;
            }
            else
            {
                   *x1 = new_x;
                   *x0 = *x1;
                   *recall = true;
                   *iter+=1;
            }
            break;
            
            case US_4:
            if (US_func4(new_x,isValid) == 0 || abs(*x1 - new_x) < 0.0001f) 
            {
                  *outVal = new_x;
                  *recall = false;
            }
            else
            {
                   *x1 = new_x;
                   *x0 = *x1;
                   *recall = true;
                   *iter+=1;
            }
            break;
            
            case US_5:
            if (US_func5(new_x,isValid) == 0 || abs(*x1 - new_x) < 0.0001f) 
            {
                  *outVal = new_x;
                  *recall = false;
            }
            else
            {
                   *x1 = new_x;
                   *x0 = *x1;
                   *recall = true;
                   *iter+=1;
            }
            break;
            
            default:
            break;
            
    }
}
/*-----------------------------------------------------------------------------
 *   US_Newton:  Newton method
 *  
 *  Parameters: US_Cost_func_e cost_func, float64_t *x, int16_t *iter, int8_t *recall, 
 *              float64_t *outVal, uint8_t *isValid 
 *
 *  Return: void  
 *----------------------------------------------------------------------------*/
void US_Newton(US_Cost_func_e cost_func, float64_t *x, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid) 
{
        float64_t new_x = *x - US_func1(*x,isValid) / US_Diff(cost_func, *x, isValid);;
        
        switch (cost_func)
        {
            case US_1:
            if (US_func1(new_x,isValid) == 0 || abs(*x - new_x) < 0.0001f) 
            {
                  *outVal = new_x;
                  *recall = false;
            }
            else
            {
                   *x = new_x;
                   *recall = true;
                   *iter+=1;
            }
            break;
            
            case US_2:
            if (US_func2(new_x,isValid) == 0 || abs(*x - new_x) < 0.0001f) 
            {
                  *outVal = new_x;
                  *recall = false;
            }
            else
            {
                   *x = new_x;
                   *recall = true;
                   *iter+=1;
            }
            break;
            
            case US_3:
            if (US_func3(new_x,isValid) == 0 || abs(*x - new_x) < 0.0001f) 
            {
                  *outVal = new_x;
                  *recall = false;
            }
            else
            {
                   *x = new_x;
                   *recall = true;
                   *iter+=1;
            }
            break;
            
            case US_4:
            if (US_func4(new_x,isValid) == 0 || abs(*x - new_x) < 0.0001f) 
            {
                  *outVal = new_x;
                  *recall = false;
            }
            else
            {
                   *x = new_x;
                   *recall = true;
                   *iter+=1;
            }
            break;
            
            case US_5:
            if (US_func5(new_x,isValid) == 0 || abs(*x - new_x) < 0.0001f) 
            {
                  *outVal = new_x;
                  *recall = false;
            }
            else
            {
                   *x = new_x;
                   *recall = true;
                   *iter+=1;
            }
            break;
            
            default:
            break;
            
    }
}
/* Bisection method
        *        f = function
        *        a = first_x
        *        b = second x
        *        iter = number of iteration
*/
/*-----------------------------------------------------------------------------
 *   US_Bisection:  Bisection method
 *  
 *  Parameters: US_Cost_func_e cost_func, float64_t *a, float64_t *b, int16_t *iter, 
 *              int8_t *recall, float64_t *outVal, uint8_t *isValid 
 *
 *  Return: void  
 *----------------------------------------------------------------------------*/
void US_Bisection(US_Cost_func_e cost_func, float64_t *a, float64_t *b, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid) 
{
    float64_t new_x = (*a + *b) / 2.0f;

    switch (cost_func)
    {
        case US_1:                
        if (iter == 0 && (US_func1(*a,isValid)*US_func1(*b,isValid) > 0.0f)) 
        {
            return;
        }
        else if (US_func1(new_x,isValid) == 0 || abs(new_x - *a) < 0.0001f)
        {
            *outVal = new_x;
        }
        else if ((US_func1(*a,isValid)*US_func1(new_x,isValid)) < 0.0f)
        {
            *a = new_x;
            *b = *a; 
            *iter+=1;
         }
         else
         {
            *a = new_x;
            *iter+=1;
         }
         break;
         
         default:
         break;
    }
}
/*-----------------------------------------------------------------------------
 *   US_doUnivariateSmooth:  Bisection method
 *  
 *  Parameters: US_method_e meth, US_Cost_func_e cost_func, float64_t *a, float64_t *b, 
 *              int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid  
 *
 *  Return: void  
 *----------------------------------------------------------------------------*/
void US_doUnivariateSmooth(US_method_e meth, US_Cost_func_e cost_func, float64_t *a, float64_t *b, int16_t *iter, int8_t *recall, float64_t *outVal, uint8_t *isValid)
{
   switch(meth)
   {
      case BISECTION:
      US_Bisection(cost_func,a,b,iter,recall,outVal,isValid); 
      if (*recall == false)
      {
         meth = COMPLETE;
      }
      break;

      case NEWTON:
      US_Newton(cost_func,a,iter,recall,outVal,isValid); 
      if (*recall == false)
      {
         meth = COMPLETE;
      }
      break;
      
      case SECANT:
      US_Secant(cost_func,a,b,iter,recall,outVal,isValid); 
      if (*recall == false)
      {
         meth = COMPLETE;
      }
      break;
      
      case RFM:
      US_RFM(cost_func,a,b,iter,recall,outVal,isValid); 
      if (*recall == false)
      {
         meth = COMPLETE;
      }
      break;  
      
      case COMPLETE:
      break;
      
      default:
      break;         
   }
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
#endif /* requirements were univariate smooth */