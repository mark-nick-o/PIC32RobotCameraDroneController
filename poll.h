#ifndef __POLL_LIB
#define __POLL_LIB

#ifdef __cplusplus
extern "C"
{
#endif

#define INTERVAL_16_MSEC 16u
#define INTERVAL_50_MSEC 50u
#define INTERVAL_500_MSEC 500u

typedef struct
{
   uint8_t interval;                                                            /* How often to call the task */
   void (*proc)(void);                                                          /* pointer to function returning void doesnt work ?? */
   void (*function)(void*);
   void* arg;
} timed_task_t;                                                                 /* poll this task every interval time */

const char * fnA();                                                                // Example function prototype
const char * fnB();                                                                // Example function prototype
const char * fnC();                                                                // Example function prototype

static timed_task_t poll_task[4u] = { { INTERVAL_16_MSEC,  fnA }, { INTERVAL_50_MSEC,  fnB }, { INTERVAL_500_MSEC, fnC }, { 0, NULL } };
                                                                    // end the poll table
const CHAR *fnA(void)
{
    return 'a';
}

const CHAR *fnB(void)
{
    return 'b';
}

const CHAR *fnC(void)
{
    return 'c';
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif