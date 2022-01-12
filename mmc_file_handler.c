#include <stdint.h>
#include "definitions.h"
#include "Struts.h"
#include "ilda.h"
#include "gc_events.h"
#include "mmc_file_handler.h"
/* #include "types.h"     */

int32_t mmcTimeDuration;
uint32_t mmcTimeDurationLast;
int32_t g_mmcTimeDuration;
uint32_t g_mmcTimeDurationLast;
mmc_init_e mmcFileSysState=MMC_SPI_INIT;
mmc_init_e g_mmcStep=MMC_SPI_INIT;
mmc_init_e g_FATprevState=MMC_SPI_INIT;
uint16_t formatRet=0u;

/* MMC module connections */
sbit Mmc_Chip_Select           at LATG9_bit;                                    /* for writing to output pin always use latch */
sbit Mmc_Chip_Select_Direction at TRISG9_bit;
/* eof MMC module connections                                                 */

/* ================ function definitions ==================================== */
void initMMCFileSys();
void mmcCreateSwapFile();
void mmcCreateNewFile( char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp, const rionUTCDateTimeField_t *tim );
void mmcOpenFileRewrite( char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp );
void mmcOpenFileAppend( char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp );
void mmcOpenFileApp2Pos( char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp, uint32_t *posn );
void mmcOpenFileRead( char *filename, mmc_file_buff_t *ReadDat );
uint8_t mmcOpenFileRecord( char *filename, mmc_file_read_t *file_data, mmc_data_typ_e recTyp, uint8_t prod_chosen );
void mmcOpenFileAppRec( char *filename, mmc_file_read_t *file_data, mmc_file_writ_t *writ_data, uint8_t prod_chosen );
uint8_t mmcManageRecords( char *filename, mmc_file_read_t *file_data,  mmc_data_typ_e recTyp, mmc_file_writ_t file_write, gui_spray_data_t *guiReq, const rionUTCDateTimeField_t *tim );
//uint8_t mmcManageRecords( char *filename, mmc_file_read_t *file_data, mmc_data_typ_e recTyp, mmc_file_writ_t file_write );

/*-----------------------------------------------------------------------------
 *      mmcCreateSwapFile():  Tries to create a swap file, whose size will be at
 *                            least 100 sectors
 *  Parameters: none
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void mmcCreateSwapFile()
{
  uint16_t i;
  uint32_t size=0u;

  char Buffer[MMC_SECTOR_SZ];

  for(i=0; i<MMC_SECTOR_SZ; i++)
    Buffer[i] = i;

  size=Mmc_Fat_Get_Swap_File(5000, "mikroE.txt", ~(MMC_FL_ATTR_ARC));           /* create swap file */
  if (size >= 0u)
  {
    for(i=0u; i<5000u; i++)
    {
      Mmc_Write_Sector(size++, Buffer);
    }
  }
}
/*-----------------------------------------------------------------------------
 *      mmcCreateNewFile():  Create new MMC file
 *
 *  Parameters: const char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp
 *              const rionUTCDateTimeField_t *tim
 *
 *              (not const as function isnt double misra violation so just accept warning for const)
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void mmcCreateNewFile( char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp, const rionUTCDateTimeField_t *tim )
{
  uint8_t line_len=0u;
  char file_line[MMC_LINE_LEN];

  if (filename == NULL)                                                         /* null pointer passed */
  {
     return;
  }
  
  Mmc_Fat_Set_File_Date(tim->year,tim->month,tim->day,tim->hour,tim->min,tim->sec);  /* Set file date & time info  */
  if (Mmc_Fat_Assign(filename, ~(MMC_FL_ATTR_ARC & MMC_FL_ATTR_NEW))==1u)       /* Find existing file or create a new one */
  {
     Mmc_Fat_Rewrite();                                                         /* To clear file and start with new data  */
     switch(recTyp)
     {
           case MMC_ILDA_HEAD:                                                  /* laser file start header */
           line_len=sizeof(file_data.filerec.ildahead);
           memcpy((void*) file_line,(void*) &file_data.filerec.ildahead, line_len);
           break;

           case MMC_3D_REC:                                                     /* laser file 3d record */
           line_len=sizeof(file_data.filerec.data3d);
           memcpy((void*) file_line,(void*) &file_data.filerec.data3d, line_len);
           break;

           case MMC_2D_REC:                                                     /* laser file 2d record */
           line_len=sizeof(file_data.filerec.data2d);
           memcpy((void*) file_line,(void*) &file_data.filerec.data2d, line_len);
           break;

           case MMC_CLR_IDX:                                                    /* laser file color index */
           line_len=sizeof(file_data.filerec.colorIdx);
           memcpy((void*) file_line,(void*) &file_data.filerec.colorIdx, line_len);
           break;

           case MMC_TXT_ONLY:                                                   /* 50 char text lines */
           line_len=sizeof(file_data.filerec.textLine);
           memcpy((void*) file_line,(void*) &file_data.filerec.textLine, line_len);
           break;

           case MMC_SPRAY_RCP:                                                  /* batch mixer settings for sprayer data */
           line_len=sizeof(file_data.filerec.spraymix);
           memcpy((void*) file_line,(void*) &file_data.filerec.spraymix, line_len);
           break;

           case MMC_REALTIME_DATA:                                              /* realtime saved data relating to program and used to re-initialise at boot-up */
           line_len=sizeof(file_data.filerec.realTimeData);
           memcpy((void*) file_line,(void*) &file_data.filerec.realTimeData, line_len);
           break;
           
           default:
           break;
     }
     Mmc_Fat_Write(file_line, line_len);                                        /* write data to the assigned file */
  }

}
/*-----------------------------------------------------------------------------
 *      mmcOpenFileRewrite():  Opens an existing file and rewrites it
 *
 *  Parameters: const char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void mmcOpenFileRewrite( char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp )
{
  uint8_t line_len=0u;
  char file_line[MMC_LINE_LEN];

  if (filename==NULL)                                                           /* null pointer */
  {
    return;                                                                     /* exit */
  }
  
  if (Mmc_Fat_Assign(filename, 0u)==1u)                                         /* Find existing file and use it  */
  {
      Mmc_Fat_Rewrite();                                                        /* Opens the currently assigned file for writing. If the file is not empty its content will be erased. */
      switch(recTyp)                                                            /* select the record type chosen by the calling process */
      {
           case MMC_ILDA_HEAD:                                                  /* laser header file */
           line_len=sizeof(file_data.filerec.ildahead);
           memcpy((void*) file_line,(void*) &file_data.filerec.ildahead, sizeof(file_data.filerec.ildahead));
           break;

           case MMC_3D_REC:                                                     /* laser 3d record */
           line_len=sizeof(file_data.filerec.data3d);
           memcpy((void*) file_line,(void*) &file_data.filerec.data3d, sizeof(file_data.filerec.data3d));
           break;

           case MMC_2D_REC:                                                     /* laser 2d record */
           line_len=sizeof(file_data.filerec.data2d);
           memcpy((void*) file_line,(void*) &file_data.filerec.data2d, sizeof(file_data.filerec.data2d));
           break;

           case MMC_CLR_IDX:                                                    /* laser color index */
           line_len=sizeof(file_data.filerec.colorIdx);
           memcpy((void*) file_line,(void*) &file_data.filerec.colorIdx, sizeof(file_data.filerec.colorIdx));
           break;

           case MMC_TXT_ONLY:                                                   /* text file */
           line_len=sizeof(file_data.filerec.textLine);
           memcpy((void*) file_line,(void*) &file_data.filerec.textLine, sizeof(file_data.filerec.textLine));
           break;

           case MMC_SPRAY_RCP:                                                  /* sprayer system recipe and field data */
           line_len=sizeof(file_data.filerec.spraymix);
           memcpy((void*) file_line,(void*) &file_data.filerec.spraymix, line_len);
           break;

           case MMC_REALTIME_DATA:                                              /* save of realtime data for reboot intialisation */
           line_len=sizeof(file_data.filerec.realTimeData);
           memcpy((void*) file_line,(void*) &file_data.filerec.realTimeData, line_len);
           break;
           
           default:
           break;
      }
      Mmc_Fat_Write(file_line, line_len);                                       /* write data to the assigned file */
  }
}
/*-----------------------------------------------------------------------------
 *      mmcOpenFileAppend():  Opens an existing file and appends data to it
 *
 *  Parameters: const char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void mmcOpenFileAppend( char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp )
{
  uint8_t line_len=0u;
  char file_line[MMC_LINE_LEN];
 
  if (filename==NULL)                                                           /* NULL pointer passed */
  {
    return;                                                                     /* exit */
  }
  
  if (Mmc_Fat_Assign(filename, 0u)==1u)                                         /* Find existing file and use it  */
  {
      Mmc_Fat_Append();                                                         /* Prepare file for append */
      switch(recTyp)
      {
          case MMC_ILDA_HEAD:                                                   /* laser header file */
          line_len=sizeof(file_data.filerec.ildahead);
          memcpy((void*) file_line,(void*) &file_data.filerec.ildahead, sizeof(file_data.filerec.ildahead));
          break;

          case MMC_3D_REC:                                                      /* laser 3d record */
          line_len=sizeof(file_data.filerec.data3d);
          memcpy((void*) file_line,(void*) &file_data.filerec.data3d, sizeof(file_data.filerec.data3d));
          break;

          case MMC_2D_REC:                                                      /* laser 2d record */
          line_len=sizeof(file_data.filerec.data2d);
          memcpy((void*) file_line,(void*) &file_data.filerec.data2d, sizeof(file_data.filerec.data2d));
          break;

          case MMC_CLR_IDX:                                                     /* laser color index */
          line_len=sizeof(file_data.filerec.colorIdx);
          memcpy((void*) file_line,(void*) &file_data.filerec.colorIdx, sizeof(file_data.filerec.colorIdx));
          break;

          case MMC_TXT_ONLY:                                                    /* text file */
          line_len=sizeof(file_data.filerec.textLine);
          memcpy((void*) file_line,(void*) &file_data.filerec.textLine, sizeof(file_data.filerec.textLine));
          break;

          case MMC_SPRAY_RCP:                                                   /* sprayer system recipe and field data */
          line_len=sizeof(file_data.filerec.spraymix);
          memcpy((void*) file_line,(void*) &file_data.filerec.spraymix, line_len);
          break;

          default:
          break;
      }
      Mmc_Fat_Write(file_line, line_len);                                       /* write data to the assigned file  */
  }
}
/*-----------------------------------------------------------------------------
 *      mmcOpenFileApp2Pos():  Opens an existing file and appends data to position
 *                             specified in the file
 *
 *  Parameters: const char *filename, mmc_file_writ_t file_data,
 *              mmc_data_typ_e recTyp, const uint32_t *posn
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void mmcOpenFileApp2Pos( char *filename, mmc_file_writ_t file_data, mmc_data_typ_e recTyp, uint32_t *posn )
{
  uint8_t line_len=0u;
  char file_line[MMC_LINE_LEN];
  uint32_t newPos=0u;

  if ((filename==NULL) || (posn==NULL))                                         /* guard NULL pointer */
  {
     return;                                                                    /* exit */
  }
  
  if (Mmc_Fat_Assign(filename, 0u)==1u)                                         /* Find existing file and use it  */
  {
      newPos = Mmc_Fat_Seek(*posn);                                             /* seek to the position in the file */
      Mmc_Fat_Append();                                                         /* Prepare file for append */
      switch(recTyp)
      {
          case MMC_ILDA_HEAD:
          line_len=sizeof(file_data.filerec.ildahead);
          memcpy((void*) file_line,(void*) &file_data.filerec.ildahead, sizeof(file_data.filerec.ildahead));
          break;

          case MMC_3D_REC:
          line_len=sizeof(file_data.filerec.data3d);
          memcpy((void*) file_line,(void*) &file_data.filerec.data3d, sizeof(file_data.filerec.data3d));
          break;

          case MMC_2D_REC:
          line_len=sizeof(file_data.filerec.data2d);
          memcpy((void*) file_line,(void*) &file_data.filerec.data2d, sizeof(file_data.filerec.data2d));
          break;

          case MMC_CLR_IDX:
          line_len=sizeof(file_data.filerec.colorIdx);
          memcpy((void*) file_line,(void*) &file_data.filerec.colorIdx, sizeof(file_data.filerec.colorIdx));
          break;

          case MMC_TXT_ONLY:
          line_len=sizeof(file_data.filerec.textLine);
          memcpy((void*) file_line,(void*) &file_data.filerec.textLine, sizeof(file_data.filerec.textLine));
          break;

          case MMC_SPRAY_RCP:
          line_len=sizeof(file_data.filerec.spraymix);
          memcpy((void*) file_line,(void*) &file_data.filerec.spraymix, line_len);
          break;

          default:
          break;
      }
      Mmc_Fat_Write(file_line, line_len);                                       /* write data to the assigned file  */
  }
}
/*-----------------------------------------------------------------------------
 *      mmcOpenFileRead():  Opens mmc file for reading
 *
 *  Parameters: const char *filename, mmc_file_buff_t *ReadDat
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void mmcOpenFileRead( char *filename, mmc_file_buff_t *ReadDat )
{
  char fCharacter;
  uint8_t i;

  if ((ReadDat==NULL) || (filename==NULL))                                      /* NULL pointer passed */
  {
     return;                                                                    /* exit now */
  }
  
  if (Mmc_Fat_Assign(filename, 0u)==1u)                                         /* Find existing file and use it */
  {
     Mmc_Fat_Reset(&ReadDat->size);                                             /* To read file, procedure returns size of file  */
     if (ReadDat->size <= MMC_FILE_READ_BUFSZ)                                  /* comment out if you dont want to limit the ammount of dynamic memory used by pBuff (max file size) */
     {
        for (i = 1u; i <= ReadDat->size; i++)                                   /* for each byte in the file */
        {
           Mmc_Fat_Read(&fCharacter);                                           /* read the byte from the open file stream */
           *ReadDat->pBuff=fCharacter;                                          /* put data into buffer */
           ReadDat->pBuff++;                                                    /* increment the pointer up to the maximum file size */
        }
     }
  }
}
/*-----------------------------------------------------------------------------
 *      mmcOpenFileRecord():  Opens mmc file looks for record relating to the spray
 *                            system and reads the data
 *
 *  Parameters: char *filename, mmc_file_read_t file_data, mmc_data_typ_e recTyp, 
 *              uint8_t prod_chosen
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
uint8_t mmcOpenFileRecord( char *filename, mmc_file_read_t *file_data, mmc_data_typ_e recTyp, uint8_t prod_chosen )
{
  char fCharacter;
  uint8_t i;
  uint8_t NumOfRecs=0u;
  uint8_t RecLen=0u;
  uint8_t prodIDRead=0u;
  uint32_t posFile=0u;
  uint32_t newPos=0u;
  uint8_t fieldSquare=0u;
  mmc_data_read_state_e dataSetStep=MMC_SPRAY_SEARCH;
  
  if ((filename==NULL) || (file_data==NULL))                                    /* guard against the NULL pointer */
  {
     return MMC_NULL_POINTER;                                                   /* return that we passed a NULL pointer */
  }
  
  if (Mmc_Fat_Assign(filename, 0u)==1u)                                         /* Find existing file and use it */
  {
     Mmc_Fat_Reset(&file_data->fileread.rawdata->size);                         /* To read file, procedure returns size of file */
     switch(recTyp)
     {
     
#if (defined(GPS_POS_SPRAYER) && defined(BATCH_MIXER))                          /* file also contains position spray information as well as batch make-up */
        case MMC_SPRAY_RCP:                                                     /* full recipe request batch plant and position */
        RecLen=sizeof(file_data->fileread.spraymix);                            /* read record size including the productId's */
        if (RecLen>=1u)
           NumOfRecs=file_data->fileread.rawdata->size/RecLen;                  /* so long as record length is non zero calculate the number of records stored in the mmc file */
        dataSetStep=MMC_SPRAY_SEARCH;                                           /* set the read state for the records to search for the one you want */
        while(NumOfRecs>=1u)
        {
           Mmc_Fat_Read(&fCharacter);                                           /* get the next char from the file input stream */
           prodIDRead=fCharacter;                                               /* put data into buffer */
           if (prodIDRead==prod_chosen)                                         /* product request equals one found in the file */
           {
              dataSetStep=MMC_SPRAY_READ_RCP;                                   /* read the recipe data */
           }
           else
           {
              posFile = Mmc_Fat_Tell();                                         /* tell position in the file */
              newPos = Mmc_Fat_Seek(posFile+RecLen);                            /* advance to the next record in the mmc file */
              NumOfRecs=--NumOfRecs;                                            /* decrement the number of records */
           }
           
           switch(dataSetStep)                                                  /* state engine for reading records from the file stream */
           {
              case MMC_SPRAY_SEARCH:                                            /* keep searching */
              break;
              
              case MMC_SPRAY_READ_RCP:                                          /* read the batch recipe data from the mmc file */
              RecLen=sizeof(file_data->fileread.spraymix->recipe);              /* size of a recipe field (parameters for the batch mixing sequence) */
              if (RecLen<=file_data->fileread.rawdata->size)                    /* we have at least one record left in the file stream */
              {
                 for (i = 1u; i <= RecLen; i++)                                 /* for each characture up to the record size */
                 {
                    Mmc_Fat_Read(&fCharacter);                                  /* read the char from the chosen file input stream */
                    *file_data->fileread.rawdata->pBuff=fCharacter;             /* put data into buffer */
                    file_data->fileread.rawdata->pBuff++;                       /* increment the pointer to the buffer holding the data to get copied into the structure */
                 }
                 memcpy((void*) &file_data->fileread.spraymix->recipe,(void*) &file_data->fileread.rawdata->pBuff, RecLen); /* copy the buffer to the recipe structure */
                 file_data->fileread.rawdata->size=file_data->fileread.rawdata->size-RecLen; /* reduce the number of bytes left in the file by the record length */
                 dataSetStep=MMC_SPRAY_READ_POS;                                /* now read the spray posiitons from the mmc file */
              }
              else
              {
                 return MMC_INCOMPLETE_FILE;                                    /* incomplete file */
              }
              
              case MMC_SPRAY_READ_POS:                                          /* read the batch recipe data from the mmc file */
              RecLen=sizeof(file_data->fileread.spraymix->field);               /* size of a field square (co-ordinates of square and spray rate setting from picture scan) */
              if (RecLen<=file_data->fileread.rawdata->size)                    /* we have at least one record left in the file stream */
              {
                 for(fieldSquare=0u;fieldSquare<=NO_SPRAY_OF_AREAS;fieldSquare++)
                 {
                    for (i = 1u; i <= RecLen; i++)                              /* for each characture up to the record size */
                    {
                       Mmc_Fat_Read(&fCharacter);                               /* read the char from the chosen file input stream */
                       *file_data->fileread.rawdata->pBuff=fCharacter;          /* put data into buffer */
                       file_data->fileread.rawdata->pBuff++;                    /* increment the pointer to the buffer holding the data to get copied into the structure */
                    }
                    memcpy((void*) &file_data->fileread.spraymix->field[fieldSquare],(void*) &file_data->fileread.rawdata->pBuff, RecLen); /* copy each field square information into each piece of the field structure */
                    file_data->fileread.rawdata->size=file_data->fileread.rawdata->size-RecLen; /* reduce the number of bytes left in the file by the record length */
                 }
                 dataSetStep=MMC_SPRAY_READ_COMP;                               /* set the read state to complete */
               }
               else
               {
                  return MMC_INCOMPLETE_FILE;                                   /* incomplete file */
               }
               
              case MMC_SPRAY_READ_COMP:                                         /* we have completed the read of the file for that chosen record */
              NumOfRecs=0u;                                                     /* set number of records to zero as we found what we want and dont need to do anymore */
              break;
              
              default:                                                          /* bogus state */
              dataSetStep=MMC_SPRAY_SEARCH;                                     /* set back to searching for product id state */
              break;
           }

        }
        break;
        
        case MMC_SPRAY_RCP2:                                                    /* alternate reader for just tank batch make-up data (slower as copies each record ?) */
        RecLen=sizeof(file_data->fileread.spraymix);                            /* read record size including the productId's */
        if (RecLen>=1u)
           NumOfRecs=file_data->fileread.rawdata->size/RecLen;                  /* calculate the number of records in the file */
        while(NumOfRecs>=1u)
        {
          if (RecLen<=file_data->fileread.rawdata->size)                        /* we have at least one record left in the file stream */
          {
             for (i = 0u; i <= RecLen; i++)                                     /* for each byte in the file up to the record length */
             {
                Mmc_Fat_Read(&fCharacter);                                      /* read a byte from the open file stream */
                *file_data->fileread.rawdata->pBuff=fCharacter;                 /* put data into buffer  */
                file_data->fileread.rawdata->pBuff++;                           /* increment file pointer to next byte */
             }
             memcpy((void*) &file_data->fileread.spraymix,(void*) &file_data->fileread.rawdata->pBuff, RecLen);
             file_data->fileread.rawdata->size=file_data->fileread.rawdata->size-RecLen; /* reduce the number of bytes left in the file by the record length */
             if (file_data->fileread.spraymix->prodId==prod_chosen)             /* product id matches */
             {
                NumOfRecs=0u;                                                   /* exit the loop iteration */
             }
             else
             {
                NumOfRecs=--NumOfRecs;
             }
          }
          else
          { /* incomplete file */
            return MMC_INCOMPLETE_FILE;
          }
        }
        break;
        
        case MMC_TANK_RCP_DAT:                                                  /* batch tank only data read request made */
        RecLen=sizeof(file_data->fileread.spraymix);                            /* read record size including the productId's */
        if (RecLen>=1u)                                                         /* we have more than zero bytes to process */
           NumOfRecs=file_data->fileread.rawdata->size/RecLen;                  /* divide the file size by the record length to get the number of records in the file */
        while(NumOfRecs>=1u)                                                    /* for each record in the file */
        {
           Mmc_Fat_Read(&fCharacter);                                           /* read a byte from the open file stream */
           prodIDRead=fCharacter;                                               /* put data into buffer */
           RecLen=sizeof(file_data->fileread.spraymix->recipe);                 /* set the record length to that of a container to hold the settings for a batch makeup */

           if (prodIDRead==prod_chosen)                                         /* product Id is the one we are looking for */
           {
              if (RecLen<=file_data->fileread.rawdata->size)                    /* we have at least one record left in the file stream */
              {
                 for (i = 1u; i <= RecLen; i++)                                 /* for each byte in the file up to the length of this type of record */
                 {
                    Mmc_Fat_Read(&fCharacter);                                  /* get a byte from the open file stream */
                   *file_data->fileread.rawdata->pBuff=fCharacter;              /* put data into buffer */
                    file_data->fileread.rawdata->pBuff++;                       /* increment the pointer to the buffer holding the data to get copied into the structure */
                 }
                 memcpy((void*) &file_data->fileread.spraymix->recipe,(void*) &file_data->fileread.rawdata->pBuff, RecLen);  /* copy the data into the structure for that object */
                 file_data->fileread.rawdata->size=file_data->fileread.rawdata->size-RecLen; /* reduce the number of bytes left in the file by the record length */
                 NumOfRecs=0u;
              }
              else
              {
                 return MMC_INCOMPLETE_FILE;                                    /* incomplete file */
              }
           }
           else
           {
              posFile = Mmc_Fat_Tell();                                         /* get current position of file pointer */
              newPos = Mmc_Fat_Seek(posFile+RecLen+(sizeof(file_data->fileread.spraymix->field)*NO_SPRAY_OF_AREAS)); /* advance pointer to new record */
              NumOfRecs=--NumOfRecs;                                            /* decrement the number of records still not checked */
           }
        }
        break;

        case MMC_FIELD_DATA:                                                    /* field co-ordinates and spray info for field request only */
        RecLen=sizeof(file_data->fileread.spraymix);                            /* read record size including the productId's */
        if (RecLen>=1u)
           NumOfRecs=file_data->fileread.rawdata->size/RecLen;
        while(NumOfRecs>=1u)
        {
           Mmc_Fat_Read(&fCharacter);
           prodIDRead=fCharacter;                                               /* put data into buffer */
           RecLen=sizeof(file_data->fileread.spraymix->field);                  /* set the record length to that of a container to hold the settings for a position and spray quantity in the field */

           if (RecLen<=file_data->fileread.rawdata->size)                       /* we have at least one record left in the file stream */
           {
              if (prodIDRead==prod_chosen)                                      /* product id given matches this one in the mmc file */
              {
                 newPos = Mmc_Fat_Seek(posFile+sizeof(file_data->fileread.spraymix->recipe));  /* advance the file pointer to skip the recipe data */
                 for(fieldSquare=0u;fieldSquare<=NO_SPRAY_OF_AREAS;fieldSquare++)  /* for each field square of data */
                 {
                     if (RecLen<=file_data->fileread.rawdata->size)             /* we have at least one record left in the file stream */
                     {
                        for (i = 1u; i <= RecLen; i++)                          /* read charactures from the file stream up to one field square record length */
                        {
                           Mmc_Fat_Read(&fCharacter);                           /* read char from open file stream */
                           *file_data->fileread.rawdata->pBuff=fCharacter;      /* put data into buffer */
                           file_data->fileread.rawdata->pBuff++;                /* increment the pointer to the buffer holding the data to get copied into the structure */
                        }
                        memcpy((void*) &file_data->fileread.spraymix->field[fieldSquare],(void*) &file_data->fileread.rawdata->pBuff, RecLen); /* copy each square in the field to the field structure */
                        file_data->fileread.rawdata->size=file_data->fileread.rawdata->size-RecLen; /* reduce the number of bytes left in the file by the record length */
                     }
                     else
                     {
                        return MMC_INCOMPLETE_FILE;                             /* incomplete file */
                     }
                 }
                 NumOfRecs=0u;                                                  /* set number of records to zero as we have completed our search */
              }
              else
              {
                 posFile = Mmc_Fat_Tell();                                      /* return current file pointer position in the file input stream */
                 newPos = Mmc_Fat_Seek(posFile+(RecLen*NO_SPRAY_OF_AREAS)+sizeof(file_data->fileread.spraymix->recipe)); /* advance pointer to new record */
                 NumOfRecs=--NumOfRecs;                                         /* decrement the record count as we have just processed another one */
              }
           }
           else
           {
              return MMC_INCOMPLETE_FILE;                                       /* incomplete file */
           }

        }
        break;

        case MMC_REALTIME_DATA:                                                 /* realtime sequence and hmi settings data file (not recipe file) */
        RecLen=sizeof(file_data->realTimeData);                                 /* read record size */
        if (file_data->fileread.rawdata->size==RecLen)                          /* file read contains the same number of bytes of data */
        {
           for (i = 1u; i <= RecLen; i++)                                       /* read charactures from the file stream for one realtime data length */
           {
              Mmc_Fat_Read(&fCharacter);                                        /* read char from open file stream */
              *file_data->fileread.rawdata->pBuff=fCharacter;                   /* put data into buffer */
              file_data->fileread.rawdata->pBuff++;                             /* increment the pointer to the buffer holding the data to get copied into the structure */
           }
           memcpy((void*) &file_data->realTimeData,(void*) &file_data->fileread.rawdata->pBuff, RecLen); /* copy the bytes received from the file into the realtime data structure */
        }
        break;
        
#elif (!defined(GPS_POS_SPRAYER) && defined(BATCH_MIXER))
        case MMC_TANK_RCP_DAT:                                                  /* recipe only */
        RecLen=sizeof(file_data->fileread.spraymix);                            /* set record size to that of product spraymix data */
        if (RecLen>=1u)
           NumOfRecs=file_data->fileread.rawdata->size/RecLen;                  /* calculate the number of records in the file from the total file size and size of record */
        while(NumOfRecs>=1u)
        {
           Mmc_Fat_Read(&fCharacter);                                           /* read byte from file stream */
           prodIDRead=fCharacter;                                               /* put data into buffer */
           RecLen=sizeof(file_data->fileread.spraymix->recipe);                 /* change record length to that of one set of recipe paramters */

           if (prodIDRead==prod_chosen)                                         /* product found in file matches the one specified to the function */
           {
              if (RecLen<=file_data->fileread.rawdata->size)                    /* we have at least one record left in the file stream */
              {
                 for (i = 1u; i <= RecLen; i++)
                 {
                    Mmc_Fat_Read(&fCharacter);                                  /* read a char from the file input stream */
                    *file_data->fileread.rawdata->pBuff=fCharacter;             /* put data into buffer */
                    file_data->fileread.rawdata->pBuff++;                       /* increment the pointer to the buffer holding the data to get copied into the structure */
                 }
                 memcpy((void*) &file_data->fileread.spraymix->recipe,(void*) &file_data->fileread.rawdata->pBuff, RecLen); /* copy tank make-up recipe */
                 file_data->fileread.rawdata->size=file_data->fileread.rawdata->size-RecLen; /* reduce the number of bytes left in the file by the record length */
                 NumOfRecs=0u;                                                  /* set number of records to zero as we have completed our search */
              }
              else
              {
                  return MMC_INCOMPLETE_FILE;                                   /* incomplete file */
              }
           }
           else
           {
              posFile = Mmc_Fat_Tell();                                         /* return current file pointer position in the file input stream */
              newPos = Mmc_Fat_Seek(posFile+RecLen);                            /* advance to next record in file */
              NumOfRecs=--NumOfRecs;                                            /* decrement the record count as we have just processed another one */
           }
        }
        break;
#elif (defined(GPS_POS_SPRAYER) && !defined(BATCH_MIXER))
        case MMC_FIELD_DATA:
        RecLen=sizeof(file_data->fileread.spraymix);                            /* get the spray mixture data length */
        if (RecLen>=1u)
           NumOfRecs=file_data->fileread.rawdata->size/RecLen;                  /* calculate the number of records from the file size and record length */
        while(NumOfRecs>=1u)
        {
           Mmc_Fat_Read(&fCharacter);                                           /* read byte from open file stream */
           prodIDRead=fCharacter;                                               /* put data into buffer */
           RecLen=sizeof(file_data->fileread.spraymix->field);                  /* set record length to that of a field square */

           if (prodIDRead==prod_chosen)                                         /* product id given matches this one in the mmc file */
           {
              for(fieldSquare=0u;fieldSquare<=NO_SPRAY_OF_AREAS;fieldSquare++)  /* for each field square of data */
              {
                 if (RecLen<=file_data->fileread.rawdata->size)                 /* we have at least one record left in the file stream */
                 {
                    for (i = 1u; i <= RecLen; i++)                              /* read charactures from the file stream up to one field square record length */
                    {
                       Mmc_Fat_Read(&fCharacter);                               /* read char from open file stream */
                       *file_data->fileread.rawdata->pBuff=fCharacter;          /* put data into buffer */
                       file_data->fileread.rawdata->pBuff++;                    /* increment the pointer to the buffer holding the data to get copied into the structure */
                    }
                    memcpy((void*) &file_data->fileread.spraymix->field[fieldSquare],(void*) &file_data->fileread.rawdata->pBuff, RecLen); /* copy each square in the field to the field structure */
                    file_data->fileread.rawdata->size=file_data->fileread.rawdata->size-RecLen; /* reduce the number of bytes left in the file by the record length */
                 }
                 else
                 {
                    return MMC_INCOMPLETE_FILE;                                 /* incomplete file */
                 }
              }
              NumOfRecs=0u;                                                     /* set number of records to zero as we have completed our search */
           }
           else
           {
              posFile = Mmc_Fat_Tell();                                         /* return current file pointer position in the file input stream */
              newPos = Mmc_Fat_Seek(posFile+RecLen);                            /* advance to next record in file */
              NumOfRecs=--NumOfRecs;                                            /* decrement the record count as we have just processed another one */
           }
        }
        break;
#endif  /* end of pos sprayer additional information */

        default:                                                                /* not defined in record structure then do nothing */
        return MMC_TYPE_INVALID;
        break;
     }
  }
  else
  {
     return MMC_INVALID_FILE;                                                   /* return invalid file name error */
  }
  return MMC_FILE_OK;
}
/*-----------------------------------------------------------------------------
 *      mmcOpenFileAppRec():  Opens mmc file looks for record and appends a new 
 *                            data record at that position or at the end
 *
 *  Parameters: const char *filename, mmc_file_read_t file_data, mmc_file_writ_t *writ_data,
 *                   uint8_t prod_chosen, mmc_data_typ_e datType
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void mmcOpenFileAppRec( char *filename, mmc_file_read_t *file_data, mmc_file_writ_t *writ_data, uint8_t prod_chosen, mmc_data_typ_e datType )
{
  char fCharacter;
  uint8_t i;
  int8_t NumOfRecs=0;
  uint8_t RecLen=0u;
  uint8_t prodIDRead=0u;
  uint32_t posFile=0u;
  uint32_t newPos=0u;
  uint8_t numField;
 
  if (((filename == NULL) || (file_data == NULL)) || (writ_data == NULL))       /* guard against NULL pointer */
  {
     return;                                                                    /* exit subroutine */
  }
  
  if (Mmc_Fat_Assign(filename, 0u)==1u)                                         /* Find existing file and use it */
  {
     Mmc_Fat_Reset(&file_data->fileread.rawdata->size);                         /* To read file, procedure returns size of file */
     RecLen=sizeof(file_data->fileread.spraymix);                               /* this is the size of a complete spraymix set of data batch plant and position if enabled */
     if (RecLen>=1u)
       NumOfRecs=file_data->fileread.rawdata->size/RecLen;
     while(NumOfRecs>=1u)                                                       /* for each record in the file */
     {
        Mmc_Fat_Read(&fCharacter);
        prodIDRead=fCharacter;                                                  /* put data into buffer */

        if (prodIDRead==prod_chosen)                                            /* product match its already in the file */
        {
           switch(datType)                                                      /* datType describes if we want to write whole record, batch data, or field data */
           {
#if defined(BATCH_MIXER)
                case MMC_TANK_RCP_DAT:                                          /* batch tank only */
                RecLen=sizeof(file_data->fileread.spraymix->recipe);            /* length is batch makeup sequence settings only for that product */
                memcpy((void*) &file_data->fileread.rawdata->pBuff, (void*) &writ_data->filerec.spraymix->recipe, RecLen);  /* copy recipe to buffer */
                posFile = Mmc_Fat_Tell();                                       /* seek current position in the file */
                newPos = Mmc_Fat_Seek(posFile+1u);                              /* new position just after the prodId byte */
                Mmc_Fat_Append();                                               /* Prepare file for append */
                Mmc_Fat_Write(file_data->fileread.rawdata->pBuff, RecLen);      /* write to the mmc file */
                break;
#endif

#if defined(GPS_POS_SPRAYER)
                case MMC_FIELD_DATA:                                            /* field sprayer data only */
                RecLen=sizeof(file_data->fileread.spraymix->field);             /* length is field position and quantity information only for that product */
                for (numField=0u;numField<=NO_SPRAY_OF_AREAS; numField++)       /* for each square of field in the farm */
                {
                   memcpy((void*) &file_data->fileread.rawdata->pBuff, (void*) &writ_data->filerec.spraymix->field[numField], RecLen);  /* copy from recipe to write buffer */
                   posFile = Mmc_Fat_Tell();                                    /* report current position in the file */
#if defined(BATCH_MIXER)
                   newPos = Mmc_Fat_Seek(posFile+(numField!=0u)+((numField!=0u)*sizeof(file_data->fileread.spraymix->recipe))); /* new position just after the prodId add one first time only */
#else
                   newPos = Mmc_Fat_Seek(posFile+(numField!=0u));               /* new position just after the prodId add one first time only */
#endif
                   Mmc_Fat_Append();                                            /* Prepare file for append */
                   Mmc_Fat_Write(file_data->fileread.rawdata->pBuff, RecLen);   /* write to the mmc file */
                }
                break;
#endif

#if (defined(GPS_POS_SPRAYER) || defined(BATCH_MIXER))
                case MMC_SPRAY_RCP:                                             /* batch makeup data and field position sprayer data to be written to the mmc file */
                RecLen=sizeof(file_data->fileread.spraymix);                    /* length is full container of information for that product */
                memcpy((void*) &file_data->fileread.rawdata->pBuff, (void*) &writ_data->filerec.spraymix, RecLen); /* copy complete spray mix data recipe and field to the write buffer */
                posFile = Mmc_Fat_Tell();                                       /* report current position in the file */
                newPos = Mmc_Fat_Seek(posFile);                                 /* new position overwrite including the prodId */
                Mmc_Fat_Append();                                               /* Prepare file for append */
                Mmc_Fat_Write(file_data->fileread.rawdata->pBuff, RecLen);      /* write to the mmc file */
                break;
#endif

                default:                                                        /* non valid state passed then write nothing */
                break;
           }
           NumOfRecs=-127;                                                      /* skip the rest of the file and end */
        }
        else                                                                    /* no match occurred yet */
        {
           posFile = Mmc_Fat_Tell();                                            /* get file position */
           newPos = Mmc_Fat_Seek(posFile+RecLen);                               /* advance by size of one complete record to that of the next productId */
           NumOfRecs=--NumOfRecs;                                               /* decrease the record count as we have processed a record more */
        }
     }
     
     if (NumOfRecs!=-127)                                                       /* no match was found append it at the end of the file as a new record */
     {
        switch(datType)                                                         /* datType describes if we want to write whole record, batch data, or field data */
        {
#if defined(BATCH_MIXER)
           case MMC_TANK_RCP_DAT:                                               /* batch tank only */
           RecLen=sizeof(file_data->fileread.spraymix->recipe);                 /* length is batch makeup sequence settings only for that product */
           memcpy((void*) &file_data->fileread.rawdata->pBuff, (void*) &writ_data->filerec.spraymix->recipe, RecLen);
           Mmc_Fat_Append();                                                    /* Prepare file for append */
           Mmc_Fat_Write(file_data->fileread.rawdata->pBuff, RecLen);           /* write to the mmc file */
           break;
#endif

#if defined(GPS_POS_SPRAYER)
           case MMC_FIELD_DATA:                                                 /* field sprayer data only */
           RecLen=sizeof(file_data->fileread.spraymix->field);                  /* length is field position and quantity information only for that product */
           for (numField=0u;numField<=NO_SPRAY_OF_AREAS; numField++)
           {
               memcpy((void*) &file_data->fileread.rawdata->pBuff, (void*) &writ_data->filerec.spraymix->field[numField], RecLen);  /* copy the field square to the write buffer */
               posFile = Mmc_Fat_Tell();                                        /* report current location in the file where this product is found already */
#if defined(BATCH_MIXER)
               newPos = Mmc_Fat_Seek(posFile+(numField!=0u)+((numField!=0u)*sizeof(file_data->fileread.spraymix->recipe))); /* new position just after the prodId add one first time only */
#else
               newPos = Mmc_Fat_Seek(posFile+(numField!=0u));                   /* new position just after the prodId add one first time only */
#endif
               Mmc_Fat_Append();                                                /* Prepare file for append */
               Mmc_Fat_Write(file_data->fileread.rawdata->pBuff, RecLen);       /* write to the mmc file */
            }
            break;
#endif
           
#if (defined(GPS_POS_SPRAYER) || defined(BATCH_MIXER))
            case MMC_SPRAY_RCP:                                                 /* batch makeup data and field position sprayer data to be written to the mmc file */
            RecLen=sizeof(file_data->fileread.spraymix);                        /* length is full container of information for that product */
            memcpy((void*) &file_data->fileread.rawdata->pBuff, (void*) &writ_data->filerec.spraymix, RecLen); /* copy spray mixture recipe and field to write buffer */
            posFile = Mmc_Fat_Tell();                                           /* report current location in the file where this product is found already */
            newPos = Mmc_Fat_Seek(posFile);                                     /* new position overwrite including the prodId */
            Mmc_Fat_Append();                                                   /* Prepare file for append */
            Mmc_Fat_Write(file_data->fileread.rawdata->pBuff, RecLen);          /* write to the mmc file */
            break;
#endif
        }
     }
  }
}
/*-----------------------------------------------------------------------------
 *      mmcManageRecords():  Manages mmc file records relating to the spray
 *                            system
 *
 *  Parameters: char *filename, mmc_file_read_t *file_data, mmc_file_writ_t file_write,
 *              mmc_data_typ_e recTyp, gui_spray_data_t *guiReq, const rionUTCDateTimeField_t *tim
 *
 *  Return: 0=nothing, else saving, deleting, saved, deleted
 *----------------------------------------------------------------------------*/
uint8_t mmcManageRecords( char *filename, mmc_file_read_t *file_data,  mmc_data_typ_e recTyp, mmc_file_writ_t file_write, gui_spray_data_t *guiReq, const rionUTCDateTimeField_t *tim )
{

   char status;                                                                 /* unsigned byte to hold status of file exists query */
   uint16_t retCode=0u;
   
   if (guiReq->formatDsk == TRUE)                                               /* format SD Card request from gui */
   {
       g_mmcStep= MMC_FORMAT_SD;                                                /* set the filesystem state to format SD card */
       guiReq->formatDsk = FALSE;                                               /* unlatch the HMI trigger */
       return (1u);
   }
   else if (guiReq->saveProduct == TRUE)                                        /* save product request from gui */
   {
      status = Mmc_Fat_Exists(filename);
      if (1 == status)                                                          /* file exists */
      {
         mmcOpenFileAppRec( filename, file_data, &file_write, file_write.filerec.spraymix.prodId, recTyp );   /* open and either replace or append this product record */
      }
      else if (0 == status)                                                     /* file doesnt exist create a new one */
      {
         mmcCreateNewFile( filename, file_write, recTyp, tim );                 /* create a new file with this record */
      }
      else {}
      guiReq->saveProduct= FALSE;                                               /* complete the request */
      return (1u);
   }
   else if (guiReq->delProduct == TRUE)                                         /* delete product request from gui */
   {
      retCode=Mmc_Fat_Assign(filename, 0u);
      if (MMC_FILE_EXIST==retCode)                                              /* file exists and has been assigned */
      {
         Mmc_Fat_Delete();                                                      /* delete assigned file */
         guiReq->delProduct= FALSE;                                             /* clear and therefore complete the request */
      }
      else if (MMC_NO_HANDLER==retCode)                                         /* no command free handlers keep repeating until done */
      {
      }
      else                                                                      /* return 0 was already not there just complete the request */
      {
         guiReq->delProduct= FALSE;
      }
      return retCode;
   }
   else
   {
      return 0u;
   }

}
/*-----------------------------------------------------------------------------
 *      initMMCFileSys()():  Initialises the MMC file system
 *
 *  Parameters: none
 *
 *  Return: nothing
 *----------------------------------------------------------------------------*/
void initMMCFileSys( )
{
  switch(g_mmcStep)                                                             /* global mmc file system state */
  {
    case MMC_SPI_INIT:                                                          /* initialise the SPI connection to the mmc on SPI port 2 */
    SPI2_Init_Advanced(_SPI_MASTER, _SPI_8_BIT, 64u, _SPI_SS_DISABLE, _SPI_DATA_SAMPLE_MIDDLE, _SPI_CLK_IDLE_HIGH, _SPI_ACTIVE_2_IDLE);
    mmcTimeDuration = -1;
    calculateTick2Now( &mmcTimeDuration, &mmcTimeDurationLast );                /* initialise timer */
    g_mmcStep= MMC_DELAY;                                                       /* go to wait step */
    break;

    case MMC_DELAY:                                                             /* wait for initialsation delay */
    if (mmcTimeDuration >= MMC_INIT_DELAY)                                      /* timer delay has expired */
    {
       g_mmcStep=MMC_FILE_INIT;                                                 /* goto file init */
    }
    else
    {
       calculateTick2Now( &mmcTimeDuration, &mmcTimeDurationLast );               /* recalculate time from start to now */
    }
    break;

    case MMC_FILE_INIT:                                                         /* file initialisation */
    if (Mmc_Fat_Init()==0u)                                                     /* initialise the file allocation table if success re-init SPI port */
    {
       SPI2_Init_Advanced(_SPI_MASTER, _SPI_8_BIT, 8u, _SPI_SS_DISABLE, _SPI_DATA_SAMPLE_MIDDLE, _SPI_CLK_IDLE_HIGH, _SPI_ACTIVE_2_IDLE);
       g_mmcStep= MMC_CRE_SWAP;                                                 /* create swap file */
    }
    else
    {
       if (g_FATprevState==MMC_FAT_FAIL)                                        /* mmc fat failure has occurred */
       {
         g_mmcStep= MMC_SLOW_POLL;                                              /* failure then wait for long delay and re-try */
       }
       else
       {
         g_mmcStep= MMC_FAT_FAIL;                                               /* failure then wait for short delay and re-try */
       }
    }
    break;

    case MMC_CRE_SWAP:                                                          /* create swap file */
    mmcCreateSwapFile();
    g_mmcStep= MMC_GO_READY;                                                    /* file system is ready */
    break;

    case MMC_GO_READY:                                                          /* state just to wait a couple of ticks before using file system */
    mmcFileSysState= MMC_FAT16_READY;                                           /* set the state to ready */
    g_FATprevState= MMC_FAT16_READY;                                            /* set previous state to ready to clear global FAT failure timer*/
    g_mmcStep= MMC_FAT16_READY;                                                 /* set the step to ready it is only changed globally if the writes to the FAT continously fail */
    break;
    
    case MMC_FAT16_READY:                                                       /* wait here once ready */
    break;

    case MMC_FAT_FAIL:                                                          /* file allocation fail */
    mmcTimeDuration = -1;
    calculateTick2Now( &mmcTimeDuration, &mmcTimeDurationLast );
    if (g_FATprevState!=MMC_FAT_FAIL)                                           /* start a global timer measuring the time since first FAT failure */
    {
      mmcTimeDuration = -1;
      calculateTick2Now( &g_mmcTimeDuration, &g_mmcTimeDurationLast );            /* initialise global timer for timeout at start-up */
      g_FATprevState= MMC_FAT_FAIL;
    }
    g_mmcStep=MMC_FAT_WAIT;
    break;

    case MMC_FAT_WAIT:                                                          /* wait for delay before re-trying to allocate file table */
    if (mmcTimeDuration >= MMC_FAT_DELAY)
    {
       g_mmcStep=MMC_FILE_INIT;                                                 /* try to re-init the FAT */
    }
    else
    {
       calculateTick2Now( &mmcTimeDuration, &mmcTimeDurationLast );               /* re-calcuate time to now */
    }
    break;

    case MMC_SLOW_POLL:                                                         /* set to slow poll mode as we timed out in main program reading the FAT at boot-up */
    mmcTimeDuration = -1;
    calculateTick2Now( &mmcTimeDuration, &mmcTimeDurationLast );
    g_mmcStep=MMC_SLOW_WAIT;
    break;

    case MMC_SLOW_WAIT:                                                         /* wait for long delay before re-trying to allocate file table as we didnt get it at boot-up */
    if (mmcTimeDuration >= MMC_SLOW_DELAY)
    {
       g_mmcStep= MMC_FILE_INIT;                                                /* try to re-init the FAT */
    }
    else
    {
       calculateTick2Now( &mmcTimeDuration, &mmcTimeDurationLast );               /* re-calcuate time to now */
    }
    break;

    case MMC_FORMAT_SD:                                                         /* SD card format request */
    SPI2_Init_Advanced(_SPI_MASTER, _SPI_8_BIT, 64u, _SPI_SS_DISABLE, _SPI_DATA_SAMPLE_MIDDLE, _SPI_CLK_IDLE_HIGH, _SPI_ACTIVE_2_IDLE);  /* slow speed */
    formatRet=Mmc_Fat_QuickFormat("ACPData");                                   /* attempt to format the SD card */
    switch(formatRet)                                                           /* look at the result of the format */
    {
       case MMC_FORMAT_OK:
       g_mmcStep= MMC_FILE_INIT;                                                /* try to re-init the FAT */
       break;
       
       case MMC_FORMAT_FL:
       g_mmcStep= MMC_FORMAT_FAIL;                                              /* goto format failure */
       break;
       
       case MMC_SD_ERROR:
       g_mmcStep= MMC_NO_SD_CARD;                                               /* goto no SD card error */
       break;
    }
    break;

    case MMC_NO_SD_CARD:                                                        /* no SD card error after attempting to format */
    break;                                                                      /* wait until action made */
    
    case MMC_FORMAT_FAIL:                                                       /* format failure */
    break;
    
    default:
    if (mmcFileSysState!=MMC_GO_READY)                                          /* state is not ready */
    {
       g_mmcStep=MMC_SPI_INIT;                                                  /* re-initialise spi port */
    }
    break;
  }
}