#ifndef __yandex_disk_file_
#define __yandex_disk_file_
/* 
   Communicate with cloud storage on yandex disk
*/
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define HTTP_URL_GET_YANDEX_FILE(user,pass,dir,fileOnYan,filetoCall,buf)        do { sprintf(buf,"https:\/\/webdav.yandex.ru\/%s\/%s -T %s --user \"%s:%s\"",dir,fileOnYan,filetoCall,user,pass); } while(0);
#define HTTP_URL_PUT_YANDEX_FILE(user,pass,dir,filetoCall,buf)                  do { sprintf(buf,"https:\/\/webdav.yandex.ru\/%s -T %s --user \"%s:%s\"",dir,filetoCall,user,pass); } while(0);
#define HTTP_URL_MKDIR_YANDEX_FILE(user,pass,dir,buf)                           do { sprintf(buf,"https:\/\/webdav.yandex.ru\/%s -X MKCOL --user \"%s:%s\" ",dir,user,pass); } while(0);
#define HTTP_URL_DELETE_YANDEX_FILE(user,pass,dir,delFile,buf)                  do { sprintf(buf,"https:\/\/webdav.yandex.ru\/%s\/%s -X DELETE --user \"%s:%s\" ",dir,delFile,user,pass); } while(0);
#define HTTP_URL_MOVE_YANDEX_FILE(user,pass,dir,movFile,buf)                    do { sprintf(buf,"https:\/\/webdav.yandex.ru\/%s\/%s -X MOVE --user \"%s:%s\" ",dir,movFile,user,pass); } while(0);
#define HTTP_HEADER_MOVE_YANDEX_FILE(url,file,X)                                do { sprintf(X,"Destination:http:\/\/%s\/%s",url,file); } while(0);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif