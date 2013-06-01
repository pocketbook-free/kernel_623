#ifndef __CRC32_H__
#define __CRC32_H__

#ifdef __cplusplus
extern "C" {
#endif

#define ex_bl(var,type)  do\
{\
        int i,j;\
        char tmp, *s = (char *)(&var);\
        for(i=0,j=sizeof(type)-1; i<j; i++,j--) {\
                tmp=s[i];\
                s[i]=s[j];\
                s[j]=tmp;\
        }\
} while(0)


extern unsigned long crc32(unsigned long crc, unsigned char *buf, unsigned int len);

#ifdef __cplusplus
}
#endif

#endif

