#ifndef __OGG_GPADEC_API_H__
#define __OGG_GPADEC_API_H__

#include "gp_acodec.h"
#ifdef __cplusplus
extern "C" {
#endif

unsigned int acodec_oggdec_instance_size(void);
int acodec_oggdec_init(audec_param_t *adp);
int acodec_oggdec_dec(audec_param_t *adp, unsigned char *buf, unsigned int *out_size);
int acodec_oggdec_uninit(audec_param_t *adp);

#ifdef __cplusplus
}
#endif

#endif //__OGG_GPADEC_API_H__
