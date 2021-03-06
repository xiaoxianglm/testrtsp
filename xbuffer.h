#ifndef XBUFFER_H
#define XBUFFER_H

void *xbuffer_init(int chunk_size);
void *xbuffer_free(void *buf);
void *xbuffer_copyin(void *buf, int index, const void *data, int len);
void *xbuffer_ensure_size(void *buf, int size);
void *xbuffer_strcat(void *buf, char *data);

#endif /* XBUFFER_H */
