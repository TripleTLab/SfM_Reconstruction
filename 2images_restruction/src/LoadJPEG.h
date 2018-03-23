


#ifndef __load_jpeg_h__
#define __load_jpeg_h__

img_t *LoadJPEG(const char *filename);
void GetJPEGDimensions(const char *filename, int &w, int &h);
void WriteJPEG(const img_t *img, const char *filename);

#endif 
