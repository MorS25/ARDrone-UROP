#include "video_processing.h"

void process(CvMat* in, CvMat *out)
{
  cvCanny(in, out, 0.5, 0.6, 3);
}
