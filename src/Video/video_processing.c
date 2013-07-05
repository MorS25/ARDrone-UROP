void process(cvMat* in, cvMat *out)
{
  cvCanny(in, out, 0.5, 0.6);
}
