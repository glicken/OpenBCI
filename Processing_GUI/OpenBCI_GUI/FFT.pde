
class FFT {
  final static int HAMMING = 1;
  int Nfft;
  float fs_Hz;
  float[] data;
  private int window_ind;
 
  FFT(int N,float fs) {
    Nfft = N;
    fs_Hz = fs;
    data = new float[N];
  }
  
  public void window(int win) {
    window_ind = win;
  }
  
  public void forward(float[] foo) {
  }
  
  public int specSize() {
    return Nfft;
  }
  
  public float getBand(int I) {
    return data[I];
    //
  }
  public void setBand(int I, float val) {
    data[I] = val;
  }
  
  public float indexToFreq(int I) {
    return fs_Hz*(float(I)/float(Nfft));
  }
}
