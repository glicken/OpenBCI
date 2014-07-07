///////////////////////////////////////////////
//
// GUI for controlling the ADS1299-based OpenBCI Shield
//
// Created: Chip Audette, July 2014
//
// No warranty.  Use at your own risk.  Use for whatever you'd like.
// 
///////////////////////////////////////////////


import processing.serial.*;  //for serial communication to Arduino/OpenBCI
import ddf.minim.analysis.*; //for FFT
import java.util.*; //for Array.copyOfRange()
import java.lang.Math; //for exp, log, sqrt...they seem better than Processing's built-in
import processing.core.PApplet;


//choose where to get the EEG data
final int DATASOURCE_NORMAL =  0;        //Receive LIVE data from OpenBCI
final int DATASOURCE_PLAYBACKFILE = 3; //Playback previously recorded data...see "playbackData_fname" down below
final int eegDataSource = DATASOURCE_PLAYBACKFILE;

//Serial communications constants
OpenBCI_ADS1299 openBCI = new OpenBCI_ADS1299(); //dummy creation to get access to constants, create real one later
String openBCI_portName = "COM14";   /************** CHANGE THIS TO MATCH THE COM PORT REPORTED ON *YOUR* COMPUTER *****************/


//these settings are for a single OpenBCI board
int openBCI_baud = 115200; //baud rate from the rArduino
final int OpenBCI_Nchannels = 8; //normal OpenBCI has 8 channels

//here are variables that are used if loading input data from a CSV text file...double slash ("\\") is necessary to make a single slash
String playbackData_fname = "C:\\Users\\disco\\Documents\\GitHub\\EEGHacker\\Data\\2014-05-31 RobotControl\\SavedData\\openBCI_raw_2014-05-31_20-57-51_Robot05.txt";
Table_CSV playbackData_table;
int currentTableRowIndex=0;
int nextPlayback_millis = -100; //any negative number

//other data fields
float dataBuffX[];
float dataBuffY_uV[][]; //2D array to handle multiple data channels, each row is a new channel so that dataBuffY[3][] is channel 4
float dataBuffY_filtY_uV[][];
int nchan = OpenBCI_Nchannels; //normally, nchan = OpenBCI_Nchannels.  Choose a smaller number to show fewer on the GUI
int nchan_active_at_startup = 2;  //how many channels to be LIVE at startup
int prev_time_millis = 0;
final int nPointsPerUpdate = 50; //update screen after this many data points.  
float yLittleBuff[] = new float[nPointsPerUpdate];
DataStatus is_railed[];
final int threshold_railed = int(pow(2,23)-1000);  //fully railed should be +/- 2^23, so set this threshold close to that value
final int threshold_railed_warn = int(pow(2,23)*0.75); //set a somewhat smaller value as the warning threshold
float yLittleBuff_uV[][] = new float[nchan][nPointsPerUpdate]; //small buffer used to send data to the filters

//create objects that'll do the EEG signal processing
EEG_Processing eegProcessing;
EEG_Processing_User eegProcessing_user;
HexBug hexBug;

////fft constants
//int Nfft = 512; //set resolution of the FFT.  Use N=256 for normal, N=512 for MU waves
////float fft_smooth_fac = 0.75f; //use value between [0 and 1].  Bigger is more smoothing.  Use 0.9 for MU waves, 0.75 for Alpha, 0.0 for no smoothing
//FFT fftBuff[] = new FFT[nchan];   //from the minim library
//float[] smoothFac = new float[]{0.75, 0.9, 0.95, 0.98, 0.0, 0.5};
//final int N_SMOOTHEFAC = 6;
//int smoothFac_ind = 1;  //which index to start on

//plotting constants
//Gui_Manager gui;
//float default_vertScale_uV = 200.0f;
float displayTime_sec = 5f;
float dataBuff_len_sec = displayTime_sec+3f; //needs to be wider than actual display so that filter startup is hidden

//program constants
boolean isRunning=false;
boolean redrawScreenNow = true;
int openBCI_byteCount = 0;
int inByte = -1;    // Incoming serial data

//file writing variables
OutputFile_rawtxt fileoutput;
String output_fname;

//openBCI data packet
final int nDataBackBuff = 3*(int)openBCI.fs_Hz;
DataPacket_ADS1299 dataPacketBuff[] = new DataPacket_ADS1299[nDataBackBuff]; //allocate the array, but doesn't call constructor.  Still need to call the constructor!
int curDataPacketInd = -1;
int lastReadDataPacketInd = -1;




/////////////////////////////////////////////////////////////////////// functions

void appendAndShift(float[] data, float[] newData) {
  int nshift = newData.length;
  int end = data.length-nshift;
  for (int i=0; i < end; i++) {
    data[i]=data[i+nshift];  //shift data points down by 1
  }
  for (int i=0; i<nshift;i++) {
    data[end+i] = newData[i];  //append new data
  }
}

void prepareData(float[] dataBuffX, float[][] dataBuffY_uV, float fs_Hz) {
  //initialize the x and y data
  int xoffset = dataBuffX.length - 1;
  for (int i=0; i < dataBuffX.length; i++) {
    dataBuffX[i] = ((float)(i-xoffset)) / fs_Hz; //x data goes from minus time up to zero
    for (int Ichan = 0; Ichan < nchan; Ichan++) { 
      dataBuffY_uV[Ichan][i] = 0f;  //make the y data all zeros
    }
  }
}

//void initializeFFTObjects(FFT[] fftBuff, float[][] dataBuffY_uV, int N, float fs_Hz) {
//
//  float[] fooData;
//  for (int Ichan=0; Ichan < nchan; Ichan++) {
//    //make the FFT objects...Following "SoundSpectrum" example that came with the Minim library
//    //fftBuff[Ichan] = new FFT(Nfft, fs_Hz);  //I can't have this here...it must be in setup
//    fftBuff[Ichan].window(FFT.HAMMING);
//
//    //do the FFT on the initial data
//    fooData = dataBuffY_uV[Ichan];
//    fooData = Arrays.copyOfRange(fooData, fooData.length-Nfft, fooData.length); 
//    fftBuff[Ichan].forward(fooData); //compute FFT on this channel of data
//  }
//}

//set window size
int win_x = 1200;  //window width
int win_y = 768; //window height
void setup() {

  //open window
  size(win_x, win_y, P2D);
  println("Starting setup...");
  
  //prepare data variables
  dataBuffX = new float[(int)(dataBuff_len_sec * openBCI.fs_Hz)];
  dataBuffY_uV = new float[nchan][dataBuffX.length];
  dataBuffY_filtY_uV = new float[nchan][dataBuffX.length];
  //data_std_uV = new float[nchan];
  //data_elec_imp_ohm = new float[nchan];
  //is_railed = new DataStatus[nchan];
  //for (int i=0; i<nchan;i++) is_railed[i] = new DataStatus(threshold_railed,threshold_railed_warn);
  for (int i=0; i<nDataBackBuff;i++) { 
    //dataPacketBuff[i] = new DataPacket_ADS1299(nchan+n_aux_ifEnabled);
    dataPacketBuff[i] = new DataPacket_ADS1299(OpenBCI_Nchannels);
  }

  //for data processing
  eegProcessing = new EEG_Processing(nchan,openBCI.fs_Hz);

  //initialize the data
  prepareData(dataBuffX, dataBuffY_uV, openBCI.fs_Hz);

  //initialize the FFT objects
//  for (int Ichan=0; Ichan < nchan; Ichan++) { 
//    fftBuff[Ichan] = new FFT(Nfft, openBCI.fs_Hz);
//  };  //make the FFT objects
//  initializeFFTObjects(fftBuff, dataBuffY_uV, Nfft, openBCI.fs_Hz);

  //initilize the GUI
//  String filterDescription = eegProcessing.getFilterDescription();
//  EEG_Processing_User foo = new EEG_Processing_User();
//  gui = new Gui_Manager(this, win_x, win_y, nchan,displayTime_sec,default_vertScale_uV,filterDescription, smoothFac[smoothFac_ind],foo);
  
  //associate the data to the GUI traces
//  gui.initDataTraces(dataBuffX, dataBuffY_filtY_uV, fftBuff, eegProcessing.data_std_uV, is_railed,eegProcessing.polarity);

  //limit how much data is plotted...hopefully to speed things up a little
//  gui.setDoNotPlotOutsideXlim(true);
//  gui.setDecimateFactor(2);


  //prepare the source of the input data
  switch (eegDataSource) {
    case DATASOURCE_NORMAL:
      //list all the serial ports available...useful for debugging
      println(Serial.list());
      //openBCI_portName = Serial.list()[0];
      
      // Open the serial port to the Arduino that has the OpenBCI
      println("OpenBCI_GUI: Opening Serial " + openBCI_portName);
      int nDataValuesPerPacket = OpenBCI_Nchannels;
      openBCI = new OpenBCI_ADS1299(this, openBCI_portName, openBCI_baud, nDataValuesPerPacket); //this also starts the data transfer after XX seconds
      break;
    case DATASOURCE_PLAYBACKFILE:
      //open and load the data file
      println("OpenBCI_GUI: loading playback data from " + playbackData_fname);
      try {
        playbackData_table = new Table_CSV(playbackData_fname);
      } catch (Exception e) {
        println("setup: could not open file for playback: " + playbackData_fname);
        println("   : quitting...");
        exit();
      }
      println("OpenBCI_GUI: loading complete.  " + playbackData_table.getRowCount() + " rows of data, which is " + round(float(playbackData_table.getRowCount())/openBCI.fs_Hz) + " seconds of EEG data");
      
      //removing first column of data from data file...the first column is a time index and not eeg data
      playbackData_table.removeColumn(0);
      break;
    default: 
  }

  //initialize the on/off state of the different channels...only if the user has specified fewer channels
  //than is on the OpenBCI board
  for (int Ichan=nchan_active_at_startup; Ichan<OpenBCI_Nchannels;Ichan++) deactivateChannel(Ichan);  //deactivate unused channels
  
  // initialize the user processing stuff as well as the HexBug
  //hexBug = new HexBug(openBCI.serial_openBCI);
  //eegProcessing_user = new EEG_Processing_User(nchan,openBCI.fs_Hz,hexBug);
  
  //final config
  //setBiasState(openBCI.isBiasAuto);

  //start
  startRunning();

  println("setup: Setup complete...");
}

int pointCounter = 0;
//boolean newData = true;
int prevBytes = 0; 
int prevMillis=millis();
int byteRate_perSec = 0;
int drawLoop_counter = 0;
void draw() {
  drawLoop_counter++;
  if (isRunning) {
    //get the data, if it is available
    pointCounter = getDataIfAvailable(pointCounter);
    
    //has enough data arrived to process it and update the GUI?
    if (pointCounter >= nPointsPerUpdate) {
      pointCounter = 0;  //reset for next time
      
      //process the data
      processNewData();

      //tell the GUI that it has received new data via dumping new data into arrays that the GUI has pointers to
      //gui.update(eegProcessing.data_std_uV,data_elec_imp_ohm);
        
      redrawScreenNow=true;
    }
  }
    
  int drawLoopCounter_thresh = 100;
  if ((redrawScreenNow) || (drawLoop_counter >= drawLoopCounter_thresh)) {
    //if (drawLoop_counter >= drawLoopCounter_thresh) println("OpenBCI_GUI: redrawing based on loop counter...");
    drawLoop_counter=0; //reset for next time
    redrawScreenNow = false;  //reset for next time
    
    //update the title of the figure;
    switch (eegDataSource) {
      case DATASOURCE_NORMAL:
        frame.setTitle(int(frameRate) + " fps, Byte Count = " + openBCI_byteCount + ", bit rate = " + byteRate_perSec*8 + " bps" + ", " + int(float(fileoutput.getRowsWritten())/openBCI.fs_Hz) + " secs Saved, Writing to " + output_fname);
        break;
      case DATASOURCE_PLAYBACKFILE:
        frame.setTitle(int(frameRate) + " fps, Playing " + int(float(currentTableRowIndex)/openBCI.fs_Hz) + " of " + int(float(playbackData_table.getRowCount())/openBCI.fs_Hz) + " secs, Reading from: " + playbackData_fname);
        break;
    } 
    
    //redraw the screen...not every time, get paced by when data is being plotted    
    background(0);  //clear the screen
    //gui.draw(); //draw the GUI
  }
}

int getDataIfAvailable(int pointCounter) {
  
  if (eegDataSource == DATASOURCE_NORMAL) {
    //get data from serial port as it streams in

      //first, get the new data (if any is available)
      openBCI.updateState(); //this is trying to listen to the openBCI hardware.  New data is put into dataPacketBuff and increments curDataPacketInd.
      
      //next, gather any new data into the "little buffer"
      while ( (curDataPacketInd != lastReadDataPacketInd) && (pointCounter < nPointsPerUpdate)) {
        lastReadDataPacketInd = (lastReadDataPacketInd+1) % dataPacketBuff.length;  //increment to read the next packet
        for (int Ichan=0; Ichan < nchan; Ichan++) {   //loop over each cahnnel
          //scale the data into engineering units ("microvolts") and save to the "little buffer"
          yLittleBuff_uV[Ichan][pointCounter] = dataPacketBuff[lastReadDataPacketInd].values[Ichan] * openBCI.scale_fac_uVolts_per_count;
        } 
        pointCounter++; //increment counter for "little buffer"
      }
  } else {
    // load data to simulate real time
        
    //has enough time passed?
    int current_millis = millis();
    if (current_millis >= nextPlayback_millis) {
      //prepare for next time
      int increment_millis = int(round(float(nPointsPerUpdate)*1000.f/openBCI.fs_Hz));
      if (nextPlayback_millis < 0) nextPlayback_millis = current_millis;
      nextPlayback_millis += increment_millis;

      // generate or read the data
      lastReadDataPacketInd = 0;
      for (int i = 0; i < nPointsPerUpdate; i++) {
        dataPacketBuff[lastReadDataPacketInd].sampleIndex++;
        currentTableRowIndex=getPlaybackDataFromTable(playbackData_table,currentTableRowIndex,openBCI.scale_fac_uVolts_per_count, dataPacketBuff[lastReadDataPacketInd]);
        
        //gather the data into the "little buffer"
        for (int Ichan=0; Ichan < nchan; Ichan++) {
          //scale the data into engineering units..."microvolts"
          yLittleBuff_uV[Ichan][pointCounter] = dataPacketBuff[lastReadDataPacketInd].values[Ichan]* openBCI.scale_fac_uVolts_per_count;
        }
        pointCounter++;
      } //close the loop over data points
    } // close "has enough time passed"
  } 
  return pointCounter;
}

void processNewData() {

  byteRate_perSec = (int)(1000.f * ((float)(openBCI_byteCount - prevBytes)) / ((float)(millis() - prevMillis)));
  prevBytes = openBCI_byteCount; 
  prevMillis=millis();
  float foo_val;
  //float prevFFTdata[] = new float[fftBuff[0].specSize()];
  double foo;

  //update the data buffers
  for (int Ichan=0;Ichan < nchan; Ichan++) {
    //append the new data to the larger data buffer...because we want the plotting routines
    //to show more than just the most recent chunk of data.  This will be our "raw" data.
    appendAndShift(dataBuffY_uV[Ichan], yLittleBuff_uV[Ichan]);
    
    //make a copy of the data that we'll apply processing to.  This will be what is displayed on the full montage
    dataBuffY_filtY_uV[Ichan] = dataBuffY_uV[Ichan].clone();
  }

  
//  //update the FFT (frequency spectrum)
//  for (int Ichan=0;Ichan < nchan; Ichan++) {  
//
//    //copy the previous FFT data...enables us to apply some smoothing to the FFT data
//    for (int I=0; I < fftBuff[Ichan].specSize(); I++) prevFFTdata[I] = fftBuff[Ichan].getBand(I); //copy the old spectrum values
//    
//    //prepare the data for the new FFT
//    float[] fooData_raw = dataBuffY_uV[Ichan];  //use the raw data for the FFT
//    fooData_raw = Arrays.copyOfRange(fooData_raw, fooData_raw.length-Nfft, fooData_raw.length);   //trim to grab just the most recent block of data
//    float meanData = mean(fooData_raw);  //compute the mean
//    for (int I=0; I < fooData_raw.length; I++) fooData_raw[I] -= meanData; //remove the mean (for a better looking FFT
//    
//    //compute the FFT
//    fftBuff[Ichan].forward(fooData_raw); //compute FFT on this channel of data
//    
//    //average the FFT with previous FFT data so that it makes it smoother in time
//    double min_val = 0.01d;
//    for (int I=0; I < fftBuff[Ichan].specSize(); I++) {   //loop over each fft bin
//      if (prevFFTdata[I] < min_val) prevFFTdata[I] = (float)min_val; //make sure we're not too small for the log calls
//      foo = fftBuff[Ichan].getBand(I); if (foo < min_val) foo = min_val; //make sure this value isn't too small
//      
//       if (true) {
//        //smooth in dB power space
//        foo =   (1.0d-smoothFac[smoothFac_ind]) * java.lang.Math.log(java.lang.Math.pow(foo,2));
//        foo += smoothFac[smoothFac_ind] * java.lang.Math.log(java.lang.Math.pow((double)prevFFTdata[I],2)); 
//        foo = java.lang.Math.sqrt(java.lang.Math.exp(foo)); //average in dB space
//      } else { 
//        //smooth (average) in linear power space
//        foo =   (1.0d-smoothFac[smoothFac_ind]) * java.lang.Math.pow(foo,2);
//        foo+= smoothFac[smoothFac_ind] * java.lang.Math.pow((double)prevFFTdata[I],2); 
//        // take sqrt to be back into uV_rtHz
//        foo = java.lang.Math.sqrt(foo);
//      }
//      fftBuff[Ichan].setBand(I,(float)foo); //put the smoothed data back into the fftBuff data holder for use by everyone else
//    } //end loop over FFT bins
//  } //end the loop over channels.
  
//  //apply additional processing for the time-domain montage plot (ie, filtering)
//  eegProcessing.process(yLittleBuff_uV,dataBuffY_uV,dataBuffY_filtY_uV,fftBuff);
  
//  //apply user processing
//  eegProcessing_user.process(yLittleBuff_uV,dataBuffY_uV,dataBuffY_filtY_uV,fftBuff);
  
//  //look to see if the latest data is railed so that we can notify the user on the GUI
//  for (int Ichan=0;Ichan < nchan; Ichan++) is_railed[Ichan].update(dataPacketBuff[lastReadDataPacketInd].values[Ichan]);

//  //compute the electrode impedance. Do it in a very simple way [rms to amplitude, then uVolt to Volt, then Volt/Amp to Ohm]
//  for (int Ichan=0;Ichan < nchan; Ichan++) data_elec_imp_ohm[Ichan] = (sqrt(2.0)*eegProcessing.data_std_uV[Ichan]*1.0e-6) / openBCI.leadOffDrive_amps;     
      
  //add your own processing steps here!


}


//here is the routine that listens to the serial port.
//if any data is waiting, get it, parse it, and stuff it into our vector of 
//pre-allocated dataPacketBuff
void serialEvent(Serial port) {
  //check to see which serial port it is
  if (port == openBCI.serial_openBCI) {
    boolean echoBytes = !openBCI.isStateNormal(); 
    openBCI.read(echoBytes);
    openBCI_byteCount++;
    if (openBCI.isNewDataPacketAvailable) {
      //copy packet into buffer of data packets
      curDataPacketInd = (curDataPacketInd+1) % dataPacketBuff.length; //this is also used to let the rest of the code that it may be time to do something
      openBCI.copyDataPacketTo(dataPacketBuff[curDataPacketInd]);  //resets isNewDataPacketAvailable to false
      
      //write this chunk of data to file
      fileoutput.writeRawData_dataPacket(dataPacketBuff[curDataPacketInd],openBCI.scale_fac_uVolts_per_count,nchan);
    }
  } 
  else {
    inByte = port.read();
  }
}

//interpret a keypress...the key pressed comes in as "key"
void keyPressed() {
  //note that the Processing variable "key" is the keypress as an ASCII character
  //note that the Processing variable "keyCode" is the keypress as a JAVA keycode.  This differs from ASCII  
  //println("OpenBCI_GUI: keyPressed: key = " + key + ", int(key) = " + int(key) + ", keyCode = " + keyCode);
  
  if ((int(key) >=32) && (int(key) <= 126)) {  //32 through 126 represent all the usual printable ASCII characters
    parseKey(key);
  } else {
    parseKeycode(keyCode);
  }
}
void parseKey(char val) {
  int Ichan; boolean activate; int code_P_N_Both;
  
  //assumes that val is a usual printable ASCII character (ASCII 32 through 126)
  switch (val) {
    case 's':
      stopButtonWasPressed();
      break;
    case 'm':
     String picfname = "OpenBCI-" + getDateString() + ".jpg";
     println("OpenBCI_GUI: 'm' was pressed...taking screenshot:" + picfname);
     saveFrame(picfname);    // take a shot of that!
     break;
    default:
     println("OpenBCI_GUI: '" + key + "' Pressed...sending to OpenBCI...");
     if (openBCI.serial_openBCI != null) openBCI.serial_openBCI.write(key + "\n"); //send the value as ascii with a newline character
     break;
  }
}
void parseKeycode(int val) { 
  //assumes that val is Java keyCode
  switch (val) {
    case 27:
      println("OpenBCI_GUI: parseKeycode(" + val + "): received ESC keypress.  Stopping OpenBCI...");
      stopRunning();
      break; 
    default:
      println("OpenBCI_GUI: parseKeycode(" + val + "): value is not known.  Ignoring...");
      break;
  }
}
String getDateString() {
    String fname = year() + "-";
    if (month() < 10) fname=fname+"0";
    fname = fname + month() + "-";
    if (day() < 10) fname = fname + "0";
    fname = fname + day(); 
    
    fname = fname + "_";
    if (hour() < 10) fname = fname + "0";
    fname = fname + hour() + "-";
    if (minute() < 10) fname = fname + "0";
    fname = fname + minute() + "-";
    if (second() < 10) fname = fname + "0";
    fname = fname + second();
    return fname;
}
  
//swtich yard if a click is detected
void mousePressed() {
   
  //was the stopButton pressed?
//  if (gui.stopButton.isMouseHere()) { 
//    gui.stopButton.setIsActive(true);
//    stopButtonWasPressed(); 
//  }
   
  redrawScreenNow = true;  //command a redraw of the GUI whenever the mouse is pressed
}

void mouseReleased() {
  
  redrawScreenNow = true;  //command a redraw of the GUI whenever the mouse is released
}

void stopRunning() {
    if (openBCI != null) openBCI.stopDataTransfer();
    //if (wave != null) wave.amplitude.setLastValue(0); //turn off audio
    closeLogFile();
    isRunning = false;
}
void startRunning() {
    if (eegDataSource == DATASOURCE_NORMAL) openNewLogFile();  //open a new log file
    if (openBCI != null) openBCI.startDataTransfer(); //use whatever was the previous data transfer mode (TXT vs BINARY)
    isRunning = true;
}

//execute this function whenver the stop button is pressed
void stopButtonWasPressed() {
  //toggle the data transfer state of the ADS1299...stop it or start it...
  if (isRunning) {
    println("openBCI_GUI: stopButton was pressed...stopping data transfer...");
    stopRunning();
  } 
  else { //not running
    println("openBCI_GUI: startButton was pressed...starting data transfer...");
    startRunning();
    nextPlayback_millis = millis();  //used for synthesizeData and readFromFile.  This restarts the clock that keeps the playback at the right pace.
  }

  //update the push button with new text based on the current running state
  //gui.stopButton.setActive(isRunning);
  if (isRunning) {
    //println("OpenBCI_GUI: stopButtonWasPressed (a): changing string to " + Gui_Manager.stopButton_pressToStop_txt);
    //gui.stopButton.setString(Gui_Manager.stopButton_pressToStop_txt); 
  } 
  else {
    //println("OpenBCI_GUI: stopButtonWasPressed (a): changing string to " + Gui_Manager.stopButton_pressToStart_txt);
    //gui.stopButton.setString(Gui_Manager.stopButton_pressToStart_txt);
  }
}


int getPlaybackDataFromTable(Table datatable, int currentTableRowIndex, float scale_fac_uVolts_per_count, DataPacket_ADS1299 curDataPacket) {
  float val_uV = 0.0f;
  
  //check to see if we can load a value from the table
  if (currentTableRowIndex >= datatable.getRowCount()) {
    //end of file
    println("OpenBCI_GUI: hit the end of the playback data file.  starting over...");
    //if (isRunning) stopRunning();
    currentTableRowIndex = 0;
  } else {
    //get the row
    TableRow row = datatable.getRow(currentTableRowIndex);
    currentTableRowIndex++; //increment to the next row
    
    //get each value
    for (int Ichan=0; Ichan < nchan; Ichan++) {
      //if (isChannelActive(Ichan) && (Ichan < datatable.getColumnCount())) {
        val_uV = row.getFloat(Ichan);
      //} else {
      //  //use zeros for the missing channels
      //  val_uV = 0.0f;
      //}

      //put into data structure
      curDataPacket.values[Ichan] = (int) (0.5f+ val_uV / scale_fac_uVolts_per_count); //convert to counts, the 0.5 is to ensure rounding
    }
  }
  return currentTableRowIndex;
}

//toggleChannelState: : Ichan is [0 nchan-1]
//void toggleChannelState(int Ichan) {
//  if (Ichan >= 0) {
//    if (isChannelActive(Ichan)) {
//      deactivateChannel(Ichan);      
//    } 
//    else {
//      activateChannel(Ichan);
//    }
//  }
//}


////Ichan is zero referenced (not one referenced)
//boolean isChannelActive(int Ichan) {
//  boolean return_val = false;
//  
//  //account for 16 channel case...because the channel 9-16 (aka 8-15) are coupled to channels 1-8 (aka 0-7)
//  if ((Ichan > 7) && (OpenBCI_Nchannels > 8)) Ichan = Ichan - 8;
//    
//  //now check the state of the corresponding channel button
//  //if ((Ichan >= 0) && (Ichan < gui.chanButtons.length)) {
//    boolean button_is_pressed = gui.chanButtons[Ichan].isActive();
//    if (button_is_pressed) { //button is pressed, which means the channel was NOT active
//      return_val = false;
//    } else { //button is not pressed, so channel is active
//      return_val = true;
//    }
//  }
//  return return_val;
//}

//activateChannel: Ichan is [0 nchan-1] (aka zero referenced)
void activateChannel(int Ichan) {
  println("OpenBCI_GUI: activating channel " + (Ichan+1));
  if (openBCI != null) openBCI.changeChannelState(Ichan, true); //activate
  //if (Ichan < gui.chanButtons.length) gui.chanButtons[Ichan].setIsActive(false); //an active channel is a light-colored NOT-ACTIVE button
}  
void deactivateChannel(int Ichan) {
  println("OpenBCI_GUI: deactivating channel " + (Ichan+1));
  if (openBCI != null) openBCI.changeChannelState(Ichan, false); //de-activate
  //if (Ichan < gui.chanButtons.length) gui.chanButtons[Ichan].setIsActive(true); //a deactivated channel is a dark-colored ACTIVE button
}


//
//
//void setBiasState(boolean state) {
//  openBCI.isBiasAuto = state;
//  
//  //send message to openBCI
//  if (openBCI != null) openBCI.setBiasAutoState(state);
//  
//  //change button text
//  if (openBCI.isBiasAuto) {
//    gui.biasButton.but_txt = "Bias\nAuto";
//  } else {
//    gui.biasButton.but_txt = "Bias\nFixed";
//  }
//  
//}

void openNewLogFile() {
  //close the file if it's open
  if (fileoutput != null) {
    println("OpenBCI_GUI: closing log file");
    closeLogFile();
  }
  
  //open the new file
  fileoutput = new OutputFile_rawtxt(openBCI.fs_Hz);
  output_fname = fileoutput.fname;
  println("openBCI: openNewLogFile: opened output file: " + output_fname);
}

void closeLogFile() {
  if (fileoutput != null) fileoutput.closeFile();
}


//void incrementFilterConfiguration() {
//  eegProcessing.incrementFilterConfiguration();
//  
//  //update the button strings
////  gui.filtBPButton.but_txt = "BP Filt\n" + filtCoeff_bp[currentFilt_ind].short_name;
////  gui.titleMontage.string = "EEG Data (" + filtCoeff_bp[currentFilt_ind].name + ", " + filtCoeff_notch[currentFilt_ind].name + ")"; 
//  gui.filtBPButton.but_txt = "BP Filt\n" + eegProcessing.getShortFilterDescription();
//  gui.titleMontage.string = "EEG Data (" + eegProcessing.getFilterDescription() + ")"; 
//  
//}
//  
//void incrementSmoothing() {
//  smoothFac_ind++;
//  if (smoothFac_ind >= N_SMOOTHEFAC) smoothFac_ind = 0;
//  
//  //tell the GUI
//  gui.setSmoothFac(smoothFac[smoothFac_ind]);
//  
//  //update the button
//  gui.smoothingButton.but_txt = "Smooth\n" + smoothFac[smoothFac_ind];
//}
//
//void toggleShowPolarity() {
//  gui.headPlot1.use_polarity = !gui.headPlot1.use_polarity;
//  
//  //update the button
//  gui.showPolarityButton.but_txt = "Show Polarity\n" + gui.headPlot1.getUsePolarityTrueFalse();
//}
//
//void fileSelected(File selection) {  //called by the Open File dialog box after a file has been selected
//  if (selection == null) {
//    println("no selection so far...");
//  } else {
//    //inputFile = selection;
//    playbackData_fname = selection.getAbsolutePath();
//  }
//}

// here's a function to catch whenever the window is being closed, so that
// it stops OpenBCI
// from: http://forum.processing.org/one/topic/run-code-on-exit.html
//
// must add "prepareExitHandler();" in setup() for Processing sketches 
//private void prepareExitHandler () {
//  Runtime.getRuntime().addShutdownHook(
//    new Thread(new Runnable() {
//        public void run () {
//          //System.out.println("SHUTDOWN HOOK");
//          println("OpenBCI_GUI: executing shutdown code...");
//          try {
//            stopRunning();
//            if (openBCI != null) {
//              openBCI.closeSerialPort();
//            }
//            stop();
//          } catch (Exception ex) {
//            ex.printStackTrace(); // not much else to do at this point
//          }
//        }
//      }
//    )
//  );
//}  

