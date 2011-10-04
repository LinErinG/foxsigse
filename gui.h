// generated by Fast Light User Interface Designer (fluid) version 1.0300

#ifndef gui_h
#define gui_h
#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include "Application.h"
#include <stdlib.h>
#include "Foxsidata.h"
#include "usbd2xx.h"
#include <FL/Fl_Menu_Bar.H>
#include "mainHistogram.h"
#include "mainImage.h"
#include "subImage.h"
#include <FL/Fl_Group.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Light_Button.H>
#include <FL/Fl_Value_Output.H>
#include "mainChart.h"
#include <FL/Fl_Text_Display.H>
#include <FL/Fl_Choice.H>
#include "mainLightcurve.h"
#include <FL/Fl_Value_Input.H>

class Gui {
public:
  Gui();
  Fl_Double_Window *mainWindow;
  Fl_Menu_Bar *menuBar;
  static Fl_Menu_Item menu_menuBar[];
  static Fl_Menu_Item *fileMenu;
  static Fl_Menu_Item *simulateData;
private:
  void cb_simulateData_i(Fl_Menu_*, void*);
  static void cb_simulateData(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *readFile;
private:
  void cb_readFile_i(Fl_Menu_*, void*);
  static void cb_readFile(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *readUSBStream;
private:
  void cb_readUSBStream_i(Fl_Menu_*, void*);
  static void cb_readUSBStream(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *readTeleStream;
private:
  void cb_readTeleStream_i(Fl_Menu_*, void*);
  static void cb_readTeleStream(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *WritePicScreen;
private:
  void cb_WritePicScreen_i(Fl_Menu_*, void*);
  static void cb_WritePicScreen(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *WriteLightcurve;
private:
  void cb_WriteLightcurve_i(Fl_Menu_*, void*);
  static void cb_WriteLightcurve(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *exitButton;
private:
  void cb_exitButton_i(Fl_Menu_*, void*);
  static void cb_exitButton(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *menuProc;
  static Fl_Menu_Item *Sync;
private:
  void cb_Sync_i(Fl_Menu_*, void*);
  static void cb_Sync(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *Detector;
  static Fl_Menu_Item *setDetector0;
private:
  void cb_setDetector0_i(Fl_Menu_*, void*);
  static void cb_setDetector0(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *setDetector1;
private:
  void cb_setDetector1_i(Fl_Menu_*, void*);
  static void cb_setDetector1(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *setDetector2;
private:
  void cb_setDetector2_i(Fl_Menu_*, void*);
  static void cb_setDetector2(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *setDetector3;
private:
  void cb_setDetector3_i(Fl_Menu_*, void*);
  static void cb_setDetector3(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *setDetector4;
private:
  void cb_setDetector4_i(Fl_Menu_*, void*);
  static void cb_setDetector4(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *setDetector5;
private:
  void cb_setDetector5_i(Fl_Menu_*, void*);
  static void cb_setDetector5(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *setDetector6;
private:
  void cb_setDetector6_i(Fl_Menu_*, void*);
  static void cb_setDetector6(Fl_Menu_*, void*);
public:
  static Fl_Menu_Item *setDetector7;
private:
  void cb_setDetector7_i(Fl_Menu_*, void*);
  static void cb_setDetector7(Fl_Menu_*, void*);
public:
  mainHistogram *mainHistogramWindow;
  mainImage *mainImageWindow;
  subImage *subImageWindow;
  Fl_Group *TopOutput;
  Fl_Output *rateOutput0;
  Fl_Output *rateOutput1;
  Fl_Output *rateOutput2;
  Fl_Output *rateOutput3;
  Fl_Output *rateOutput4;
  Fl_Output *rateOutput5;
  Fl_Output *rateOutput6;
  Fl_Output *rateOutput7;
  Fl_Output *framenumOutput;
  Fl_Output *shutterstateOutput;
private:
  void cb_shutterstateOutput_i(Fl_Output*, void*);
  static void cb_shutterstateOutput(Fl_Output*, void*);
public:
  Fl_Output *TempOutput;
  Fl_Output *histEnergy;
  Fl_Output *histCounts;
  Fl_Group *dataPlayback;
  Fl_Button *nextFrameBut;
private:
  void cb_nextFrameBut_i(Fl_Button*, void*);
  static void cb_nextFrameBut(Fl_Button*, void*);
public:
  Fl_Button *prevFrameBut;
private:
  void cb_prevFrameBut_i(Fl_Button*, void*);
  static void cb_prevFrameBut(Fl_Button*, void*);
public:
  Fl_Light_Button *syncLightBut;
  Fl_Button *flushBut;
private:
  void cb_flushBut_i(Fl_Button*, void*);
  static void cb_flushBut(Fl_Button*, void*);
public:
  Fl_Button *printFrame;
private:
  void cb_printFrame_i(Fl_Button*, void*);
  static void cb_printFrame(Fl_Button*, void*);
public:
  Fl_Value_Output *frameTime;
  Fl_Value_Output *chipbitValOut0;
  Fl_Value_Output *chipbitValOut1;
  Fl_Value_Output *chipbitValOut2;
  Fl_Value_Output *chipbitValOut3;
  Fl_Value_Output *trigbitValOut0;
  Fl_Value_Output *trigbitValOut1;
  Fl_Value_Output *trigbitValOut2;
  Fl_Value_Output *trigbitValOut3;
  Fl_Value_Output *seubitValOut0;
  Fl_Value_Output *seubitValOut1;
  Fl_Value_Output *seubitValOut2;
  Fl_Value_Output *seubitValOut3;
  Fl_Value_Output *noiseValOut0;
  Fl_Value_Output *noiseValOut1;
  Fl_Value_Output *noiseValOut2;
  Fl_Value_Output *noiseValOut3;
  mainChart *mainChartWindow;
  Fl_Output *pixelNum;
  Fl_Output *pixelCounts;
  Fl_Light_Button *subImageLockbut;
  Fl_Light_Button *mainHistogramLockbut;
  mainChart *mainChartWindow1;
  Fl_Value_Output *mainHistogramXlabelmid;
  Fl_Value_Output *mainHistogramXlabelmax;
  Fl_Value_Output *mainHistogramYlabelmax;
  Fl_Value_Output *mainHistogramYlabelmid;
  Fl_Text_Display *consoleBuf;
  static Fl_Menu_Item menu_Detector[];
  static Fl_Menu_Item menu_Data[];
  Fl_Light_Button *initializeBut;
private:
  void cb_initializeBut_i(Fl_Light_Button*, void*);
  static void cb_initializeBut(Fl_Light_Button*, void*);
public:
  Fl_Button *startReadingDataButton;
private:
  void cb_startReadingDataButton_i(Fl_Button*, void*);
  static void cb_startReadingDataButton(Fl_Button*, void*);
public:
  Fl_Light_Button *closeBut;
private:
  void cb_closeBut_i(Fl_Light_Button*, void*);
  static void cb_closeBut(Fl_Light_Button*, void*);
public:
  mainLightcurve *mainLightcurveWindow;
  Fl_Light_Button *glitchBut;
  Fl_Button *sendParamsBut;
private:
  void cb_sendParamsBut_i(Fl_Button*, void*);
  static void cb_sendParamsBut(Fl_Button*, void*);
public:
  Fl_Button *testBut;
private:
  void cb_testBut_i(Fl_Button*, void*);
  static void cb_testBut(Fl_Button*, void*);
public:
  Fl_Value_Input *nEvents;
  Fl_Light_Button *writeFileBut;
private:
  void cb_writeFileBut_i(Fl_Light_Button*, void*);
  static void cb_writeFileBut(Fl_Light_Button*, void*);
public:
  Fl_Value_Input *nEventsDone;
private:
  void cb_BREAK_i(Fl_Button*, void*);
  static void cb_BREAK(Fl_Button*, void*);
  void cb_CLEAR_i(Fl_Button*, void*);
  static void cb_CLEAR(Fl_Button*, void*);
public:
  Fl_Button *stopReadingDataButton;
private:
  void cb_stopReadingDataButton_i(Fl_Button*, void*);
  static void cb_stopReadingDataButton(Fl_Button*, void*);
public:
  Fl_Button *setHoldBut;
private:
  void cb_setHoldBut_i(Fl_Button*, void*);
  static void cb_setHoldBut(Fl_Button*, void*);
public:
  Fl_Double_Window *sendParamsWindow;
  Fl_Button *sendParamsWindow_sendBut;
private:
  void cb_sendParamsWindow_sendBut_i(Fl_Button*, void*);
  static void cb_sendParamsWindow_sendBut(Fl_Button*, void*);
  void cb_Close_i(Fl_Button*, void*);
  static void cb_Close(Fl_Button*, void*);
public:
  Fl_Value_Input *sendParamsWindow_value[45];
  Fl_Light_Button *sendParamsWindow_chan[64];
  Fl_Value_Input *sendParamsWindow_asic;
private:
  void cb_sendParamsWindow_asic_i(Fl_Value_Input*, void*);
  static void cb_sendParamsWindow_asic(Fl_Value_Input*, void*);
public:
  Fl_Value_Input *sendParamsWindow_holdTime;
private:
  void cb_Set_i(Fl_Button*, void*);
  static void cb_Set(Fl_Button*, void*);
  void cb_set_i(Fl_Button*, void*);
  static void cb_set(Fl_Button*, void*);
  void cb_clear_i(Fl_Button*, void*);
  static void cb_clear(Fl_Button*, void*);
public:
  Fl_Double_Window *setHoldTimeWindow;
  Fl_Value_Input *setHoldTimeWindow_holdTime;
  Fl_Button *setHoldTimeWindow_setBut;
private:
  void cb_setHoldTimeWindow_setBut_i(Fl_Button*, void*);
  static void cb_setHoldTimeWindow_setBut(Fl_Button*, void*);
public:
  Fl_Button *setHoldTimeWindow_autorunBut;
private:
  void cb_setHoldTimeWindow_autorunBut_i(Fl_Button*, void*);
  static void cb_setHoldTimeWindow_autorunBut(Fl_Button*, void*);
public:
  void show();
  Application *app; 
  Foxsidata *data; 
  USB_d2xx *usb; 
  Fl_Text_Buffer *buff; 
};
#endif
