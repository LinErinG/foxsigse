/*
	FOXSI USB GSE USB Package ----------------------------------------
	
	PURPOSE : Library for the USB D2xx class

	REQUIRED : 
 
 	WRITTEN : Steven Christe (18-Dec-2009), based on code by L. Glesener
	MODIFIED Oct-2010 by Lindsay

	NOTES : This class is shared with the full GSE program.  The commented out lines of 
	code are used by the GSE program, please do not delete!
	
	TO DO : Read the to do in the .h file and under each of the function definitions below.
  	
*/

#include <iostream>
#include "ftd2xx.h"
#include "usbd2xx.h"
#include <pthread.h>
#include "Application.h"
#include "gui.h"

using namespace std;

extern Gui *gui;
extern Application *app;

#define XSTRIPS 128
#define YSTRIPS 128
#define CHANNELS 1024
#define FRAME_SIZE_IN_SHORTINTS 784
#define FRAME_SIZE_IN_BYTES 1568

extern int HistogramFunction[CHANNELS];
extern double detImage[XSTRIPS][YSTRIPS];
extern double detImagemask[XSTRIPS][YSTRIPS];

extern unsigned short int buffer0[FRAME_SIZE_IN_SHORTINTS];

// default constructor method
USB_d2xx::USB_d2xx()
{
	nBytesFrame = 624;
	nBytesFrame = FRAME_SIZE_IN_SHORTINTS;
	frameData = new unsigned short int [FRAME_SIZE_IN_SHORTINTS];
	ftStatus = FT_OK;
	ftHandle = NULL;
	loadDefaultSettings();
}

// construct with initializer
USB_d2xx::USB_d2xx( const int n)
{
	nBytesFrame = n;
	ftStatus = FT_OK;
	frameData = new unsigned short int [n];
	ftHandle = NULL;
	loadDefaultSettings();
}

int USB_d2xx::open(void)
{	
	// Define variables:
	
	char * 	pcBufLD[2];			// Pointer to cBufLD
	char 	cBufLD[1][64];		// Holds the device serial numbers
//	FILE *	dataFile;
//	char	dataFileName[20];
	unsigned int lostbytes = 0, framesread = 0;
	int	iNumDevs = 0;	//the number of found devices (should only be 1)
	char buffer[50];	// used to pass messages to console.
	
	// Initialize pcBufLD.  This points to the character string where the device list is stored.
	pcBufLD[0] = cBufLD[0];
	pcBufLD[1] = NULL;
	
	// Initialize USB connection (with errors)
	// first list the devices
	ftStatus = FT_ListDevices(pcBufLD, &iNumDevs, FT_LIST_ALL | FT_OPEN_BY_SERIAL_NUMBER);	
	//check if status is okay, if not display an error message
	if(ftStatus != FT_OK or iNumDevs < 1) {
		app->printf_to_console("Error: FT_ListDevices(%d)\n", NULL, ftStatus);
		return -1;
	}
	//if status okay then print out list of found devices
	for(int i = 0; i < iNumDevs; i++) {
		app->printf_to_console("Found Device %d.", NULL, i);
		app->printf_to_console("Serial Number - %s\n", cBufLD[i], 0);
	}
	
	//status is okay; now open the device
	ftStatus = FT_OpenEx(cBufLD, FT_OPEN_BY_SERIAL_NUMBER, &ftHandle);
	if(ftStatus != FT_OK){
		app->printf_to_console("Error FT_OpenEx(%d), device\n", NULL, ftStatus);
		return -1;
	}
	
	//	printf("Opened device %s\n", cBufLD);
	//FIX this code below
	//gui->consoleBuf->insert("Opened device %s\n", cBufLD);
	
	//fthandle holds the pointer to the device
	// Set data transfer rate to 9600
//	ftStatus = FT_SetBaudRate(ftHandle, 9600);
//	if(ftStatus != FT_OK){
//		char buffer[50];
//		sprintf(buffer, "Error FT_SetBaudRate(%d), cBufLD = %s\n", ftStatus, cBufLD[1]);
//		gui->consoleBuf->insert(buffer);
//		cout << buffer << endl;

//		return -1;
//	}
	// Set read and write timeouts, in millisec.  (for now, arbitrary value of 1 second)
	ftStatus = FT_SetTimeouts(ftHandle, 1000, 1000);
	if(ftStatus != FT_OK){
		sprintf(buffer, "Error FT_SetTimeouts(%d)\n", NULL, ftStatus);
		gui->consoleBuf->insert(buffer);
		return -1;
	}

	ftStatus = FT_Purge(ftHandle, 3);	// Purge write and read buffers
	if(ftStatus != FT_OK){
		app->printf_to_console("Error FT_Purge(%d)\n", NULL, ftStatus);
		return -1;
	}
	
	// set flow control.  although this is not required for FT245R chip, it can help prevent errors on the TXE line
	// Set RTS/CTS flow control
	ftStatus = FT_SetFlowControl(ftHandle, FT_FLOW_RTS_CTS, 0x11, 0x13); 
	if (ftStatus == FT_OK) { 
		cout << "SetFlowControl successful." << endl;
		// FT_SetFlowControl OK 
	} else {
		cout << "SetFlowControl failed." << endl;
		return -1;
	} 
	
	// set buffer size for transfer in function.
	ftStatus = FT_SetUSBParameters(ftHandle, 704, 0);
	if(ftStatus != FT_OK){
		app->printf_to_console("Error FT_SetUSBParameters(%d)\n", NULL, ftStatus);
		return -1;
	}
	
	//device is now open and ready!

	// default data directory, display in "Filename" buffer.
	//if(gui->filenameInput->size()==0)	gui->filenameInput->insert("/Users/foxsi/GSE/testdata/");
	
	return 0;
	
}

int USB_d2xx::findSync(void)
{
	//cout << "Starting sync\n";
	
	DWORD	nBytes = 2;		// number of bytes to read
	DWORD	nBytesRead = 0; // actual number of bytes read	
	unsigned int dataWord = 0;
	int		i = 0;			// keep track of bytes read before sync word is found
	int		iMax = 10*nBytesFrame;	// maximum times to try
	DWORD	nBytesToRead = nBytesFrame;
	
	ftStatus = FT_GetQueueStatus(ftHandle, &nBytesRead);
	ftStatus = FT_SetTimeouts(ftHandle, 500, 500);

//	if( nBytesRead < nBytesToRead )	return -1;
	
	while (1) {
		if (i == iMax){	
			return -1;
		}			
		i++;
		
		ftStatus = FT_GetQueueStatus(ftHandle, &nBytesRead);
		//cout << "Attempt " << i << " value " << dataWord << " nBytesRead " << nBytesRead << endl;
		if (nBytesRead == 0 or ftStatus != FT_OK) {
			//app->printf_to_console("Error FT_Read(%i).", NULL, ftStatus);
			return -1;
		}
		ftStatus = FT_Read(ftHandle, &dataWord, nBytes, &nBytesRead);
		// if the sync word is found, read again to see if the sync word is repeated.
		if (dataWord == 0xEB90){
			ftStatus = FT_Read(ftHandle, &dataWord, nBytes, &nBytesRead);
			if (dataWord == 0xEB90) break;
		}
	}
		return 1;
}


int USB_d2xx::readFrame(void)
{
	//cout << "Starting frame read\n";
	
	/* Read */
	int		nFrames = 1;
	DWORD	nBytesToRead = nFrames*nBytesFrame;
	DWORD 	nBytesRead = 0;	// actual number of bytes read.

	char buffer[50];

	ftStatus = FT_GetQueueStatus(ftHandle, &nBytesRead);
	ftStatus = FT_SetTimeouts(ftHandle, 500, 500);
	
	if( nBytesRead < nBytesToRead ){
//		sprintf(buffer, "Not enough data to read. %d bytes waiting.\n", nBytesRead);
//		gui->consoleBuf->insert(buffer);
		return -1;
	}
	
	for(int i=0; i<nBytesToRead; i++){  
		frameData[i] = 0;	// initialize buffer
//		cout << frameData[i] << endl;
	}
	
	if(ftStatus == FT_OK) {

//		cout << "Attempting to read " << nBytesToRead << " bytes." << endl;
//		sprintf(buffer, "Attempting to read %d bytes.\n", nBytesToRead);
//		gui->consoleBuf->insert(buffer);

		ftStatus = FT_Read(ftHandle, frameData, nBytesToRead, &nBytesRead);
		
		memcpy((void *) buffer0,(void *) frameData, FRAME_SIZE_IN_BYTES);

		if((ftStatus) != FT_OK || nBytesRead != nBytesToRead){
			//app->printf_to_console("Error FT_Read(%d)\n", NULL, ftStatus);
			//cout << buffer << endl;
			return -1;
		}
		else {			
			//app->printf_to_console("Read %d bytes.\n", NULL, nBytesRead);
		}
	} else {
		app->print_to_console("Could not get USB queue status.\n");
	}
	
	//for(int i=0; i<nBytesToRead; i++){  
	//	cout << frameData[i] << endl;
	//}	
	
	return nBytesRead;
	
//	gui->mainImageWindow->redraw();
//	gui->subImageWindow->redraw();
//	gui->mainHistogramWindow->redraw();
}

void USB_d2xx::close(void)
{
	if(ftHandle != NULL) {
		FT_Close(ftHandle);
		ftHandle = NULL;
		//fclose(dataFile);		
		app->print_to_console("Closing connection.\n");
	}
	//gui->usbReadBut->deactivate();
	//gui->usbCloseBut->deactivate();
}

void USB_d2xx::printFrame(void)
{
	// Data are stored as 16-bit words (short int)
	
//	unsigned short int data[2];
	int nData = 98; // n bytes per frame.
	int pixel_value = 0;
	int ydata[YSTRIPS];
	int xdata[XSTRIPS];
	unsigned int xmask[XSTRIPS];
	unsigned int ymask[YSTRIPS];
	int good = 0;

	// Loop through the 4 ASICS
	for(int j=0; j<4; j++)
	{
		int index = j*nData;	// used for counting words in the data packet
		// temporary variables to hold values from the data stream
		int chipBit, trigBit, seuBit, commonMode, pedestal;
		int mask1, mask2, mask3, mask4;
				
		cout << "\n ASIC " << j << endl << endl;
		
		// Sync word again.
		cout << "Sync word:\t\t"		<< hex << uppercase	<< frameData[ index ] << endl;	index++;
		cout << "Detector time:\t\t"	<< dec << frameData[ index ] << endl;	index++;
		cout << "Start bit:\t\t"		<< frameData[ index ] << endl;	index++;

		cout << "Chip data bit:\t\t"	<< frameData[ index ] << endl;
		chipBit = frameData[ index ];		
		index++;

		cout << "Analog Trigger bit: "	<< frameData[ index ] << endl;
		trigBit = frameData[ index ];
		index++;

		cout << "SEU bit:\t\t"			<< frameData[ index ] << endl;
		seuBit = frameData[ index ];
		index++;
		
		// channel mask displayed in hex
		// First two channel mask bits are stored as one word each.
		cout << "Channel mask:\t\t:"	<< hex << uppercase << frameData[ index ]; index++;
		cout << frameData[ index ]; index++;
		cout << frameData[ index ]; mask1 = frameData[ index ]; index++;
		cout << frameData[ index ]; mask2 = frameData[ index ]; index++;
		cout << frameData[ index ]; mask3 = frameData[ index ]; index++;
		cout << frameData[ index ] << endl; mask4 = frameData[ index ]; index++;
		
		cout << "Common mode:\t\t"	<< dec << frameData[ index ] << endl;
		commonMode = frameData[ index ];
		index++;
		
		for(int i=0; i<64; i++){
			cout << "Strip " << i << ":\t\t" << frameData[ index ] << endl;	index++;
		}
		
		cout << endl << mask1 << endl << mask2 << endl << mask3 << endl << mask4 << endl << seuBit << endl << endl;
		
		if((mask1 == 65535) && (mask2 == 65535) && (mask3 == 65535) && (mask4 == 65535) && (seuBit == 0)){
			good++;
		for(int i=0; i< XSTRIPS/2; i++){	
			pixel_value = (int)(frameData[ index ]);// - (unsigned short int)commonMode;
			cout << "Strip " << i << " data:\t" << (pixel_value) << endl;
			
			// Update the Histogram but limit values to 1024
			// only update for the first two ASICs (p-side)
			if (j == 0 || j == 1){HistogramFunction[pixel_value < 1024 ? pixel_value : 1024]++;}
			
			//// update for both p-side and n-side
			//HistogramFunction[pixel_value < 1024 ? pixel_value : 1024]++;
			
//			// Update the data but limit values to 1024
//			if (j == 0 || j == 1) {
//				xdata[i + j*XSTRIPS/2] = pixel_value < 1024 ? pixel_value : 1024;	}
////			xmask[i + j*XSTRIPS/2] = getbits(frameData[j*nData+3+i], i, 1);
			
//			if (j == 2 || j == 3){
//				ydata[i + (j-2)*YSTRIPS/2] = pixel_value < 1024 ? pixel_value : 1024;	
//			}

			if (j == 0) xdata[i] = 0;
			if (j == 1) xdata[i+64] = 1;
			if (j == 2) ydata[i] = 2;
			if (j == 3) ydata[i+64] = 3;
			
			index++;
		}
		}
		
		cout << "Pedestal:\t\t" << frameData[index] << endl; index++;


		frame[j] = * (asic_dataframe*) (&frameData[ j*nData ]);
		cout << "start bit:" << frame[j].start_bit << endl;
		cout << "chip bit:" << frame[j].chip_bit << endl;
		cout << "trig bit:" << frame[j].trig_bit << endl;
		cout << "seu bit:" << frame[j].seu_bit << endl;
		cout << "pedestal_bit:" << frame[j].pedestal_bit << endl;
		cout << "asic mask:" << frame[j].asic_mask << endl;
		cout << "common mode:" << frame[j].common_mode << endl;
		cout << "noise:" << frame[j].noise << endl;
		
		/*  Some leftover code
		
		// first two ASICs are the p-side ASICs and give the energy measurements
		// define these as the X strips
		
		cout << "Pedestal value:\t" << frameData[j*nData+72] << endl;
		
		data[0] = frameData[ j*nData+73] % 0x100;
		data[1] = frameData[ j*nData+73] / 0x100;

		printf("Stop bit\t%u\n", data[0]);
		printf("\n\nSYNC WORD\t\t%X\n", data[1]);	
						
		//for( int k = 0; k<64; k++){
		//	cout << k << " " << ;
		//update displays
		//HistogramFunction[dataWord%1024/10]++;
					
		//for(int j=0;j<XSTRIPS;j++)
		//{
		//	detImage[i-8][j] = dataWord%1024/10;
		//}
					
		//sprintf(buffer, "%u", dataWord%1024);
					//gui->consoleBuf->insert(buffer);
		//			cout << buffer << endl;
	}

*/
	}
		
	for (int i = 0; i<XSTRIPS; i++) {
		for (int j = 0; j<YSTRIPS; j++) {
			detImage[i][j] = xdata[i]*ydata[j];
			// If the glitch button is not ON then multiply the image by the 
			// mask image to kill the bad pixel data
			if (gui->glitchBut->value() == 0) {
			//detImage[i][j] *= xmask[i]*ymask[j];}
			detImage[i][j] *= xmask[i]*ymask[j];}
			if(i == 0 && j == 0) detImage[i][j]=10;  // set scale of intensity plot
		}
	}
		
	cout << "good = " << good << endl << endl;
	
//	if(good >= 3){
		gui->mainHistogramWindow->redraw();
		gui->mainImageWindow->redraw();
//	}
}

void USB_d2xx::writeHeader(FILE *dataFile)
{
	int i=0;
	// If first event, write data file header 
	// Header: Nevents and hold time
	fprintf( dataFile, "%u\n", (unsigned int)gui->nEvents->value());
	fprintf( dataFile, "%u\n", (unsigned int)gui->setHoldTimeWindow_holdTime->value());
	// Header: slow control values for all ASICs.
	for(i=0; i<45; i++)	fprintf( dataFile, "%u\t", asic0settings[i]);
	fprintf( dataFile, "\n");
	for(i=0; i<64; i++)	fprintf( dataFile, "%u\t", disable0[i]);
	fprintf( dataFile, "\n");
	for(i=0; i<64; i++)	fprintf( dataFile, "%u\t", test0[i]);
	fprintf( dataFile, "\n\n");
//	fprintf( dataFile, "\ndone ASIC 0 settings\n"); //debug
	for(i=0; i<45; i++)	fprintf( dataFile, "%u\t", asic1settings[i]);
	fprintf( dataFile, "\n");
	for(i=0; i<64; i++)	fprintf( dataFile, "%u\t", disable1[i]);
	fprintf( dataFile, "\n");
	for(i=0; i<64; i++)	fprintf( dataFile, "%u\t", test1[i]);
	fprintf( dataFile, "\n\n");
//	fprintf( dataFile, "\ndone ASIC 1 settings\n"); //debug
	for(i=0; i<45; i++)	fprintf( dataFile, "%u\t", asic2settings[i]);
	fprintf( dataFile, "\n");
	for(i=0; i<64; i++)	fprintf( dataFile, "%u\t", disable2[i]);
	fprintf( dataFile, "\n");
	for(i=0; i<64; i++)	fprintf( dataFile, "%u\t", test2[i]);
	fprintf( dataFile, "\n\n");
//	fprintf( dataFile, "\ndone ASIC 2 settings\n"); //debug
	for(i=0; i<45; i++)	fprintf( dataFile, "%u\t", asic3settings[i]);
	fprintf( dataFile, "\n");
	for(i=0; i<64; i++)	fprintf( dataFile, "%u\t", disable3[i]);
	fprintf( dataFile, "\n");
	for(i=0; i<64; i++)	fprintf( dataFile, "%u\t", test3[i]);
	fprintf( dataFile, "\n");
//	fprintf( dataFile, "\ndone ASIC 3 settings\n"); //debug
}

void USB_d2xx::writeFrame(FILE *dataFile)
{
	// This function is very close to the last one except that data is printed to file instead of screen.
	// Also identifiers like "start bit", "strip 1 data", etc are not written.	
	// Data are stored as 16-bit words (short int)
	
	int nData = 97; // n bytes per frame.
	int pixel_value = 0;
	int ydata[YSTRIPS];
	int xdata[XSTRIPS];
	unsigned int xmask[XSTRIPS];
	unsigned int ymask[YSTRIPS];
	int good = 0;
	int index = 0;
		
	// Loop through the 4 ASICS
	for(int j=0; j<4; j++)
	{
//		int index = j*nData;	// used for counting words in the data packet
		
		fprintf( dataFile, "\n\n" );
		
		// for the first ASIC, we need to artificially write the sync word.
		// for the other ASICs, it's already in the data stream.
		if(j==0){
			fprintf( dataFile, "eb90\t");
		} else{
			fprintf( dataFile, "%x\t", frameData[index]);	index++;
		}
		
		fprintf( dataFile, "%u\t", frameData[index]);	index++;	//frame counter
		fprintf( dataFile, "%x\t", frameData[index]);	index++;	//start
		fprintf( dataFile, "%x\t", frameData[index]);	index++;	//chip
		fprintf( dataFile, "%x\t", frameData[index]);	index++;	//trigger
		fprintf( dataFile, "%x\t", frameData[index]);	index++;	//seu
				
		// channel mask displayed in hex
		// First two channel mask bits are stored as one word each.
		fprintf( dataFile, "%x", frameData[index]);	index++;
		fprintf( dataFile, "%x", frameData[index]);	index++;
		fprintf( dataFile, "%x", frameData[index]);	index++;
		fprintf( dataFile, "%x", frameData[index]);	index++;
		fprintf( dataFile, "%x", frameData[index]);	index++;
		fprintf( dataFile, "%x\t", frameData[index]);	index++;

		fprintf( dataFile, "%u\t", frameData[index]);	index++;	//pedestal
		
		// strip data
		for(int i=0; i<64; i++){
			fprintf( dataFile, "%u\t", frameData[index]);	index++;
		}
		
		fprintf( dataFile, "%u\t", frameData[index]);	index++;	//common mode
		
		// 10 extra words of readout information
		fprintf( dataFile, "%u\t", frameData[index]);	index++;
		fprintf( dataFile, "%u\t", frameData[index]);	index++;
		fprintf( dataFile, "%u\t", frameData[index]);	index++;
		fprintf( dataFile, "%u\t", frameData[index]);	index++;
		fprintf( dataFile, "%u\t", frameData[index]);	index++;
		fprintf( dataFile, "%u\t", frameData[index]);	index++;
		fprintf( dataFile, "%u\t", frameData[index]);	index++;
		fprintf( dataFile, "%u\t", frameData[index]);	index++;
		fprintf( dataFile, "%u\t", frameData[index]);	index++;
		fprintf( dataFile, "%u\t", frameData[index]);	index++;		
					
	}	
	
}	

// ADDED JULY 2011
// Set individual ASIC slow control parameters
void USB_d2xx::setConfig(void)
{
	const int n=867;	// number of components in write array
	const int nV=45;	// number of values in sendParamsWindow
	char cBufWrite[n];	// write array
	DWORD 	dwBytesWritten;  // returns number of bytes written.

	// load user input configuration settings from sendParamsWindow.
	int value[nV];
	int chan[64];
	int test[64];
	int asic = gui->sendParamsWindow_asic->value();
	
	// initialize configuration arrays
	for(int i=0; i<nV; i++)	value[i] = 0;
	for(int i=0; i<64; i++)	chan[i] = 0;
	for(int i=0; i<64; i++)	test[i] = 0;
	
	// initialize write buffer.
	for(int i=0; i<n; i++) cBufWrite[i] = 0;
	
	// fill arrays with values from sendParameters window.
	// CHAN values are from the channel disable buttons.
	for(int i=0; i<nV; i++)	value[i] = gui->sendParamsWindow_value[i]->value();
	for(int i=0; i<64; i++)	chan[i]  = gui->sendParamsWindow_chan[i]->value();
	for(int i=0; i<64; i++)	test[i]  = gui->sendParamsWindow_test[i]->value();
	
	// logic to assemble configuration settings into write array.
	for(int j=0; j<11; j++)
		for(int i=0; i<3; i++) cBufWrite[3*j+i] = getbits(value[44-j], 2-i, 1);
	for(int j=0; j<4; j++)
		for(int i=0; i<4; i++) cBufWrite[4*j+i+33] = getbits(value[33-j], 3-i, 1);
	for(int i=0; i<5; i++)	cBufWrite[i+49] = getbits(value[29], 4-i, 1);
	cBufWrite[54] = getbits(value[28], 0, 1);
	cBufWrite[55] = getbits(value[27], 0, 1);
	cBufWrite[56] = getbits(value[26], 0, 1);
	for(int i=0; i<64; i++)		cBufWrite[i+57] = test[63-i];
	for(int i=0; i<256; i++)	cBufWrite[i+121] = 0;
	for(int i=0; i<64; i++)		cBufWrite[i+377] = chan[63-i];	
	for(int i=0; i<10; i++)		cBufWrite[i+441] = getbits(value[25], 9-i, 1);
	for(int i=0; i<390; i++)	cBufWrite[i+451] = 0;
	for(int i=0; i<24; i++)		cBufWrite[i+841] = getbits(value[23-i], 0, 1);
	cBufWrite[865] = 0;
	cBufWrite[866] = 0;
	
	// Add identifier so data is routed properly in the FPGA
	for(int i=0; i<n-1; i++) cBufWrite[i] = cBufWrite[i+1] + 32*asic;

	/* Write */
	dwBytesWritten = 0;
	if((ftStatus = FT_Write(ftHandle, cBufWrite, n, &dwBytesWritten)) != FT_OK) {
		printf("Error FT_Write(%d)\n", ftStatus);
		return;
	}
			
	cout << "Wrote " << dwBytesWritten << " bytes." << endl << endl;
			
}

// Added July 2011
// Set global configuration
void USB_d2xx::setGlobalConfig(int option)
{
	const int n = 867;	// number bytes to send (same as control register width b/c FPGA comm will expect 867 bytes)
	int value = 0;	
	char cBufWrite[n];	// write array
	DWORD 	dwBytesWritten;  // returns number of bytes written.

	// initialize write buffer.
	for(int i=0; i<n; i++) cBufWrite[i] = 0;

	switch (option) {
		case 0:
			value = gui->setHoldTimeWindow_holdTime->value();
			break;
		case 1:
			value = gui->setTrigWindow_timeoutTime->value();
			break;
		case 2:
			value = gui->setTrigWindow_useTimeout->value();
			break;
		case 3:
			value = gui->setTrigWindow_delayTime->value();
			break;
		default:
			value = 0;
			break;
	}

	value = value + 128;							// put '1' in MSB as flag that this is a global setting.
	value = value + option*32;						// put indicator in bits [6:5] to tell FPGA which kind of setting it is.

	for(int i=0; i<n; i++) cBufWrite[i] = value;
//	cBufWrite[0] = value;
	
	/* Write */
	dwBytesWritten = 0;
	if((ftStatus = FT_Write(ftHandle, cBufWrite, 1, &dwBytesWritten)) != FT_OK) {
		printf("Error FT_Write(%d)\n", ftStatus);
		return;
	}
	
	cout << "Wrote " << dwBytesWritten << " bytes, value: " << dec << cBufWrite[0] << " value is " << value << endl << endl;	
	printf("Wrote %d bytes, value %d\n\n", dwBytesWritten, cBufWrite[0]);
}

/*
// Added August 2011
// Break acquisition loop.
void USB_d2xx::breakAcq(int data)
{
	const int n = 1;	// number bytes to send
	char cBufWrite[n];	// write array
	DWORD 	dwBytesWritten;  // returns number of bytes written.
	
	// initialize write buffer.
	cBufWrite[0] = data;
	
	// Write
	dwBytesWritten = 0;
	if((ftStatus = FT_Write(ftHandle, cBufWrite, n, &dwBytesWritten)) != FT_OK) {
		printf("Error FT_Write(%d)\n", ftStatus);
		return;
	}
	
	cout << "Wrote " << dwBytesWritten << " bytes, value: " << dec << cBufWrite[0] << endl << endl;	
	printf("Wrote %d bytes, value %d\n\n", dwBytesWritten, cBufWrite[0]);
}
*/

// Added August 2011
// Save configuration settings for the selected ASIC.
// save values for this ASIC.
void USB_d2xx::saveSettings()
{
	int nVal = 45;
	int nChan = 64;
	int asic = 	gui->sendParamsWindow_asic->value();
	
	switch (asic) {
		case 0:
			for(int i=0; i<nVal; i++)	asic0settings[i] = gui->sendParamsWindow_value[i]->value();
			for(int i=0; i<nChan; i++)	disable0[i] = gui->sendParamsWindow_chan[i]->value();
			for(int i=0; i<nChan; i++)	test0[i] = gui->sendParamsWindow_test[i]->value();
			break;
		case 1:
			for(int i=0; i<nVal; i++)	asic1settings[i] = gui->sendParamsWindow_value[i]->value();
			for(int i=0; i<nChan; i++)	disable1[i] = gui->sendParamsWindow_chan[i]->value();
			for(int i=0; i<nChan; i++)	test1[i] = gui->sendParamsWindow_test[i]->value();
			break;
		case 2:
			for(int i=0; i<nVal; i++)	asic2settings[i] = gui->sendParamsWindow_value[i]->value();
			for(int i=0; i<nChan; i++)	disable2[i] = gui->sendParamsWindow_chan[i]->value();
			for(int i=0; i<nChan; i++)	test2[i] = gui->sendParamsWindow_test[i]->value();
			break;
		case 3:
			for(int i=0; i<nVal; i++)	asic3settings[i] = gui->sendParamsWindow_value[i]->value();
			for(int i=0; i<nChan; i++)	disable3[i] = gui->sendParamsWindow_chan[i]->value();
			for(int i=0; i<nChan; i++)	test3[i] = gui->sendParamsWindow_test[i]->value();
			break;
		default:
			break;
	}
	
	// testing
	//	for(int i=0; i<nVal; i++)
	//		cout <<  "  " << asic0settings[i] << "  " << asic1settings[i] << "  " << asic2settings[i] << "  " << asic3settings[i] << endl;

}

// Added August 2011
// Restore configuration settings for the selected ASIC.
void USB_d2xx::restoreSettings()
{
	int nVal = 45;
	int nChan = 64;
	int asic = 	gui->sendParamsWindow_asic->value();
	
	switch (asic) {
		case 0:
			for(int i=0; i<nVal; i++)	gui->sendParamsWindow_value[i]->value(asic0settings[i]);
			for(int i=0; i<nChan; i++)	gui->sendParamsWindow_chan[i]->value(disable0[i]);
			for(int i=0; i<nChan; i++)	gui->sendParamsWindow_test[i]->value(test0[i]);
			break;
		case 1:
			for(int i=0; i<nVal; i++)	gui->sendParamsWindow_value[i]->value(asic1settings[i]);
			for(int i=0; i<nChan; i++)	gui->sendParamsWindow_chan[i]->value(disable1[i]);
			for(int i=0; i<nChan; i++)	gui->sendParamsWindow_test[i]->value(test1[i]);
			break;
		case 2:
			for(int i=0; i<nVal; i++)	gui->sendParamsWindow_value[i]->value(asic2settings[i]);
			for(int i=0; i<nChan; i++)	gui->sendParamsWindow_chan[i]->value(disable2[i]);
			for(int i=0; i<nChan; i++)	gui->sendParamsWindow_test[i]->value(test2[i]);
			break;
		case 3:
			for(int i=0; i<nVal; i++)	gui->sendParamsWindow_value[i]->value(asic3settings[i]);
			for(int i=0; i<nChan; i++)	gui->sendParamsWindow_chan[i]->value(disable3[i]);
			for(int i=0; i<nChan; i++)	gui->sendParamsWindow_test[i]->value(test3[i]);
			break;
		default:
			break;
	}
	
}

// This function sets the default configuration settings.  For now, these are hardcoded.
void USB_d2xx::loadDefaultSettings()
{
	
	asic0settings[0] = 0;	asic1settings[0] = 0;	asic2settings[0] = 0;	asic3settings[0] = 0;
	asic0settings[1] = 0;	asic1settings[1] = 0;	asic2settings[1] = 0;	asic3settings[1] = 0;
	asic0settings[2] = 0;	asic1settings[2] = 0;	asic2settings[2] = 0;	asic3settings[2] = 0;
	asic0settings[3] = 1;	asic1settings[3] = 1;	asic2settings[3] = 1;	asic3settings[3] = 1;  // sbi_hp2
	asic0settings[4] = 0;	asic1settings[4] = 0;	asic2settings[4] = 0;	asic3settings[4] = 0;
	asic0settings[5] = 0;	asic1settings[5] = 0;	asic2settings[5] = 0;	asic3settings[5] = 0;
	asic0settings[6] = 0;	asic1settings[6] = 0;	asic2settings[6] = 0;	asic3settings[6] = 0;
	asic0settings[7] = 1;	asic1settings[7] = 1;	asic2settings[7] = 1;	asic3settings[7] = 1;	// ro_all
	asic0settings[8] = 0;	asic1settings[8] = 0;	asic2settings[8] = 0;	asic3settings[8] = 0;
	asic0settings[9] = 1;	asic1settings[9] = 1;	asic2settings[9] = 1;	asic3settings[9] = 1;	// preb_hp
	asic0settings[10] = 0;	asic1settings[10] = 0;	asic2settings[10] = 0;	asic3settings[10] = 0;
	asic0settings[11] = 0;	asic1settings[11] = 0;	asic2settings[11] = 0;	asic3settings[11] = 0;
	asic0settings[12] = 1;	asic1settings[12] = 1;	asic2settings[12] = 0;	asic3settings[12] = 0;	// nside
	asic0settings[13] = 0;	asic1settings[13] = 0;	asic2settings[13] = 0;	asic3settings[13] = 0;
	asic0settings[14] = 0;	asic1settings[14] = 0;	asic2settings[14] = 0;	asic3settings[14] = 0;
	asic0settings[15] = 0;	asic1settings[15] = 0;	asic2settings[15] = 0;	asic3settings[15] = 0;
	asic0settings[16] = 1;	asic1settings[16] = 1;	asic2settings[16] = 0;	asic3settings[16] = 0;	// negQ
	asic0settings[17] = 0;	asic1settings[17] = 0;	asic2settings[17] = 0;	asic3settings[17] = 0;
	asic0settings[18] = 0;	asic1settings[18] = 0;	asic2settings[18] = 0;	asic3settings[18] = 0;
	asic0settings[19] = 0;	asic1settings[19] = 0;	asic2settings[19] = 0;	asic3settings[19] = 0;
	asic0settings[20] = 0;	asic1settings[20] = 0;	asic2settings[20] = 0;	asic3settings[20] = 0;
	asic0settings[21] = 0;	asic1settings[21] = 0;	asic2settings[21] = 0;	asic3settings[21] = 0;
	asic0settings[22] = 0;	asic1settings[22] = 0;	asic2settings[22] = 0;	asic3settings[22] = 0;
	asic0settings[23] = 0;	asic1settings[23] = 0;	asic2settings[23] = 0;	asic3settings[23] = 0;
	asic0settings[24] = 0;	asic1settings[24] = 0;	asic2settings[24] = 0;	asic3settings[24] = 0;
	asic0settings[25] = 0;	asic1settings[25] = 0;	asic2settings[25] = 0;	asic3settings[25] = 0;
	asic0settings[26] = 0;	asic1settings[26] = 0;	asic2settings[26] = 0;	asic3settings[26] = 0;
	asic0settings[27] = 0;	asic1settings[27] = 0;	asic2settings[27] = 0;	asic3settings[27] = 0;
	asic0settings[28] = 0;	asic1settings[28] = 0;	asic2settings[28] = 0;	asic3settings[28] = 0;
	asic0settings[29] = 31;	asic1settings[29] = 31;	asic2settings[29] = 15;	asic3settings[29] = 15;	// vthr
	asic0settings[30] = 10;	asic1settings[30] = 10;	asic2settings[30] = 6;	asic3settings[30] = 6;	// ifp
	asic0settings[31] = 0;	asic1settings[31] = 0;	asic2settings[31] = 9;	asic3settings[31] = 9;	// iramp
	asic0settings[32] = 0;	asic1settings[32] = 0;	asic2settings[32] = 0;	asic3settings[32] = 0;
	asic0settings[33] = 0;	asic1settings[33] = 0;	asic2settings[33] = 0;	asic3settings[33] = 0;
	asic0settings[34] = 0;	asic1settings[34] = 0;	asic2settings[34] = 0;	asic3settings[34] = 0;
	asic0settings[35] = 7;	asic1settings[35] = 7;	asic2settings[35] = 7;	asic3settings[35] = 7;	// shabias
	asic0settings[36] = 1;	asic1settings[36] = 1;	asic2settings[36] = 0;	asic3settings[36] = 0;	// ifss
	asic0settings[37] = 0;	asic1settings[37] = 0;	asic2settings[37] = 0;	asic3settings[37] = 0;
	asic0settings[38] = 0;	asic1settings[38] = 0;	asic2settings[38] = 0;	asic3settings[38] = 0;
	asic0settings[39] = 0;	asic1settings[39] = 0;	asic2settings[39] = 0;	asic3settings[39] = 0;
	asic0settings[40] = 7;	asic1settings[40] = 7;	asic2settings[40] = 7;	asic3settings[40] = 7;	// prebias
	asic0settings[41] = 0;	asic1settings[41] = 0;	asic2settings[41] = 0;	asic3settings[41] = 0;
	asic0settings[42] = 0;	asic1settings[42] = 0;	asic2settings[42] = 0;	asic3settings[42] = 0;
	asic0settings[43] = 5;	asic1settings[43] = 5;	asic2settings[43] = 1;	asic3settings[43] = 1;	// ioffset
	asic0settings[44] = 0;	asic1settings[44] = 0;	asic2settings[44] = 0;	asic3settings[44] = 0;
		
	for(int i=0; i<64; i++){
		disable0[i] = 1;
		disable1[i] = 1;
		disable2[i] = 0;
		disable3[i] = 0;
		test0[i] = 0;
		test1[i] = 0;
		test2[i] = 0;
		test3[i] = 0;
	}
	
}

