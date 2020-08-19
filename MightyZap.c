/*
 * wiringSerial.c:
 *	Handle a serial port
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <wiringPi.h>
#include "MightyZap.h"

/*
 * serialOpen:
 *	Open and initialise the serial port, setting all the right
 *	port parameters - or as many as are required - hopefully!
 *********************************************************************************
 */
int MightyZapSetup(int DirectionPin,int Level)
{
  MightyZap_DirPin=DirectionPin;

  if(Level == 0)
  {
    MightyZap_DirPin_Level_Tx = Low;//tx
    MightyZap_DirPin_Level_Rx = High;
  }
  else 
  {
    MightyZap_DirPin_Level_Tx = High;//tx
    MightyZap_DirPin_Level_Rx = Low;
  }
  
  pinMode(MightyZap_DirPin, OUTPUT);
  //MightyZap_sheild =1;
}

int OpenMightyZap (const char *device, const int baud)
{
  struct termios options ;
  speed_t myBaud ;
  int     status ;

  
  switch (baud)
  {
  
    case    9600:	myBaud =    B9600 ; break ;
    case   19200:	myBaud =   B19200 ; break ;
    case   38400:	myBaud =   B38400 ; break ;
    case   57600:	myBaud =   B57600 ; break ;
    case  115200:	myBaud =  B115200 ; break ;

    default:
      return -2 ;
  }

  if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
    return -1 ;

  fcntl (fd, F_SETFL, O_RDWR) ;

// Get and modify current options:

  tcgetattr (fd, &options) ;

    cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

  tcsetattr (fd, TCSANOW, &options) ;

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);

  usleep (10000) ;	// 10mS
  digitalWrite( MightyZap_DirPin, MightyZap_DirPin_Level_Tx);// TX Enable

  mIRPtxrxStatus = 0;
  mBusUsed = 0;// only 1 when tx/rx is operated
  SmartDelayFlag=1;

  setPacketType(IRP_PACKET_TYPE1);

  return fd ;
}

int readRaw(void)
{
  int temp=0;
  if (available(fd)) 
  {
    temp = serialGetchar(fd);
  }
  return temp;
}

void writeRaw(char value)
{
  serialPutchar(fd,value);
}

void irpTxEnable(void)
{
	//if(MightyZap_sheild==1)
	digitalWrite( MightyZap_DirPin, MightyZap_DirPin_Level_Tx );// RX Disable
}
void irpTxDisable(void)
{
	//if(MightyZap_sheild==1)
	digitalWrite( MightyZap_DirPin, MightyZap_DirPin_Level_Rx );// RX Enable
}


void clearBuffer(void)
{
	while((available(fd)))
	{
		serialGetchar(fd);
	}
}

void setPacketType(int type){
	mPacketType = type;
	if(mPacketType == IRP_PACKET_TYPE2){
		mPktIdIndex = 4;
		mPktLengthIndex = 5;
		mPktInstIndex = 7;
		mPktErrorIndex = 8;
		//mRxLengthOffset = 7;
	}else{           // irp 1.0
		mPktIdIndex = 3; //2;
		mPktLengthIndex = 4; //3;
		mPktInstIndex = 5; //4;
		mPktErrorIndex = 5; //4;
		//mRxLengthOffset = 4;
	}
}

int getPacketType(void)
{
	return mPacketType;
}
int checkPacketType(void)
{
	return 0;//default is 1.0 protocol IRP_PACKET_TYPE1
}
/*
 * return value for getTxRxStatus(), getResult();
#define	COMM_TXSUCCESS		(0)
#define COMM_RXSUCCESS		(1)
#define COMM_TXFAIL			(2)
#define COMM_RXFAIL			(3)
#define COMM_TXERROR		(4)
#define COMM_RXWAITING		(5)
#define COMM_RXTIMEOUT		(6)
#define COMM_RXCORRUPT		(7)
*/

int getTxRxStatus(void)
{
	return mIRPtxrxStatus;
}

/*
 * Use getTxRxStatus() instead of getResult()
 * */
 
int  getResult(void){
	//	return mCommStatus;
	return getTxRxStatus();
}

int getError( int errbit ){
	return 0;
}

int txPacket(int bID, int bInstruction, int bParameterLength){

    int bCount,bCheckSum,bPacketLength;
	
	//int bCount,bCheckSum,bPacketLength;

    int offsetParamIndex;
   
	mTxBuffer[0] = 0xff;
	mTxBuffer[1] = 0xff;
	mTxBuffer[2] = 0xff; //
	mTxBuffer[3] = bID;  //[2]
	mTxBuffer[4] = bParameterLength+2; //[3] //2(int) <- instruction(1int) + checksum(1int) 
	mTxBuffer[5] = bInstruction; //[4]

	offsetParamIndex = 6; //5
	bPacketLength = bParameterLength+3+4; //+2+4;

  
    //copy parameters from mParamBuffer to mTxBuffer
    for(bCount = 0; bCount < bParameterLength; bCount++)
    {
    	mTxBuffer[bCount+offsetParamIndex] = mParamBuffer[bCount];
    }

	// chech sum
    bCheckSum = 0;
    for(bCount = 3; bCount < bPacketLength-1; bCount++){ //except 0xff,checksum //bCount = 2;
		bCheckSum += mTxBuffer[bCount];
	}
    mTxBuffer[bCount] = ~bCheckSum; //Writing Checksum with Bit Inversion
  
    //TxDStringC("bPacketLength = ");TxDHex8C(bPacketLength);TxDStringC("\r\n");
   
	
    char tempdata[bPacketLength];
 irpTxEnable(); // this define is declared in irp.h
    for(bCount = 0; bCount < bPacketLength; bCount++)
    {
	serialPutchar(fd,mTxBuffer[bCount]);
	delayMicroseconds (110) ;

    }
	flushPacket(fd);


    irpTxDisable();// this define is declared in irp.h

    return(bPacketLength); // return packet length
}  

int rxPacket(int bRxLength){

	unsigned long ulCounter, ulTimeLimit;
	int bCount, bLength, bChecksum;
	//int bCount, bLength, bChecksum; 

	int bTimeout;

	//#if false
	bTimeout = 0;
	if(bRxLength == 255 || bRxLength == 0xffff) 
		ulTimeLimit = RX_TIMEOUT_COUNT1;
	else
		ulTimeLimit = RX_TIMEOUT_COUNT2;

	for(bCount = 0; bCount < bRxLength; bCount++)
	{
		ulCounter = 0;
		while(!(available(fd)))
		{

			nDelay(NANO_TIME_DELAY); 
			if(ulCounter++ > ulTimeLimit)
			{
				printf("tgg\n");
				bTimeout = 1;		
				break;
			}
			uDelay(0); //if exist IRP 1.0 -> ok IRP 2.0 -> ok, if not exist 1.0 not ok, 2.0 ok
		}
		if(bTimeout) {break;}
		mRxBuffer[bCount] = readRaw(); // MightyZap_Serial->read(); // get packet data from USART device

	}
	bLength = bCount;
	bChecksum = 0;
	if( mTxBuffer[mPktIdIndex] != IRP_BROADCAST_ID )
	{
		if(bTimeout && bRxLength != 255)
		{
		#ifdef PRINT_OUT_COMMUNICATION_ERROR_TO_USART2
			printf("Rx Timeout");
			printf("%d",bLength);
		#endif
			mIRPtxrxStatus |= (1<<COMM_RXTIMEOUT);
			clearBuffer();

			return 0;
		}
		
		if(bLength > 3) //checking available length.
		{
			if(mRxBuffer[0] != 0xff || mRxBuffer[1] != 0xff || mRxBuffer[2] != 0xff ) mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXHEADER); //|| mRxBuffer[2] != 0xff
			else if(mRxBuffer[mPktIdIndex] != mTxBuffer[mPktIdIndex] ) mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXID);
			else if(mRxBuffer[mPktLengthIndex] != bLength-mPktInstIndex) mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXLENGTH);
			else{
				for(bCount = 3; bCount < bLength; bCount++){ //bCount = 2
					bChecksum += mRxBuffer[bCount]; //Calculate checksum of received data for compare
				}
				if(bChecksum != 0xff) mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXCHECKSUM);
				//return 0;
			}
			
			
			
			if(mRxBuffer[0] != 0xff || mRxBuffer[1] != 0xff || mRxBuffer[2] != 0xff ){ //|| mRxBuffer[2] != 0xff
			#ifdef PRINT_OUT_COMMUNICATION_ERROR_TO_USART2
				printf("Wrong Header");//[Wrong Header]
			#endif
				mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXHEADER);
				clearBuffer();
				return 0;
			}
			

			if(mRxBuffer[mPktIdIndex] != mTxBuffer[mPktIdIndex] )  //id check
			{
			#ifdef PRINT_OUT_COMMUNICATION_ERROR_TO_USART2
			if(update_crc_ir(0, mRxBuffer, bRxLength-2) == bChecksum)
				printf("[Error:TxID != RxID]");
			#endif
				mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXID);
				clearBuffer();
				return 0;
			}

			if(mRxBuffer[mPktLengthIndex] != bLength-mPktInstIndex) // status packet length check
			{
			#ifdef PRINT_OUT_COMMUNICATION_ERROR_TO_USART2
				printf("RxLength Error");
			#endif
				mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXLENGTH);
				clearBuffer();
				return 0;
			}

		
			for(bCount = 3; bCount < bLength; bCount++){ //bCount = 2
				bChecksum += mRxBuffer[bCount]; //Calculate checksum of received data for compare
			}

			bChecksum &= 0xff;

			if(bChecksum != 0xff)
			{
			#ifdef PRINT_OUT_COMMUNICATION_ERROR_TO_USART2
				printf("[RxChksum Error]");
			#endif
				mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXCHECKSUM);
				clearBuffer();
				return 0;
			}
		//end of checksum
		}//(bLength > 3)
	}//end of Rx status packet check
//	#endif
	return bLength;
}
void printBuffer(int *bpPrintBuffer, int bLength)
{
#ifdef	PRINT_OUT_TRACE_ERROR_PRINT_TO_USART2
	int bCount;
	if(bLength == 0)
	{
		if(mTxBuffer[3] == IRP_BROADCAST_ID)
		{
			printf("\r\n No Data[at Broadcast ID 0xFE]");
		}
		else
		{
			printf("\r\n No Data(Check ID, Operating Mode, Baud rate)");//TxDString("\r\n No Data(Check ID, Operating Mode, Baud rate)");
		}
	}
	for(bCount = 0; bCount < bLength; bCount++)
	{
		printf("%d ",bpPrintBuffer[bCount]);		
	}
	printf(" LEN:");//("(LEN:")
	printf("%d",bLength)	
	printf("\r\n");
#endif
}

int txRxPacket(int bID, int bInst, int bTxParaLen){
//#if false
	mIRPtxrxStatus = 0;

	int bTxLen, bRxLenEx, bTryCount;

	mBusUsed = 1;
	mRxLength = bRxLenEx = bTxLen = 0;

//	for(bTryCount = 0; bTryCount < gbIRPNumberTxRxAttempts; bTryCount++)
	for(bTryCount = 0; bTryCount < 1; bTryCount++)
	{
		while((available(fd))){
			serialGetchar(fd);
		}
		
		/**************************************   Transfer packet  ***************************************************/
		bTxLen = txPacket(bID, bInst, bTxParaLen);
		
		if (bTxLen == (bTxParaLen+4+3))	mIRPtxrxStatus = (1<<COMM_TXSUCCESS); //+4+2
				
		if(bInst == CMD_PING){		
			if(bID == IRP_BROADCAST_ID)	mRxLength = bRxLenEx = 0xff;
			else mRxLength = bRxLenEx = 7; //6; // basic response packet length			
		}
		else if(bInst == CMD_READ){
			mRxLength = bRxLenEx = 7+mParamBuffer[1]; //6+
		}
		else if( bID == IRP_BROADCAST_ID ){
			if(bInst == CMD_SYNC_READ || bInst == CMD_BULK_READ) mRxLength = bRxLenEx = 0xffff; //only 2.0 case
			else mRxLength = bRxLenEx = 0; // no response packet
		}
		else{
			if (gbIRPStatusReturnLevel>1){
				if(mPacketType == IRP_PACKET_TYPE1) mRxLength = bRxLenEx = 7; //6 //+mParamBuffer[1];
				else mRxLength = bRxLenEx = 11;
			}
			else{
				mRxLength = bRxLenEx = 0;
			}

		}

		if(bRxLenEx){

			if(SmartDelayFlag == 1)
				delay(150);
			/**************************************   Receive packet  ***************************************************/

			mRxLength = rxPacket(bRxLenEx);
			

		}//bRxLenEx is exist
	} //for() gbIRPNumberTxRxAttempts

	//TxDStringC("\r\n TEST POINT 2");//TxDString("\r\n Err ID:0x");
	mBusUsed = 0;
	
	if((mRxLength != bRxLenEx) && (mTxBuffer[mPktIdIndex] != IRP_BROADCAST_ID))
	{
		//TxDByteC('3');//
		//TxDStringC("Rx Error\r\n");//TxDString("\r\n Err ID:0x");
#ifdef	PRINT_OUT_COMMUNICATION_ERROR_TO_USART2
		//TxDString("\r\n Err ID:0x");
		//TxDHex8(bID);
		printf("\r\n ->[IRP]Err: ");
		//printBuffer(mTxBuffer,bTxLen);
		//TxDStringC("\r\n <-[IRP]Err: ");
		//printBuffer(mRxBuffer,mRxLength);
#endif

#ifdef	PRINT_OUT_TRACE_ERROR_PRINT_TO_USART2
		//TxDString("\r\n {[ERROR:");TxD16Hex(0x8100);TxDByte(':');TxD16Hex(bID);TxDByte(':');TxD8Hex(bInst);TxDByte(']');TxDByte('}');
		//TxDByte(bID);TxDByte(' ');
		//TxDByte(bInst);TxDByte(' ');
		//TxDByte(gbpParameter[0]);TxDByte(' ');
		//TxDByte(gbpParameter[1]);TxDByte(' ');
#endif
	//	return 0;
	}else if((mRxLength == 0) && (mTxBuffer[mPktInstIndex] == CMD_PING)){ 
		return 0;
	}
	
	//TxDString("\r\n TEST POINT 4");//TxDString("\r\n Err ID:0x");
#ifdef PRINT_OUT_PACKET_TO_USART2
	printf("\r\n ->[TX Buffer]: ");
	//printBuffer(mTxBuffer,bTxLen);
	printf("\r\n <-[RX Buffer]: ");
	//printBuffer(mRxBuffer,mRxLength);
#endif
	mIRPtxrxStatus = (1<<COMM_RXSUCCESS);

	//gbLengthForPacketMaking =0;
//	#endif
	return 1;
}


int fast_rxPacket(int bRxLength){

	unsigned long ulCounter, ulTimeLimit;
	int bCount, bLength, bChecksum;

	int bTimeout;

	//#if false
	bTimeout = 0;
	//if(bRxLength == 255 || bRxLength == 0xffff) 
	//	ulTimeLimit = RX_TIMEOUT_COUNT1;
	//else
		ulTimeLimit = RX_TIMEOUT_COUNT2;

	for(bCount = 0; bCount < bRxLength; bCount++)
	{
		ulCounter = 0;
		while(!(available(fd)))
		{
			nDelay(NANO_TIME_DELAY); 
			if(ulCounter++ > ulTimeLimit)
			{
				bTimeout = 1;		
				break;
			}
			uDelay(0); //if exist IRP 1.0 -> ok IRP 2.0 -> ok, if not exist 1.0 not ok, 2.0 ok
		}
		if(bTimeout) {break;}
		mRxBuffer[bCount] = readRaw(); // MightyZap_Serial->read(); // get packet data from USART device
		//mRxBuffer[bCount] = serialGetchar(fd);

	}	
	bLength = bCount;
	bChecksum=0;
	

	if((mRxLength == 0) && (mTxBuffer[mPktInstIndex] == CMD_PING)) 
		return 0;
	return bLength;
}



int fast_txRxPacket(int bID, int bInst, int bTxParaLen){

	mIRPtxrxStatus = 0;

	int bTxLen, bRxLenEx, bTryCount;

	mBusUsed = 1;
	mRxLength = bRxLenEx = bTxLen = 0;

	
	while((available(fd))){
		serialGetchar(fd);
	}
	
	/**************************************   Transfer packet  ***************************************************/
	bTxLen = txPacket(bID, bInst, bTxParaLen);
	
	if (bTxLen == (bTxParaLen+4+3))	mIRPtxrxStatus = (1<<COMM_TXSUCCESS); //+4+2
					
	mRxLength = bRxLenEx = 7+mParamBuffer[1]; //6+
	
	if(bRxLenEx){

		mRxLength = rxPacket(bRxLenEx);			
		
	} 


	mBusUsed = 0;
	
	mIRPtxrxStatus = (1<<COMM_RXSUCCESS);

	return 1;
}

//serialFlush
int flushPacket(int fd)
{
	serialFlush(fd);
	return 0;
}

int Dummy(int tmp){
	return tmp;
}
void uDelay(int uTime){
	int cnt, max;
	static int tmp = 0;

	for( max=0; max < uTime; max++)
	{
		for( cnt=0; cnt < 10 ; cnt++ )
		{
			tmp +=Dummy(cnt);
		}
	}
		//tmpdly = tmp;
}
void nDelay(int nTime){
	int cnt, max;
		cnt=0;
		static int tmp = 0;

		for( max=0; max < nTime; max++)
		{
			//for( cnt=0; cnt < 10 ; cnt++ )
			//{
				tmp +=Dummy(cnt);
			//}
		}
		//tmpdly = tmp;
}


int  ping(int  bID ){

	if(txRxPacket(bID, CMD_PING, 0)){
		if(mPacketType == IRP_PACKET_TYPE1) return (mRxBuffer[3]); //1.0
		else return IRP_MAKEint(mRxBuffer[9],mRxBuffer[10]); 
	}else{
		return 0xff;  //no irp in bus.
	}

}


int  writeByte(int bID, int bAddress, int bData){
	int param_length = 0;

	mParamBuffer[0] = bAddress;
	mParamBuffer[1] = bData;
	param_length = 2;
	
	return txRxPacket(bID, CMD_WRITE, param_length);
}

int readByte(int bID, int bAddress){
	clearBuffer();
	
	mParamBuffer[0] = bAddress;
	mParamBuffer[1] = 1;
	if( txRxPacket(bID, CMD_READ, 2 )){
		mCommStatus = 1;
		return(mRxBuffer[6]);// [5] //refer to 1.0 packet structure
	}
	else{
		mCommStatus = 0;
		return 0xff;
	}
}



int writeint(int bID, int bAddress, short wData){
    int param_length = 0;
    clearBuffer();

	mParamBuffer[0] = bAddress;
	mParamBuffer[1] = IRP_LOBYTE(wData);//(int)(wData&0xff);
	mParamBuffer[2] = IRP_HIBYTE(wData);//(int)((wData>>8)&0xff);
	param_length = 3;
	
	return txRxPacket(bID, CMD_WRITE, param_length);

}



int readint(int bID, int bAddress){
	clearBuffer();
	
	mParamBuffer[0] = bAddress;
	mParamBuffer[1] = 2;
	if(txRxPacket(bID, CMD_READ, 2)){
		return IRP_MAKEint(mRxBuffer[6],mRxBuffer[7]);//( (((int)mRxBuffer[6])<<8)+ mRxBuffer[5] );
	}
	else{
		return 0xffff;
	}	
}

#if 0
int bulkRead(int *param, int param_length){
	//mResult = 0;
	unsigned long bulkReadlength=0;

	int n, i, k=0;
	int num = param_length / 5; // each length : 5 (ID ADDR_L ADDR_H LEN_L LEN_H)
	// int pkt_length = param_length + 3;  // 3 : INST CHKSUM_L CHKSUM_H
	//   unsigned char txpacket[MAXNUM_TXPACKET] = {0};
	//   unsigned char rxpacket[MAXNUM_RXPACKET] = {0};

	for(n=0; n < param_length; n++){
		mParamBuffer[n] = param[n];
	}

	/************ TxRxPacket *************/
	// Wait for Bus Idle
	/*    while(comm->iBusUsing == 1)
	{
		//Sleep(0);
	}*/

	//mResult = txrx_PacketEx(IRP_BROADCAST_ID, CMD_BULK_READ_EX, param_length);;
	txRxPacket(IRP_BROADCAST_ID, CMD_BULK_READ, param_length);


	for(n = 0; n < num; n++){
	   // int id = param[n*5+0];

		bulkReadlength = rxPacket(param_length+11);
		/*result =  IRP_MAKEDint(	IRP_MAKEint(gbpRxBufferEx[9],gbpRxBufferEx[10]),
						IRP_MAKEint(gbpRxBufferEx[11],gbpRxBufferEx[12])
					  );*/
		if(mRxBuffer[7] == 0x55){ //packet instruction index
			mBulkData[n].iID = mRxBuffer[4]; //packet ID index
			mBulkData[n].iAddr = IRP_MAKEint(mParamBuffer[5*n+1],mParamBuffer[5*n+2]); //get address
			mBulkData[n].iLength = IRP_MAKEint(mParamBuffer[5*n+3],mParamBuffer[5*n+4]);//IRP_MAKEint(gbpRxBufferEx[PKT_LENGTH_L],gbpRxBufferEx[PKT_LENGTH_H]);
			//TxDStringC("iLength = ");TxDHex8C(mBulkData[n].iLength);TxDStringC("\r\n");
			mBulkData[n].iError = mRxBuffer[7+1]; //Error code
			for(i=0; i < mBulkData[n].iLength ; i++){
				mBulkData[n].iData[i] = mRxBuffer[7+2+i]; //DATA1
			}
		}
		for(k=0;k < IRP_RX_BUF_SIZE ; k++){
			mRxBuffer[k] = 0; //buffer clear
		}
		clearBuffer();
	}
	return bulkReadlength;

}
#endif


void setTxPacketId(int id){
	mbIDForPacketMaking = id;

}
void setTxPacketInstruction(int instruction){
	mbInstructionForPacketMaking = instruction;

}
void setTxPacketParameter( int index, int value ){
	mParamBuffer[index] = value;

}
void setTxPacketLength( int length ){
	mbLengthForPacketMaking = length;

}
int txrxPacket(void){
	mCommStatus = txRxPacket(mbIDForPacketMaking, mbInstructionForPacketMaking, mbLengthForPacketMaking);
	return mCommStatus;
}

int getRxPacketParameter( int index ){
	//return irp_get_rxpacket_parameter( index );
	return mRxBuffer[6 + index]; //5
}
int getRxPacketLength(void){
	//return irp_get_rxpacket_length();
	return mRxBuffer[4]; //length index is 3 in status packet //3
}

void goalPosition(int bID,int position){	
	writeint(bID, 0x86, position);	
}
// Parameter Map
/****************************************************************/
/* Force Enable							*/
/* Type : Read/write						*/
/****************************************************************/
void forceEnable(int bID,int enable){
	writeByte(bID, 0x80, enable);
}

/****************************************************************/
/* Alarm LED													*/
/* Type : Read/Write											*/
/****************************************************************/
void setAlarmLed(int bID, int flag){	
	writeByte(bID, 17, flag);		
}
int getAlarmLed(int bID){
	return readByte(bID, 17);
}
/****************************************************************/
/* Alarm Shutdown												*/
/* Type : Read/Write											*/
/****************************************************************/
void setAlarmShutdown(int bID,int flag){
	writeByte(bID, 18, flag);
}
int getAlarmShutdown(int bID){
	return readByte(bID, 18);
}


/****************************************************************/
/* Moving Speed							*/
/* Type : Read/write						*/
/****************************************************************/
void movingSpeed(int bID,int value){	
	writeint(bID, 0x88, value);	
}


/****************************************************************/
/* Present Position 						*/
/* Type : Read Only						*/
/****************************************************************/
int presentPosition(int bID){	
	return readint(bID, 0x8c);
}

int getFastPresentPosition(int bID){	
	clearBuffer();
	
	mParamBuffer[0] = 0x8c;
	mParamBuffer[1] = 2;
	if(fast_txRxPacket(bID, CMD_READ, 2)){
		return IRP_MAKEint(mRxBuffer[6],mRxBuffer[7]);//( (((int)mRxBuffer[6])<<8)+ mRxBuffer[5] );
	}
	else{
		return 0xffff;
	}	
}

/*
 * serialFlush:
 *	Flush the serial buffers (both tx & rx)
 *********************************************************************************
 */

 
void serialFlush (const int fd)
{
  tcflush (fd, TCIOFLUSH) ;
}


/*
 * serialClose:
 *	Release the serial port
 *********************************************************************************
 */

void CloseMightyZap (const int fd)
{
  close (fd) ;
}


/*
 * serialPutchar:
 *	Send a single character to the serial port
 *********************************************************************************
 */

void serialPutchar (const int fd, const unsigned char c)
{	
  write (fd, &c, 1) ;
}


/*
 * serialPuts:
 *	Send a string to the serial port
 *********************************************************************************
 */

void serialPuts (const int fd, const char *s)
{
  write (fd, s, strlen (s)) ;
}

int  serialGets(int fd, int maxlen, char * str)  {
   int i=0;
   int c=-1;

   if(maxlen<0) maxlen=0;
   do { 
      if(i>=maxlen) {
         str[maxlen]='\0';
         i=maxlen;
         c='\0';
         break;
      }
      c = available(fd) ;
      if (c !=-1) {
         c = serialGetchar(fd) ;
         if(c==-1)  {break;}         
         if(c==EOF)  {c='\0'; break;}  // substitute EOF  by  \0 termination
         else str[i++]=c;
         if(c=='\n')  {str[i]='\0'; break;}  // if \n then add  \0 termination, then finished!
      }
   } while ((c!='\0') && (c!=-1) && (c!='\n')  && (c!=EOF) ) ;   // partially redundant
   if(c==-1) return c;
   else return i;
}

/*
 * serialPrintf:
 *	Printf over Serial
 *********************************************************************************
 */

void serialPrintf (const int fd, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  serialPuts (fd, buffer) ;
}


/*
 * serialDataAvail:
 *	Return the number of bytes of data avalable to be read in the serial port
 *********************************************************************************
 */

int available (const int fd)
{
  int result ;

  if (ioctl (fd, FIONREAD, &result) == -1)
    return -1 ;

  return result ;
}


/*
 * serialGetchar:
 *	Get a single character from the serial device.
 *	Note: Zero is a valid character and this function will time-out after
 *	10 seconds.
 *********************************************************************************
 */

int serialGetchar (const int fd)
{
  uint8_t x ;

  if (read (fd, &x, 1) != 1)
    return -1 ;

  return ((int)x) & 0xFF ;
}
int readError(int bID)
{
	ping(bID);
	return mRxBuffer[5];
}
void write_Addr(int bID,int addr, int size,int Data)
{
	if(size==1)
		writeByte(bID,addr,Data);
	else 
		writeint(bID,addr,Data);
}
int read_Addr(int bID, int addr ,int size)
{
	if(size==1)
		return readByte(bID,addr);
	else 
		return readint(bID,addr);
}
