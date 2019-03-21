#include "xnumx.h"
#include "main.h"
#include "stm32f4xx_hal.h"

u8 findFirstChar(char * a,char a2,char len){
	for(u8 i=0;i<len;i++){
		if(a[i]==a2){
			return i;
		}
	}
		return len;
}
u8 findLastChar(char * a,char a2,char len){
	for(u8 i=len;i>0;i--){
		if(a[i-1]==a2){
			return i-1;
		}
	}
		return len;
}


void xnumx(char * rx,char rxLen,char * separator,double * mwData,char mwDataLen)
{
	
	u8 mwAddr[mwDataLen][2];
	u32 temp=0;
	s32 longNum[]={
	1.0,
	10.0,
	100.0,
	1000.0,
	10000.0,
	100000.0,
	1000000.0,
	10000000.0,
	100000000.0,
	1000000000.0
	};
	
	/*
	����������ִ��룬���ϣ����������0����ӣ����ϣ�������ǲ��ı䣬�򲻼ӡ�
	for(char i = 0; i < mwDataLen; i++)
	{
		mwData[i]=0;
	}
	*/

	for(u8 k=0,i=0;i<mwDataLen;i++,k=0){
		/*ȡ�ÿ�ʼ������ַ*/
		mwAddr[i][0] = findFirstChar(rx,separator[i],rxLen);
		mwAddr[i][1] = findLastChar(rx,separator[i],rxLen);
		
		/*����x number x��ʽ*/
		if(mwAddr[i][0]!=mwAddr[i][1]){
			mwData[i]=0;
			for(u8 j=0,len=mwAddr[i][1]-mwAddr[i][0]-1;j<len;j++){
				if(rx[mwAddr[i][1]-1-j]=='.'){
						k=j;
						continue;
					}
					if(rx[mwAddr[i][1]-1-j]=='-'){
						mwData[i] = 0 - mwData[i];
						break;
					}
					if(0==k){
						temp = (rx[mwAddr[i][1]-1-j]-48)*longNum[j];
					}else{
						temp = (rx[mwAddr[i][1]-1-j]-48)*longNum[j-1];
					}
					mwData[i] += temp;
			}
			mwData[i] = mwData[i]/longNum[k];
		}
			
	}
	
	
}
