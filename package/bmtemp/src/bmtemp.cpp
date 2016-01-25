/********************************************************************
 * BMTEMP - utility to work with BM1707 and MP707R USB thermometer
 *
 * Licence? None. This software is free to use.
 *
 * Author: bubbafix
 * v0.2.0: sakhapovi
 * Initial code samples were given by Serg
 * http://usbsergdev.narod.ru/
 *
 *******************************************************************/

#include <stdio.h>
#include <usb.h>
#include <time.h>

#define VENDOR_ID	0x16c0
#define PRODUCT_ID	0x05df
#define DEBUG		0
#define EXIT_OK		0
#define EXIT_NO_ARG	1
#define EXIT_NO_SENSOR	2
#define EXIT_ERROR	127
#define VERSION		"0.2.0"
#define USB_REPEAT      5

static	int			ONEWIRE_COUNT;		// number of ROM
static	unsigned long long	ONEWIRE_ROM[128];	// ROM identifiers
static	unsigned char		USB_BUFI [8];		// input buffer
static	unsigned char		USB_BUFO [8];		// output buffer
static	usb_dev_handle		*USB;			// USB device
static	bool			showInfo=false;		// whether to show device info
static	bool			doScan=false;		// whether to scan for sensors (overrides showInfo)
static	bool			getTempOnce=false;	// whether to get temperature for one sensor only (overrides doScan)
static	bool			getTempAll=false;	// whether to get temperature for all sensors (overrides getTempOnce)
static	bool			getTempAllNM=false;	// whether to output narodmon.ru formatted data (overrides getTempOnce and getTempAll)
static	bool			verbose=false;		// whether to show more info (not only data)
static	bool			statChanged=false;	// whether to set port status
static	unsigned long long	sensor;			// sensor id for getTempOnce
static  bool			pset;

const static int timeout=5000; /* timeout in ms */

usb_dev_handle *udev = NULL;

/*
 * msleep - delay in milliseconds
 * thanks to 'mysurface' (http://cc.byexamples.com/2007/05/25/nanosleep-is-better-than-sleep-and-usleep/)
 */
int msleep(unsigned long milisec)
{
    struct timespec req={0};
    time_t sec=(int)(milisec/1000);
    milisec=milisec-(sec*1000);
    req.tv_sec=sec;
    req.tv_nsec=milisec*1000000L;
    while(nanosleep(&req,&req)==-1)
         continue;
    return 1;
}

/*
 * find_device - find usb device by vendor/product id
 */
usb_dev_handle* find_device()
{
	struct usb_bus *bus;
	struct usb_device *dev;

	for (bus = usb_get_busses(); bus; bus = bus->next)
	for (dev = bus->devices; dev; dev = dev->next)
	{
		if (dev->descriptor.idVendor == VENDOR_ID &&
		    dev->descriptor.idProduct == PRODUCT_ID)
		{
			//usb_dev_handle *udev;
			if (!(udev = usb_open(dev)))
			{
				return NULL;
			}
			char s1[256];
			char s2[256];
			usb_get_string_simple(udev, dev->descriptor.iManufacturer, s1, 256);
			usb_get_string_simple(udev, dev->descriptor.iProduct, s2, 256);
			if ( verbose ) printf("[.] Device: %s - %s\n", s1, s2);

			return udev;
		}
	}
	return NULL;
}

/*
 * setup - initial setup for USB device
 */
usb_dev_handle* setup()
{

	if ( DEBUG ) printf("[D] enter setup\n");
	//usb_dev_handle *udev;

	usb_set_debug(0);
	usb_init();
	usb_find_busses();
	usb_find_devices();

	if (!(udev = find_device()))
	{
		if ( verbose ) printf("[-] Device not found\n");
		return NULL;
	}

	usb_detach_kernel_driver_np(udev, 0);
//	usb_detach_kernel_driver_np(udev, 1);

	if (usb_set_configuration(udev, 1) < 0)
	{
		printf("[!] usb_set_configuration failed\n");
		usb_close(udev);

		return NULL;
	}

	if (usb_claim_interface(udev, 0) < 0)
	{
		printf("[!] usb_claim_interface 0 failed\n");
		usb_close(udev);

		return NULL;
	}

	return udev;
}

/*
 * device_close - release USB device
 */
void device_close(usb_dev_handle *udev)
{
	if ( DEBUG ) printf("[D] enter device_close\n");
	usb_release_interface(udev, 0);
	usb_close(udev);
}

/*
 * hid_send_feature_report - send data to USB device
 */
int hid_send_feature_report(usb_dev_handle *dev, const unsigned char *data, size_t length)
{
	if ( DEBUG ) printf("[D] enter hid_send_feature_report\n");
	if ( DEBUG ) printf("[D] hid_send_feature_report: data=");
	if ( DEBUG ) for (size_t k=0; k<length; k++) printf("0x%02X ", data[k]);
	if ( DEBUG ) printf("\n");

	int res = -1;
	res = usb_control_msg(dev, 0x21, 0x01, 0x0300, 0x00, (char *) data, length, 5000);

	if ( DEBUG ) printf("[D] hid_send_feature_report: res=0x%02X\n", res);
/*
	if ( DEBUG ) printf("[D] hid_send_feature_report: after call data=");
	if ( DEBUG ) for (size_t k=0; k<length; k++) printf("0x%02X ", data[k]);
	if ( DEBUG ) printf("\n");
*/
	return res;
}

/*
 * hid_get_feature_report - get data from USB device
 */
int hid_get_feature_report(usb_dev_handle *dev, unsigned char *data, size_t length)
{
	if ( DEBUG ) printf("[D] enter hid_get_feature_report\n");
/*
	if ( DEBUG ) printf("[D] hid_get_feature_report: data=");
	if ( DEBUG ) for (size_t k=0; k<length; k++) printf("0x%02X ", data[k]);
	if ( DEBUG ) printf("\n");
*/
	int res = -1;
	res = usb_control_msg(dev, 0xA1, 0x01, 0x0300, 0x00, (char *) data, length, 5000);

	if ( DEBUG ) printf("[D] hid_get_feature_report: res=0x%02X\n", res);
	if ( DEBUG ) printf("[D] hid_get_feature_report: after call data=");
	if ( DEBUG ) for (size_t k=0; k<length; k++) printf("0x%02X ", data[k]);
	if ( DEBUG ) printf("\n");

	return res;
}

/*
 * cleanup input and output buffers
 */
void USB_BUF_CLEAR()
{
    if ( DEBUG ) printf("[D] enter USB_BUF_CLEAR\n");

    for (int i=0; i<9; i++) { USB_BUFI[i]=0; USB_BUFO[i]=0; }
}

/*
 * read data from device to input buffer
 */
bool USB_GET_FEATURE()
{
    bool RESULT=false;
    int i=3;
    while (!RESULT & ((i--)>0))
        try {
	    if(pset)
	    	RESULT=hid_get_feature_report(udev, (unsigned char *)USB_BUFI, 8);
	    else
		RESULT=hid_get_feature_report(USB, &USB_BUFI[1], 8);
	    //RESULT=hid_get_feature_report(udev, (unsigned char *)USB_BUFI, 8);
        }
            catch (...) { RESULT=false; };
    if (!RESULT) printf("Error reading from device\n");
    return RESULT;
}

/*
 * write data from output buffer to device
 */
bool USB_SET_FEATURE()
{
    bool RESULT=false;
    try { 
	if(pset)
		RESULT=hid_send_feature_report(udev, (unsigned char *)USB_BUFO, 8);
	else
		RESULT=hid_send_feature_report(USB, &USB_BUFO[1], 8);
	//RESULT=hid_send_feature_report(udev, (unsigned char *)USB_BUFO, 8);
    }
        catch (...) { RESULT=false;  };
    if (!RESULT) printf("Error writing to device\n");
    return RESULT;
}

/*
 * get family
 */
bool USB_GET_FAMILY(unsigned char &FAMILY)
{   //  reading device group code, 2ms
    USB_BUF_CLEAR();
    bool RESULT=false;
    USB_BUFO[1]=0x1D;
    int i=3;   //  try count
    while (!RESULT & ((i--)>0))
        if (USB_SET_FEATURE())
            if (USB_GET_FEATURE())
                if (USB_BUFI[1]==0x1D) { RESULT=true; FAMILY=USB_BUFI[2]; }
                    else RESULT=false;
    if (!RESULT) printf("[!] USB_GET_FAMILY: Error reading family\n");
    return RESULT;
}

/*
 * get firmware version
 */
bool USB_GET_SOFTV(unsigned int &SV)
{   //  reading firmware version, 2ms
    USB_BUF_CLEAR();
    bool RESULT=false;
    USB_BUFO[1]=0x1D;
    int i=3;   // number of attempts
    while (!RESULT & ((i--)>0))
        if (USB_SET_FEATURE())
            if (USB_GET_FEATURE())
                if (USB_BUFI[1]==0x1D) { RESULT=true; SV=USB_BUFI[3]+(USB_BUFI[4]<<8); }
                    else RESULT=false;
    if (!RESULT) printf("[!] USB_GET_SOFTV: Error reading firmware version\n");
    return RESULT;
}

/*
 * get device ID
 */
bool USB_GET_ID(unsigned int &ID)
{   //  reading devide ID, 2ms
    USB_BUF_CLEAR();
    bool RESULT=false;
    USB_BUFO[1]=0x1D;
    int i=3;   //  number of attempts
    while (!RESULT & ((i--)>0))
        if (USB_SET_FEATURE())
            if (USB_GET_FEATURE())
                if (USB_BUFI[1]==0x1D) { RESULT=true; ID=(USB_BUFI[5]<<24)+(USB_BUFI[6]<<16)+(USB_BUFI[7]<<8)+USB_BUFI[8]; }
                    else RESULT=false;
    if (!RESULT) printf("[!] USB_GET_ID: Error reading devide ID\n");
    return RESULT;
}

/*
 * RESET, ~3ms
 */
bool OW_RESET()
{
    if ( DEBUG ) printf("[D] enter OW_RESET\n");
    bool RESULT=false;
    USB_BUF_CLEAR();
    USB_BUFO[1]=0x18;    USB_BUFO[2]=0x48;
    unsigned char N=3;
    while (!RESULT &((N--)>0))
        if (USB_SET_FEATURE())
        {
	    if ( DEBUG ) printf("[D] OW_RESET: if USB_SET_FEATURE() true...\n");
            msleep(2);
            if (USB_GET_FEATURE()) 
	    {
		if ( DEBUG ) printf("[D] OW_RESET: if USB_GET_FEATURE() true...\n");
		RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x48)&(USB_BUFI[3]==0x00);
	    } else RESULT=false;
        }
    if (!RESULT) printf("[!] Error OW_RESET\n");
    return RESULT;
}

/*
 * read 2 bits, 3ms
 */
bool OW_READ_2BIT(unsigned char &B)
{
    if ( DEBUG ) printf("[D] enter OW_READ_2BIT\n");
    bool RESULT=false;
    USB_BUF_CLEAR();
    USB_BUFO[1]=0x18;    USB_BUFO[2]=0x82;
    USB_BUFO[3]=0x01;    USB_BUFO[4]=0x01;
    if (USB_SET_FEATURE())
        {
        msleep(1);
        if (USB_GET_FEATURE())
            { RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x82); B=(USB_BUFI[3]&0x01)+((USB_BUFI[4]<<1)&0x02); }
        }
    if (!RESULT) printf("[!] Error OW_READ_2BIT\n");
    return RESULT;
}

/*
 * read 1 byte, 3ms
 */
bool OW_READ_BYTE(unsigned char &B)
{
    if ( DEBUG ) printf("[D] enter OW_READ_BYTE\n");
    bool RESULT=false;
    USB_BUF_CLEAR();
    USB_BUFO[1]=0x18;    USB_BUFO[2]=0x88;    USB_BUFO[3]=0xFF;
    if (USB_SET_FEATURE())
        {
        msleep(1);
        if (USB_GET_FEATURE())
            { RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x88); B=USB_BUFI[3]; }
        }
    if (!RESULT) printf("[!] Error OW_READ_BYTE\n");
    return RESULT;
}

/*
 * read 4 bytes, 4ms
 */
bool OW_READ_4BYTE(unsigned long &B)
{
    if ( DEBUG ) printf("[D] enter OW_READ_4BYTE\n");
    bool RESULT=false;
    USB_BUF_CLEAR();
    USB_BUFO[1]=0x18;    USB_BUFO[2]=0x84;    USB_BUFO[3]=0xFF;
    USB_BUFO[4]=0xFF;    USB_BUFO[5]=0xFF;    USB_BUFO[6]=0xFF;
    if (USB_SET_FEATURE())
        {
        msleep(2);
        if (USB_GET_FEATURE())
            { RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x84); B=USB_BUFI[3]+(USB_BUFI[4]<<8)+(USB_BUFI[5]<<16)+(USB_BUFI[6]<<24); }
        }
    if (!RESULT) printf("[!] Error OW_READ_4BYTE\n");
    return RESULT;
}

/*
 * write 1 bit, 3ms
 */
bool OW_WRITE_BIT(unsigned char B)
{
    if ( DEBUG ) printf("[D] enter OW_WRITE_BIT\n");
    bool RESULT=false;
    USB_BUF_CLEAR();
    USB_BUFO[1]=0x18;    USB_BUFO[2]=0x81;    USB_BUFO[3]=B&0x01;
    if (USB_SET_FEATURE())
        {
        msleep(1);
        if (USB_GET_FEATURE())
            RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x81)&((USB_BUFI[3]&0x01)==(B&0x01));
        }
    if (!RESULT) printf("[!] Error OW_WRITE_BIT\n");
    return RESULT;
}

/*
 * write 1 byte, 3ms
 */
bool OW_WRITE_BYTE(unsigned char B)
{
    if ( DEBUG ) printf("[D] enter OW_WRITE_BYTE\n");
    bool RESULT=false;
    USB_BUF_CLEAR();
    USB_BUFO[1]=0x18;    USB_BUFO[2]=0x88;    USB_BUFO[3]=B;
    if (USB_SET_FEATURE())
        {
        msleep(1);
        if (USB_GET_FEATURE())
            RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x88)&(USB_BUFI[3]==B);
        }
    if (!RESULT) printf("[!] Error OW_WRITE_BYTE\n");
    return RESULT;
}

/*
 * write 4 bytes, 4ms
 */
bool OW_WRITE_4BYTE(unsigned long B)
{
    if ( DEBUG ) printf("[D] enter OW_WRITE_4BYTE\n");
    bool RESULT=false;
    unsigned char D0, D1, D2, D3;
    D0=B&0xFF;
    D1=(B>>8) &0xFF;
    D2=(B>>16)&0xFF;
    D3=(B>>24)&0xFF;
    USB_BUF_CLEAR();
    USB_BUFO[1]=0x18;    USB_BUFO[2]=0x84;    USB_BUFO[3]=B&0xFF;
    USB_BUFO[4]=(B>>8)&0xFF;
    USB_BUFO[5]=(B>>16)&0xFF;
    USB_BUFO[6]=(B>>24)&0xFF;
    if (USB_SET_FEATURE())
        {
        msleep(2);
        if (USB_GET_FEATURE())
            RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x84)&(USB_BUFI[3]==D0&(USB_BUFI[4]==D1)&(USB_BUFI[5]==D2)&(USB_BUFI[6]==D3));
        }
    if (!RESULT) printf("[!] Error OW_WRITE_4BYTE\n");
    return RESULT;
}

bool USB_GET_PORT(unsigned char &PS)
{
    USB_BUF_CLEAR();
    bool RESULT=false;
    USB_BUFO[0]=0x7E;
    int i=USB_REPEAT;
    while (!RESULT & ((i--)>0))
        if (USB_SET_FEATURE())
            if (USB_GET_FEATURE())
                if (USB_BUFI[0]==0x7E) { PS=USB_BUFI[1]; RESULT=USB_BUFI[2]==PS; }
                    else RESULT=false;
    if (!RESULT) printf("Error reading PORT\n");
    return RESULT;
}

bool USB_SET_PORT(unsigned char PS)
{
    USB_BUF_CLEAR();
    bool RESULT=false;
    USB_BUFO[0]=0xE7;
    USB_BUFO[1]=PS;
    int i=USB_REPEAT;
    while (!RESULT & ((i--)>0))
        if (USB_SET_FEATURE())
            if (USB_GET_FEATURE())
                 { RESULT=(USB_BUFI[0]==0xE7)&(USB_BUFI[1]==PS)&(USB_BUFI[2]==PS);}
    if (!RESULT) printf("Error writing PORT\n");
    return RESULT;
}

/*
 * calculate CRC for DALLAS sensor
 */
unsigned char CRC8(unsigned char CRC, unsigned char D)
{
    if ( DEBUG ) printf("[D] enter CRC8\n");
    unsigned char R=CRC;
    for (int i=0; i<8; i++)
        if ((R^(D>>i))&0x01==0x01) R=((R^0x18)>>1)|0x80;
            else R=(R>>1)&0x7F;
    return R;
}

/*
 * choose sensor by ROM id, 14ms
 */
bool MATCH_ROM(unsigned long long ROM)
{
    if ( DEBUG ) printf("[D] enter MATCH_ROM\n");
    bool RESULT=false;
    unsigned long long T=ROM;
    if ( DEBUG ) printf("[D] ROM=%llx\n", ROM);
    unsigned char N=3;
    while (!RESULT&((N--)>0))
        if (OW_RESET())
            if (OW_WRITE_BYTE(0x55))
                if (OW_WRITE_4BYTE(T&0xFFFFFFFF))
                    RESULT=OW_WRITE_4BYTE((T>>32)&0xFFFFFFFF);
    if (!RESULT) printf("[!] Error MATCH_ROM\n");
    return RESULT;
}

/*
 * searching for sensors ROM, 1 dev - 410ms, 5 dev - 2.26s, 20 dev - 8.89s
 */
bool SEARCH_ROM(unsigned long long ROM_NEXT, int PL)
{
    if ( DEBUG ) printf("[D] enter SEARCH_ROM\n");
    bool RESULT=false;
    unsigned char N=3;
    unsigned char BIT;
    bool CL[64]; for (int i=0; i<64; i++) CL[i]=false;
    unsigned long long RL[64];
    unsigned long long B1=1, CRC, ROM;
    while (!RESULT&((N--)>0))
        {
        ROM=0;
        if (OW_RESET()) RESULT=OW_WRITE_BYTE(0xF0);
	if ( DEBUG ) printf("[D] SEARCH_ROM: after OW_WRITE_BYTE(0xF0)\n");
        if (RESULT)
	{
            for (int i=0; i<64; i++)
	    {
		if ( DEBUG ) printf("[D] SEARCH_ROM: for(...)\n");
                if (RESULT)
		{
                    if (OW_READ_2BIT(BIT)) 
		    {
			if ( DEBUG ) printf("[D] SEARCH_ROM: if(OW_READ_2BIT(BIT) clause\n");
                        switch (BIT&0x03)
                            {
                            case 0 :
                                {   	//  collision
				if ( DEBUG ) printf("[D] SEARCH_ROM: switch(BIT&0x03) case 0\n");
                                if (PL<i) {CL[i]=true; RL[i]=ROM;}
                                if (PL>=i) BIT=(ROM_NEXT>>i)&0x01; else BIT=0;
                                if(!OW_WRITE_BIT(BIT)) { RESULT=false; i=64; }
                                if (BIT==1) ROM=ROM+(B1<<i);
                                break;
                                }
                            case 1 : {
				if ( DEBUG ) printf("[D] SEARCH_ROM: switch(BIT&0x03) case 1\n");
				if (!OW_WRITE_BIT(0x01)) { RESULT=false; i=64; } else ROM=ROM+(B1<<i); break;
				}
                            case 2 : {
				if ( DEBUG ) printf("[D] SEARCH_ROM: switch(BIT&0x03) case 2\n");
				if (!OW_WRITE_BIT(0x00)) { RESULT=false; i=64; } break;
				}
                            case 3 : {
				if ( DEBUG ) printf("[D] SEARCH_ROM: switch(BIT&0x03) case 3\n");
				RESULT=false; i=64; break;
				}	// not on the line
                            }
                     } else {
			if ( DEBUG ) printf("[D] SEARCH_ROM: if(OW_READ_2BIT(BIT) else clause\n");
			RESULT=false; i=64;
		     }
		}
	    }
	}
        if (ROM==0) RESULT=false;
        if (RESULT) { CRC=0; for (int j=0; j<8; j++) CRC=CRC8(CRC, (ROM>>(j*8))&0xFF); RESULT=CRC==0; }
        }
    if (!RESULT) printf("[!] Error SEARCH_ROM\n");
        else ONEWIRE_ROM[ONEWIRE_COUNT++]=ROM;
    if ( verbose ) printf("[+] found ROM=%llx\n", ONEWIRE_ROM[ONEWIRE_COUNT-1]);
    //	recurrent search call
    for (int i=0; i<64; i++)
        if (CL[i]) SEARCH_ROM(RL[i]|(B1<<i), i);
    return RESULT;
}

/*
 * skip ROM-command, start measuring temperature, 9ms
 */
bool SKIP_ROM_CONVERT()
{
    if ( DEBUG ) printf("[D] enter SKIP_ROM_CONVERT\n");
    bool RESULT=false;
    unsigned char N=3;
    while (!RESULT&((N--)>0))
        if (OW_RESET())
            if (OW_WRITE_BYTE(0xCC))
                RESULT=OW_WRITE_BYTE(0x44);
    if (!RESULT) printf("[!] Error SKIP_ROM_CONVERT\n");
    return RESULT;
}

int read_ports()
{
    int ret=0;
    unsigned char PS;
    if(USB_GET_PORT(PS)) {
        if((PS==8)|(PS==24)) printf("p1=1\n");
        else printf("p1=0\n");
        if(PS>=16) printf("p2=1\n");
        else printf("p2=0\n");
        return 1;
    }
    return 0;
}

int set_port(int num, bool stat)
 {
    unsigned char PS, PS_OLD;
    bool ret;
    if (USB_GET_PORT(PS))
        PS_OLD=PS; 
    else { 
	printf("Error USB_GET_PORT\n"); 
	return 0; 
    }
    if ((num==1)&(stat==1))  { PS=PS|0x08; ret = USB_SET_PORT(PS);}
    else if ((num==1)&(stat==0)) { PS=PS&0x10; ret = USB_SET_PORT(PS);}
    else if ((num==2)&(stat==1))  { PS=PS|0x10; ret = USB_SET_PORT(PS);}
    else if ((num==2)&(stat==0)) { PS=PS&0x08; ret = USB_SET_PORT(PS);}             
    if(!ret) return 0;
    printf("Status port changed\n");

    return 1;
}


/*
 * readin temperature, 28ms
 */
bool GET_TEMPERATURE(unsigned long long ROM, float &T)
{
    if ( DEBUG ) printf("[D] enter GET_TEMPERATURE\n");
    unsigned long long CRC;
    unsigned long L1, L2;
    unsigned char L3;
    unsigned char FAMILY=ROM&0xFF;
    bool RESULT=false;
    unsigned char N=3;
    while (!RESULT&((N--)>0))
        if (MATCH_ROM(ROM))
            if (OW_WRITE_BYTE(0xBE))
                    if (OW_READ_4BYTE(L1))
                        if (OW_READ_4BYTE(L2))
                            if (OW_READ_BYTE(L3))
                            {
				if ( DEBUG ) printf("[D] L1=%lx\tL2=%lx\tL3=%hhx\n", L1, L2, L3);
                                CRC=0;
                                for (int i=0; i<4; i++) CRC=CRC8(CRC, (L1>>(i*8))&0xFF);
				if ( DEBUG ) printf("[D] CRC=%llx\n", CRC);
                                for (int i=0; i<4; i++) CRC=CRC8(CRC, (L2>>(i*8))&0xFF);
				if ( DEBUG ) printf("[D] CRC=%llx\n", CRC);
                                CRC=CRC8(CRC, L3);
				if ( DEBUG ) printf("[D] CRC=%llx\n", CRC);
                                RESULT=CRC==0;
                                short K=L1&0xFFFF;
                                //  DS18B20 +10.125=00A2h, -10.125=FF5Eh
                                //  DS18S20 +25.0=0032h, -25.0=FFCEh
                                //  K=0x0032;
                                T=1000;     //  for unknow FAMILY - no sensor exists
                                if ((FAMILY==0x28)|(FAMILY==0x22)) T=K*0.0625;  //  DS18B20 | DS1822
                                if (FAMILY==0x10) T=K*0.5;                      //  DS18S20 | DS1820
				if ( DEBUG ) printf("[D] ~T=%f\n", T);
                            } //else printf("[!] !OW_READ_BYTE(L3) \n");
			//else printf("[!] !OW_READ_4BYTE(L2) \n");
		    //else printf("[!] !OW_READ_4BYTE(L1) \n");
		//else printf("[!] !OW_WRITE_BYTE(0xBE) \n");
	//else printf("[!] !MATCH_ROM(ROM) \n");
    if (!RESULT) printf("[!] Error GET_TEMPERATURE\n");
    return RESULT;
}

/*
 * showDeviceInfo - show some device info
 * not implemented yet
 */
void showDeviceInfo()
{
	if ( DEBUG ) printf("[D] enter showDeviceInfo\n");
//	printf("[.] Found device: %s - %s\n", manufacturer, product);
	// device group id
	unsigned char FAMILY;
	if (USB_GET_FAMILY(FAMILY)) printf("[.] device family: 0x%02X\n", FAMILY);
	// firmware version
	unsigned int SV;
	if (USB_GET_SOFTV(SV)) printf("[.] firmware version: 0x%02X\n", SV);
	// device id
	unsigned int ID;
	if (USB_GET_ID(ID)) printf("[.] device ID: 0x%08X\n", ID);
}

/*
 * show how to use this program
 */
void showUsage(char *exec)
{
	printf("bmtemp, version %s\n", VERSION);
	printf("USAGE: %s <options>\n", exec);
	printf("\t-i\t\tshow device info\n");
	printf("\t-s\t\tscan for sensors and show their ID ony by one\n");
	printf("\t-tID\tget temperature for sensor with specified ID\n");
	printf("\t-a\t\tscan for sensors and show their data in format id:value\n");
	printf("\t-n\t\tscan for sensors and show their data in format for narodmon.ru\n");
	printf("\t-v\t\tverbose mode on (default is off)\n");
	printf("\t-p\t\tset off/on port status. -p <port> <status> \n\n");
	printf("\t-k\t\get port status\n");
	printf("\nExit codes:\n");
	printf("\t0\tEXIT_OK\n");
	printf("\t1\tEXIT_NO_ARG\n");
	printf("\t2\tEXIT_NO_SENSOR\n");
	printf("\t127\tEXIT_ERROR\n");
}

/*
 * main routine
 */
int main(int argc, char *argv[])
{
        // check for arguments
	if (argc < 2)
	{
		showUsage(argv[0]);
		return EXIT_NO_ARG;
	} else
	{
		while ((argc > 1) && (argv[1][0] == '-'))
		{
			switch (argv[1][1])
			{
			case 'i':
				if (argv[1][2] != 0)
				{
					printf("[!] Wrong Argument: %s\n", argv[1]);
					showUsage(argv[0]);
					return EXIT_NO_ARG;
				}
				if (DEBUG) printf("[D] set showInfo=true\n");
				showInfo=true;
				break;
			case 's':
				if (argv[1][2] != 0)
				{
					printf("[!] Wrong Argument: %s\n", argv[1]);
					showUsage(argv[0]);
					return EXIT_NO_ARG;
				}
				if (DEBUG) printf("[D] set doScan=true\n");
				doScan=true;
				break;
			case 't':
				if (DEBUG) printf("[D] getting ID for getTempOnce\n");
				if (argv[1][2] == 0)
				{
					printf("[!] No ID specified in argument: %s\n", argv[1]);
					showUsage(argv[0]);
					return EXIT_NO_ARG;
				}
				if ( DEBUG ) printf("[D] get temperature for %s\n",&argv[1][2]);
				if (sscanf(&argv[1][2], "%llx", &sensor))
				{
					if ( DEBUG ) printf("[D] parsed argument id = %llx\n", sensor);
					getTempOnce=true;
				} else
				{	// unable to convert id from string to hex
					printf("[!] Unable to convert argument into ID: %s\n", &argv[1][2]);
					showUsage(argv[0]);
					return EXIT_NO_ARG;
				}
				break;
			case 'a':
				if (argv[1][2] != 0)
				{
					printf("[!] Wrong Argument: %s\n", argv[1]);
					showUsage(argv[0]);
					return EXIT_NO_ARG;
				}
				if (DEBUG) printf("[D] set getTempAll=true\n");
				getTempAll=true;
				break;
			case 'v':
				if (argv[1][2] != 0)
				{
					printf("[!] Wrong Argument: %s\n", argv[1]);
					showUsage(argv[0]);
					return EXIT_NO_ARG;
				}
				if (DEBUG) printf("[D] set verbose=true\n");
				verbose=true;
				break;
                        case 'n':
                                if (argv[1][2] != 0)
                                {
                                        printf("[!] Wrong Argument: %s\n", argv[1]);
                                        showUsage(argv[0]);
                                        return EXIT_NO_ARG;
                                }
                                if (DEBUG) printf("[D] use narodmon.ru format for data output\n");
				if (DEBUG) printf("[D] set getTempAllNM=true\n");
                                getTempAllNM=true;
                                break;
			case 'p':
				if (argv[1][2] != 0)
                                {
                                        printf("[!] Wrong Argument: %s\n", argv[1]);				
                                        return EXIT_NO_ARG;
                                }
				pset = true;
				udev = setup();
				set_port(atoi((const char*) argv[2]), (bool) atoi((const char*) argv[3]));
                                statChanged=true;
				break;
			case 'k':
				if (argv[1][2] != 0)
                                {
                                        printf("[!] Wrong Argument: %s\n", argv[1]);				
                                        return EXIT_NO_ARG;
                                }
				pset = true;
				udev = setup();
				read_ports();
                                statChanged=true;
				break;
			default:
				printf("[!] Wrong Argument: %s\n", argv[1]);
				showUsage(argv[0]);
				return EXIT_NO_ARG;
			}
			++argv;
			--argc;
		}
	} // end of arguments parsing

	// decide what to do
	if (getTempAll)
	{
		if ( DEBUG ) printf("[D] call getTempAll\n");
		// call setup
		USB = setup();
		if (!USB) return EXIT_ERROR;
		// SEARCH
		ONEWIRE_COUNT=0;
		if (SEARCH_ROM(0, 0))
		{
			if ( ONEWIRE_COUNT<=0 )
			{
				if ( verbose ) printf("[-] no sensors found, exiting\n");
				device_close(USB);
				return EXIT_NO_SENSOR;
			} else
			{
				float T;
				if (!SKIP_ROM_CONVERT())
				{
					printf("[!] Error SKIP_ROM_CONVERT\n");
					device_close(USB);
					return EXIT_ERROR;
				}
				if ( verbose ) printf("[.] getting temperature...");
				msleep(1000);
				if ( verbose ) printf(" temperature is here :)\n");
				for (int i=0; i<ONEWIRE_COUNT; i++)
				{
					if ( GET_TEMPERATURE(ONEWIRE_ROM[i], T) )
					{
						if ( verbose ) printf("[+] id=%llx T=%f\n", ONEWIRE_ROM[i], T);
						else printf("%llx:%f\n", ONEWIRE_ROM[i], T);
					}
				}
			}
		}
		// release device
		device_close(USB);
	} else if (getTempOnce)
	{
		if ( DEBUG ) printf("[D] getTempOnce\n");
		// call setup
		USB = setup();
		if (!USB) return EXIT_ERROR;
		// init, get temp (match_rom)
		float T;
		ONEWIRE_COUNT = 1;
		ONEWIRE_ROM[0] = sensor;
		// show temp
		if (!SKIP_ROM_CONVERT())
		{
			printf("[!] Error SKIP_ROM_CONVERT\n");
			device_close(USB);
			return EXIT_ERROR;
		} else
		{
			if ( DEBUG ) printf("[D] getting temperature...");
			msleep(1000);
			if ( DEBUG ) printf(" temperature is here :)\n");
			if (OW_RESET())
				if ( GET_TEMPERATURE(ONEWIRE_ROM[0], T) )
				{
					if (verbose) printf("[+] ROM=%llx T=%f\n", sensor, T);
					else printf("%f\n", T);
				} else
				{
					printf("[!] Error getting temperature\n");
					device_close(USB);
					return EXIT_ERROR;
				}
			else
			{
				printf("[!] Error getting temperature\n");
				device_close(USB);
				return EXIT_ERROR;
			}
		}
		// release device
		device_close(USB);
	} else if (doScan)
	{
		if ( DEBUG ) printf("[D] doScan\n");
		// call setup
		USB = setup();
		if (!USB) return EXIT_ERROR;
		// SEARCH
		ONEWIRE_COUNT=0;
		if (SEARCH_ROM(0, 0))
		{	// print ids
			if ( ONEWIRE_COUNT<=0 )
			{
				if ( verbose ) printf("[-] no sensors found, exiting\n");
				device_close(USB);
				return EXIT_NO_SENSOR;
			} else
			{
				if ( verbose ) printf("[.] found %i DALLAS sensor(s):\n", ONEWIRE_COUNT);
				for (int i=0; i<ONEWIRE_COUNT; i++)
				{
					printf("id=%llx\n", ONEWIRE_ROM[i]);
				}
			}
		}
		// release device
		device_close(USB);
	} else if (showInfo)
	{
		if ( DEBUG ) printf("[D] showInfo\n");
		verbose = true;
		// call setup
		USB = setup();
		if (!USB) return EXIT_ERROR;
		// show info
		showDeviceInfo();
		// release device
		device_close(USB);
	} else if (statChanged)
	{
		if ( DEBUG ) printf("[D] Status changed\n");
		verbose = true;
		if (!USB) return EXIT_ERROR;
		// release device
		device_close(USB);
	} else if (getTempAllNM)
	{
		// call setup
		USB = setup();
		if (!USB) return EXIT_ERROR;
		// device id
		unsigned int ID;
	        if (USB_GET_ID(ID)) printf("#1707%08X\n", ID);
		// SEARCH
		ONEWIRE_COUNT=0;
		if (SEARCH_ROM(0, 0))
		{
			if ( ONEWIRE_COUNT<=0 )
			{
				device_close(USB);
				return EXIT_NO_SENSOR;
			} else
			{
				float T;
				if (!SKIP_ROM_CONVERT())
				{
					printf("[!] Error SKIP_ROM_CONVERT\n");
					device_close(USB);
					return EXIT_ERROR;
				}
				msleep(1000);
				for (int i=0; i<ONEWIRE_COUNT; i++)
				{
					if ( GET_TEMPERATURE(ONEWIRE_ROM[i], T) )
					{
						printf("#%llX#%.2f\n", ONEWIRE_ROM[i], T);
					}
				}
			}
		}
		// release device
		device_close(USB);
		printf("##\n");
	} else
	{
		if ( DEBUG ) printf("[D] no actions specified\n");
		showUsage(argv[0]);
		return EXIT_NO_ARG;
	}
	// end of decide what to do
	return EXIT_OK;
}
