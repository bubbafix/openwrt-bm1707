#include <stdio.h>
//#include <string.h>
#include <usb.h>
//#include "libusb.h"
//#include <linux/hiddev.h>

#define VENDOR_ID  0x16c0
#define PRODUCT_ID 0x05df
#define DEBUG	1

static	int			ONEWIRE_COUNT;                      //  количество ROM
static	unsigned long long	ONEWIRE_ROM[128];                   //  номера ROM
//static	float			ONEWIRE_TEMP[128];            //  температура
static	unsigned char		USB_BUFI [9];                       //  буфер приёма
static	unsigned char		USB_BUFO [9];                       //  буфер передачи
static	usb_dev_handle		*USB;                               //  устройство USB
//static	hid_device 		*USB;


usb_dev_handle* find_device()
{
	
	if ( DEBUG ) printf("[D] enter find_device\n");
	struct usb_bus *bus;
	struct usb_device *dev;

	for (bus = usb_get_busses(); bus; bus = bus->next)
	for (dev = bus->devices; dev; dev = dev->next)
	{
		if (dev->descriptor.idVendor == VENDOR_ID &&
		    dev->descriptor.idProduct == PRODUCT_ID)
		{
			usb_dev_handle *udev;
			char s1[256];
			char s2[256];
			
			if (!(udev = usb_open(dev)))
			{
				
				return NULL;
			}
				
			usb_get_string_simple(udev, dev->descriptor.iManufacturer, s1, 256);
			usb_get_string_simple(udev, dev->descriptor.iProduct, s2, 256);

			printf("[+] Device: %s - %s\n", s1, s2);
			
							
			
			return udev;
		}
	}
	

	return NULL;
}

usb_dev_handle* setup()
{
	
	if ( DEBUG ) printf("[D] enter setup\n");
	usb_dev_handle *udev;

	usb_set_debug(0);	
	usb_init();
	usb_find_busses();
	usb_find_devices();
	
	if (!(udev = find_device()))
	{
		printf("[-] Device not found\n");
		
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

void device_close(usb_dev_handle *udev)
{
	
	if ( DEBUG ) printf("[D] enter device_close\n");
	usb_release_interface(udev, 0);
	usb_close(udev);
	
}

int hid_send_feature_report(usb_dev_handle *dev, const unsigned char *data, size_t length)
{
	
	if ( DEBUG ) printf("[D] enter hid_send_feature_report\n");

	if ( DEBUG ) printf("[D] hid_send_feature_report: data=");
	if ( DEBUG ) for (int k=0; k<length; k++) printf("0x%02X ", data[k]);
	if ( DEBUG ) printf("\n");

	int res = -1;

//	res = usb_control_msg(dev, data[0], data[1], data[2], 0x01, (char *) data, length, 5000);
	res = usb_control_msg(dev, 0x21, 0x01, 0x0300, 0x00, (char *) data, length, 5000);

	if ( DEBUG ) printf("[D] hid_send_feature_report: res=0x%02X\n", res);

	if ( DEBUG ) printf("[D] hid_send_feature_report: after call data=");
	if ( DEBUG ) for (int k=0; k<length; k++) printf("0x%02X ", data[k]);
	if ( DEBUG ) printf("\n");

//	res = libusb_control_transfer(NULL/*was: dev->device_handle */,
//		LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_OUT,
//		0x09/*HID set_report*/,
//		(3/*HID feature*/ << 8) | report_number,
//		0 /* was: dev->interface */,
//		(unsigned char *)data, length,
//		1000/*timeout millis*/);
/*	
	if (res < 0)
		return -1;
*/	
	/* Account for the report ID */
/*	if (skipped_report_id)
		length++;
	if ( DEBUG ) printf("hid_send_feature_report: length=0x%02X\n", length);
*/
//	return length;
	
	return res;
}

int hid_get_feature_report(usb_dev_handle *dev, unsigned char *data, size_t length)
{
	
	if ( DEBUG ) printf("[D] enter hid_get_feature_report\n");

	if ( DEBUG ) printf("[D] hid_get_feature_report: data=");
	if ( DEBUG ) for (int k=0; k<length; k++) printf("0x%02X ", data[k]);
	if ( DEBUG ) printf("\n");

	int res = -1;

	/*
		LIBUSB_REQUEST_TYPE_CLASS = (0x01 << 5)
		LIBUSB_RECIPIENT_INTERFACE = 0x01
		LIBUSB_ENDPOINT_IN = 0x80
		LIBUSB_ENDPOINT_OUT = 0x00
	in: 0xA1
	out 0x21
	*/
//usb_control_msg(usb_dev_handle*, int, int, int, int, char*, int, int)'
//        res = usb_control_msg(dev, data[0], data[1], 0x00, 0x00, (char *) data, length, 5000);
	res = usb_control_msg(dev, 0xA1, 0x01, 0x0300, 0x00, (char *) data, length, 5000);

//	res = usb_control_msg(dev, 0x21, 0x09, 0x0200, 0x01, (char *) data, length, 5000);
//	res = usb_control_msg(dev, 0x21, 0x01, (3 << 8) | report_number, 0, (char *)data, length, 5000);
	if ( DEBUG ) printf("[D] hid_get_feature_report: res=0x%02X\n", res);	

	if ( DEBUG ) printf("[D] hid_get_feature_report: after call data=");
	if ( DEBUG ) for (int k=0; k<length; k++) printf("0x%02X ", data[k]);
	if ( DEBUG ) printf("\n");

//	res = libusb_control_transfer(NULL/* was: dev->device_handle */,
//		LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE|LIBUSB_ENDPOINT_IN,
//		0x01/*HID get_report*/,
//		(3/*HID feature*/ << 8) | report_number,
//		0/* was: dev->interface */,
//		(unsigned char *)data, length,
//		1000/*timeout millis*/);

/*
	// definition: 
	extern int usb_control_msg(	struct usb_device *dev, 
					unsigned int pipe,
					__u8 request, 
					__u8 requesttype, 
					__u16 value, 
					__u16 index,
					void *data, 
					__u16 size, 
					int timeout);
        */
	
/*
	if (res < 0)
		return -1;
*/
/*
	if (skipped_report_id)
		res++;
*/
	
	return res;
}

void USB_BUF_CLEAR()
{   //  очистка буферов приёма и передачи
    
    if ( DEBUG ) printf("[D] enter USB_BUF_CLEAR\n");
    
    for (int i=0; i<9; i++) { USB_BUFI[i]=0; USB_BUFO[i]=0; }
    
    
}

bool USB_GET_FEATURE()
{   //  чтение в буфер из устройства
    
    if ( DEBUG ) printf("[D] enter USB_GET_FEATURE\n");
    bool RESULT=false;
    int i=3;   //  число попыток
    int j=0;

    unsigned char buf[8];

    while (!RESULT & ((i--)>0)) // why 3 times ?
    {
        for (j=0; j<8; j++) { buf[j]=USB_BUFI[j+1]; }
        try { RESULT=hid_get_feature_report(USB, buf, 8);}
//        try { RESULT=hid_get_feature_report(USB, USB_BUFI, 9);}
             catch (...) { RESULT=false; };
    }
    for (j=0; j<8; j++) { USB_BUFI[j+1]=buf[j]; }

    if (!RESULT) printf("[!] USB_GET_FEATURE: Error reading from USB-device\n");

    
    return RESULT;
}

bool USB_SET_FEATURE()
{   //  запись из буфера в устройство
    
    if ( DEBUG ) printf("[D] enter USB_SET_FEATURE\n");
    bool RESULT=false;

    int j=0;
    unsigned char buf[8];
    for (j=0; j<8; j++) { buf[j]=USB_BUFO[j+1]; }

//    try { RESULT=hid_send_feature_report(USB, USB_BUFO, 9);}
    try { RESULT=hid_send_feature_report(USB, buf, 8);}
        catch (...) { RESULT=false; };

    for (j=0; j<8; j++) { USB_BUFO[j+1]=buf[j]; }

    if (!RESULT) printf("[!] USB_SET_FEATURE: Error writing to USB-device\n");

    
    return RESULT;
}

bool OW_RESET()
{   //  RESET, ~3ms
    
    if ( DEBUG ) printf("[D] enter OW_RESET\n");
    bool RESULT=false;
    USB_BUF_CLEAR();
    USB_BUFO[1]=0x18;    USB_BUFO[2]=0x48;
    unsigned char N=3;
    while (!RESULT &((N--)>0))
        if (USB_SET_FEATURE())
        {
	    if ( DEBUG ) printf("[D] OW_RESET: if USB_SET_FEATURE() true...\n");
            sleep(2);
            if (USB_GET_FEATURE()) 
	    {
		if ( DEBUG ) printf("[D] OW_RESET: if USB_GET_FEATURE() true...\n");
		RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x48)&(USB_BUFI[3]==0x00);
	    } else RESULT=false;
        }
    if (!RESULT) printf("[!] Error OW_RESET\n");

    
    return RESULT;
}

bool OW_READ_2BIT(unsigned char &B)
{   //  чтение 2-x бит, 3ms
    
    if ( DEBUG ) printf("[D] enter OW_READ_2BIT\n");
    bool RESULT=false;
    USB_BUF_CLEAR();
    USB_BUFO[1]=0x18;    USB_BUFO[2]=0x82;
    USB_BUFO[3]=0x01;    USB_BUFO[4]=0x01;
    if (USB_SET_FEATURE())
        {
        sleep(1);
        if (USB_GET_FEATURE())
            { RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x82); B=(USB_BUFI[3]&0x01)+((USB_BUFI[4]<<1)&0x02); }
        }
    if (!RESULT) printf("[!] Error OW_READ_2BIT\n");

    
    return RESULT;
}

bool OW_READ_BYTE(unsigned char &B)
{   //  чтение байта, 3ms
    
    if ( DEBUG ) printf("[D] enter OW_READ_BYTE\n");
    bool RESULT=false;
    USB_BUF_CLEAR();
    USB_BUFO[1]=0x18;    USB_BUFO[2]=0x88;    USB_BUFO[3]=0xFF;
    if (USB_SET_FEATURE())
        {
        sleep(1);
        if (USB_GET_FEATURE())
            { RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x88); B=USB_BUFI[3]; }
        }
    if (!RESULT) printf("[!] Error OW_READ_BYTE\n");

    
    return RESULT;
}

bool OW_READ_4BYTE(unsigned long &B)
{   //  чтение 4 байта, 4ms
    
    if ( DEBUG ) printf("[D] enter OW_READ_4BYTE\n");
    bool RESULT=false;
    USB_BUF_CLEAR();
    USB_BUFO[1]=0x18;    USB_BUFO[2]=0x84;    USB_BUFO[3]=0xFF;
    USB_BUFO[4]=0xFF;    USB_BUFO[5]=0xFF;    USB_BUFO[6]=0xFF;
    if (USB_SET_FEATURE())
        {
        sleep(2);
        if (USB_GET_FEATURE())
            { RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x84); B=USB_BUFI[3]+(USB_BUFI[4]<<8)+(USB_BUFI[5]<<16)+(USB_BUFI[6]<<24); }
        }
    if (!RESULT) printf("[!] Error OW_READ_4BYTE\n");

    
    return RESULT;
}

bool OW_WRITE_BIT(unsigned char B)
{   //  запись бита, 3ms
    
    if ( DEBUG ) printf("[D] enter OW_WRITE_BIT\n");
    bool RESULT=false;
    USB_BUF_CLEAR();
    USB_BUFO[1]=0x18;    USB_BUFO[2]=0x81;    USB_BUFO[3]=B&0x01;
    if (USB_SET_FEATURE())
        {
        sleep(1);
        if (USB_GET_FEATURE())
            RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x81)&((USB_BUFI[3]&0x01)==(B&0x01));
        }
    if (!RESULT) printf("[!] Error OW_WRITE_BIT\n");

    
    return RESULT;
}

bool OW_WRITE_BYTE(unsigned char B)
{   //  запись байта, 3ms
    
    if ( DEBUG ) printf("[D] enter OW_WRITE_BYTE\n");
    bool RESULT=false;
    USB_BUF_CLEAR();
    USB_BUFO[1]=0x18;    USB_BUFO[2]=0x88;    USB_BUFO[3]=B;
    if (USB_SET_FEATURE())
        {
        sleep(1);
        if (USB_GET_FEATURE())
            RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x88)&(USB_BUFI[3]==B);
        }
    if (!RESULT) printf("[!] Error OW_WRITE_BYTE\n");

    
    return RESULT;
}

bool OW_WRITE_4BYTE(unsigned long B)
{   //  запись 4 байта, 4ms
    
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
        sleep(2);
        if (USB_GET_FEATURE())
            RESULT=(USB_BUFI[1]==0x18)&(USB_BUFI[2]==0x84)&(USB_BUFI[3]==D0&(USB_BUFI[4]==D1)&(USB_BUFI[5]==D2)&(USB_BUFI[6]==D3));
        }
    if (!RESULT) printf("[!] Error OW_WRITE_4BYTE\n");

    
    return RESULT;
}

unsigned char CRC8(unsigned char CRC, unsigned char D)
{   //  подчсёт CRC для DALLAS
    
    if ( DEBUG ) printf("[D] enter CRC8\n");
    unsigned char R=CRC;
    for (int i=0; i<8; i++)
        if ((R^(D>>i))&0x01==0x01) R=((R^0x18)>>1)|0x80;
            else R=(R>>1)&0x7F;

    
    return R;
}

bool MATCH_ROM(unsigned long long ROM)
{   //  выбор прибора по ROM, 14ms
    
    if ( DEBUG ) printf("[D] enter MATCH_ROM\n");
    bool RESULT=false;
    unsigned long long T=ROM;
    unsigned char N=3;
    while (!RESULT&((N--)>0))
        if (OW_RESET())
            if (OW_WRITE_BYTE(0x55))
                if (OW_WRITE_4BYTE(T&0xFFFFFFFF))
                    RESULT=OW_WRITE_4BYTE((T>>32)&0xFFFFFFFF);
    if (!RESULT) printf("[!] Error MATCH_ROM\n");

    
    return RESULT;
}

bool SEARCH_ROM(unsigned long long ROM_NEXT, int PL)
{   //  поиск ROM, 1 dev - 410ms, 5 dev - 2.26s, 20 dev - 8.89s
    
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
                                {   //  коллизия есть
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
				}   //  нет на линии
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

    printf("[+] found ROM=%x\n", ONEWIRE_ROM[ONEWIRE_COUNT-1]);

    //  рекурентный вызов поиска
    for (int i=0; i<64; i++)
        if (CL[i]) SEARCH_ROM(RL[i]|(B1<<i), i);

    
    return RESULT;
}

bool SKIP_ROM_CONVERT()
{   //  пропуск ROM-команд, старт измерения температуры, 9ms
    
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

bool GET_TEMPERATURE(unsigned long long ROM, float &T)
{   //  чтение температуры, 28ms
    
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
                                CRC=0;
                                for (int i=0; i<4; i++) CRC=CRC8(CRC, (L1>>(i*8))&0xFF);
                                for (int i=0; i<4; i++) CRC=CRC8(CRC, (L2>>(i*8))&0xFF);
                                CRC=CRC8(CRC, L3);
                                RESULT=CRC==0;
                                short K=L1&0xFFFF;
                                //  DS18B20 +10.125=00A2h, -10.125=FF5Eh
                                //  DS18S20 +25.0=0032h, -25.0=FFCEh
                                //  K=0x0032;
                                T=1000;     //  для неопознанной FAMILY датчик отсутствует
                                if ((FAMILY==0x28)|(FAMILY==0x22)) T=K*0.0625;  //  DS18B20 | DS1822
                                if (FAMILY==0x10) T=K*0.5;                      //  DS18S20 | DS1820
                                }
    if (!RESULT) printf("[!] Error GET_TEMPERATURE\n");

    
    return RESULT;
}

int main(int argc, char *argv[])
{
/*
	int RESULT = 1;
	USB = hid_open( VENDOR_ID, PRODUCT_ID, NULL );
	if (!USB) 
	{
		printf("unable to open device\n");
		RESULT = -1;
	} else 
	{
		printf("device opened\n");
		RESULT = 0;
  	}
*/
	usb_dev_handle *udev = setup();
	if (!udev)
		return 1;
	USB = udev;
	// find sensors	
    	ONEWIRE_COUNT=0;
    	if (SEARCH_ROM(0, 0)) printf("[.] Найдено DALLAS - %i\n", ONEWIRE_COUNT);

	float T;
        //dbg:
	//GET_TEMPERATURE(0x12345678, T);

	if ( ONEWIRE_COUNT<=0 ) { printf("[.] no sensors found, exiting\n"); return 1; }
	if (!SKIP_ROM_CONVERT()) { printf("[!] Error SKIP_ROM_CONVERT\n"); return 1; }
	printf("[.] getting temperature...");
	sleep(10);
	printf(" temperature is here :)\n");
	for (int i=0; i<ONEWIRE_COUNT; i++)
	{
		if (GET_TEMPERATURE(ONEWIRE_ROM[i], T)) 
		{
			printf("[+] ROM=%x T=%f\n", (int)ONEWIRE_ROM[i], T);
		}
	}
	
	device_close(udev);
	return 0;
}
