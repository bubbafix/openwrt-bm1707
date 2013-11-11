# Once upon a time...
There was a DALLAS 1-wire sensor.
Many devices used to deal with it.
So **BM1707** also did.
This is small USB device that has one sensor on board and ability to connect much more by wire.
Also it has ability to control external relay by temperature changes.

More information on device available at http://www.masterkit.ru/main/set.php?code_id=565375

**So what is it all about**

This repo contains files to use with OpenWRT SDK.
Current version uses libstdc++ and libusb.

**Step by step guide**

1. Run a linux machine.<br>
Let's say it would be Ubuntu on virtualbox. Make sure you have enough disk space on virtual machine to store SDK.
2. Download proper SDK from OpenWRT.org (release, hardware). If you're not sure what release/hw to use check supported devices list at http://wiki.openwrt.org/toh/start. Follwing example will download SDK for `attitude_adjustment` realease 12.09 `ar71xx` platform.
> `wget http://downloads.openwrt.org/attitude_adjustment/12.09/ar71xx/generic/OpenWrt-SDK-ar71xx-for-linux-i486-gcc-4.6-linaro_uClibc-0.9.33.2.tar.bz2`
3. Decompress SDK.

> `tar xjvf OpenWrt-SDK-ar71xx-for-linux-i486-gcc-4.6-linaro_uClibc-0.9.33.2.tar.bz2`
4. Download `bmtemp` package contents

> wget https://github.com/bubbafix/openwrt-bm1707/archive/master.zip
5. Decompress archive

		unzip ./master.zip
6. Copy package contents into proper folder.
> cp ./openwrt-bm1707-master/package/* ./OpenWrt-SDK-ar71xx-for-linux-i486-gcc-4.6-linaro_uClibc-0.9.33.2/package/
7. Go into SDK folder.
> cd /OpenWrt-SDK-ar71xx-for-linux-i486-gcc-4.6-linaro_uClibc-0.9.33.2/
8. Compile package
> make
9. If everything went well find these files: `bmtemp_2_ar71xx.ipk  Packages  Packages.gz` at
> ls ./OpenWrt-SDK-ar71xx-for-linux-i486-gcc-4.6-linaro_uClibc-0.9.33.2/bin/ar71xx/packages/
10. File `.ipk` is ready to be installed with `opkg install ...` command.

## Thanks to...

* Maxim Integrated http://www.maximintegrated.com/ for beautiful solution.
* People from www.masterkit.ru for BM1707 device.
* Serg @ http://usbsergdev.narod.ru for giving an example of code to work with device.
* All contributors for OpenWrt https://openwrt.org
* People who tried to use this package for their feedback
