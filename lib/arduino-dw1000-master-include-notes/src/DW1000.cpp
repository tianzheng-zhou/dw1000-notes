/*
 * DW1000.cpp
 *
 *  尝试汉化
 *      Author: zhou tianzheng
 */

#include "DW1000.h"
DW1000Class DW1000;

/* ###########################################################################
 * #### Static member variables ##############################################
 * ######################################################################### */
// pins
uint8_t DW1000Class::_ss;
uint8_t DW1000Class::_rst; //rst pin
uint8_t DW1000Class::_irq;


// IRQ callbacks
void (* DW1000Class::_handleSent)(void)                      = 0;
void (* DW1000Class::_handleError)(void)                     = 0;
void (* DW1000Class::_handleReceived)(void)                  = 0;
void (* DW1000Class::_handleReceiveFailed)(void)             = 0;
void (* DW1000Class::_handleReceiveTimeout)(void)            = 0;
void (* DW1000Class::_handleReceiveTimestampAvailable)(void) = 0;

// registers
byte       DW1000Class::_syscfg[LEN_SYS_CFG];  //register 0x04
byte       DW1000Class::_sysctrl[LEN_SYS_CTRL]; //register 0x0D
byte       DW1000Class::_sysstatus[LEN_SYS_STATUS]; //0x0F
byte       DW1000Class::_txfctrl[LEN_TX_FCTRL]; //register 0x08
byte       DW1000Class::_sysmask[LEN_SYS_MASK];
byte       DW1000Class::_chanctrl[LEN_CHAN_CTRL]; //register 0x1F
byte       DW1000Class::_networkAndAddress[LEN_PANADR];  //register 0x03

// monitoring
byte DW1000Class::_vmeas3v3 = 0;
byte DW1000Class::_tmeas23C = 0;

// driver internal state
byte       DW1000Class::_extendedFrameLength = FRAME_LENGTH_NORMAL;
byte       DW1000Class::_pacSize             = PAC_SIZE_8;
byte       DW1000Class::_pulseFrequency      = TX_PULSE_FREQ_16MHZ;
byte       DW1000Class::_dataRate            = TRX_RATE_6800KBPS;
byte       DW1000Class::_preambleLength      = TX_PREAMBLE_LEN_128;
byte       DW1000Class::_preambleCode        = PREAMBLE_CODE_16MHZ_4;
byte       DW1000Class::_channel             = CHANNEL_5;
DW1000Time DW1000Class::_antennaDelay;
boolean	   DW1000Class::_antennaCalibrated	 = false;
boolean    DW1000Class::_smartPower          = false;

boolean    DW1000Class::_frameCheck          = true;
boolean    DW1000Class::_permanentReceive    = false;
uint8_t    DW1000Class::_deviceMode          = IDLE_MODE; // TODO replace by enum

boolean    DW1000Class::_debounceClockEnabled = false;

// modes of operation
// TODO use enum external, not config array
// this declaration is needed to make variables accessible while runtime from external code
constexpr byte DW1000Class::MODE_LONGDATA_RANGE_LOWPOWER[];
constexpr byte DW1000Class::MODE_SHORTDATA_FAST_LOWPOWER[];
constexpr byte DW1000Class::MODE_LONGDATA_FAST_LOWPOWER[];
constexpr byte DW1000Class::MODE_SHORTDATA_FAST_ACCURACY[];
constexpr byte DW1000Class::MODE_LONGDATA_FAST_ACCURACY[];
constexpr byte DW1000Class::MODE_LONGDATA_RANGE_ACCURACY[];
/*
const byte DW1000Class::MODE_LONGDATA_RANGE_LOWPOWER[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_2048};
const byte DW1000Class::MODE_SHORTDATA_FAST_LOWPOWER[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_128};
const byte DW1000Class::MODE_LONGDATA_FAST_LOWPOWER[]  = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_1024};
const byte DW1000Class::MODE_SHORTDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_128};
const byte DW1000Class::MODE_LONGDATA_FAST_ACCURACY[]  = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_1024};
const byte DW1000Class::MODE_LONGDATA_RANGE_ACCURACY[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_2048};
*/
// range bias tables (500 MHz in [mm] and 900 MHz in [2mm] - to fit into bytes)
constexpr byte DW1000Class::BIAS_500_16[];
constexpr byte DW1000Class::BIAS_500_64[];
constexpr byte DW1000Class::BIAS_900_16[];
constexpr byte DW1000Class::BIAS_900_64[];
/*
const byte DW1000Class::BIAS_500_16[] = {198, 187, 179, 163, 143, 127, 109, 84, 59, 31, 0, 36, 65, 84, 97, 106, 110, 112};
const byte DW1000Class::BIAS_500_64[] = {110, 105, 100, 93, 82, 69, 51, 27, 0, 21, 35, 42, 49, 62, 71, 76, 81, 86};
const byte DW1000Class::BIAS_900_16[] = {137, 122, 105, 88, 69, 47, 25, 0, 21, 48, 79, 105, 127, 147, 160, 169, 178, 197};
const byte DW1000Class::BIAS_900_64[] = {147, 133, 117, 99, 75, 50, 29, 0, 24, 45, 63, 76, 87, 98, 116, 122, 132, 142};
*/
// SPI settings
#ifdef ESP8266
	// default ESP8266 frequency is 80 Mhz, thus divide by 4 is 20 MHz
	const SPISettings DW1000Class::_fastSPI = SPISettings(20000000L, MSBFIRST, SPI_MODE0);
#else
	const SPISettings DW1000Class::_fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);
#endif
const SPISettings DW1000Class::_slowSPI = SPISettings(2000000L, MSBFIRST, SPI_MODE0);
const SPISettings* DW1000Class::_currentSPI = &_fastSPI;

/* ###########################################################################
 * #### Init and end #######################################################
 * ######################################################################### */

void DW1000Class::end() {
	SPI.end();
}
/**
 * @brief 初始化 DW1000 芯片，并设置一些默认的配置
 * 
 * @param ss SPI 从机片选引脚
 */
void DW1000Class::select(uint8_t ss) {
	reselect(ss);
	// try locking clock at PLL speed (should be done already,
	// but just to be sure)
	enableClock(AUTO_CLOCK);
	delay(5);
	// reset chip (either soft or hard)
	if(_rst != 0xff) {
		// dw1000 data sheet v2.08 §5.6.1 page 20, the RSTn pin should not be driven high but left floating.
		pinMode(_rst, INPUT);
	}
	reset();

	// default network and node id 地址0x03
	// 设置默认的网络和节点 ID
	writeValueToBytes(_networkAndAddress, 0xFF, LEN_PANADR);

	//将networkAndAddress写入0x03寄存器
	writeNetworkIdAndDeviceAddress();
	
	// default system configuration
	//将_syscfg (0x04) 寄存器清零
	memset(_syscfg, 0, LEN_SYS_CFG);

	setDoubleBuffering(false);  // disable 双缓冲（也就是禁用接收缓存，详见数据手册） bit-12
	setInterruptPolarity(true);	// bit-9 设置irq输出极性 也就是设置irq输出高电平或者低电平
	writeSystemConfigurationRegister();	//将_syscfg 写入0x04寄存器
	
	// default interrupt mask, i.e. no interrupts
	clearInterrupts(); //sysmask
	/*寄存器映射寄存器文件 0x0E 是系统事件掩码寄存器。 这些与事件状态一致
	SYS_STATUS 寄存器中的位。 每当 SYS_MASK 中的某个位被置位（到 1），并且相应的被
	置位 SYS_STATUS 寄存器也被置位，然后产生一个中断来产生硬件 IRQ 输出
	线。 详见数据手册第七章
	*/
	writeSystemEventMaskRegister(); //将_sysmask 写入0x0E寄存器
	
	// load LDE micro-code
	enableClock(XTI_CLOCK); //fotce the system to the 19.2MHz XTL clock
	delay(5);

	//TODO 细节没有看，好像是管理LDE模块（用于计算信号传递延迟）
	manageLDE();

	delay(5);
	enableClock(AUTO_CLOCK);
	delay(5);
	
	// read the temp and vbat readings from OTP that were recorded during production test
	// see 6.3.1 OTP memory map
	byte buf_otp[4];
	readBytesOTP(0x008, buf_otp); // the stored 3.3 V reading
	_vmeas3v3 = buf_otp[0];
	readBytesOTP(0x009, buf_otp); // the stored 23C reading
	_tmeas23C = buf_otp[0];
}

void DW1000Class::reselect(uint8_t ss) {
	_ss = ss;
	pinMode(_ss, OUTPUT);
	digitalWrite(_ss, HIGH);
}

void DW1000Class::begin(uint8_t irq, uint8_t rst) {
	// generous initial init/wake-up-idle delay
	delay(5);
	// Configure the IRQ pin as INPUT. Required for correct interrupt setting for ESP8266
    	pinMode(irq, INPUT);
	// start SPI
	SPI.begin();
#ifndef ESP8266
	// SPI.usingInterrupt(digitalPinToInterrupt(irq)); // not every board support this, e.g. ESP8266
#endif
	// pin and basic member setup
	_rst        = rst;
	_irq        = irq;
	_deviceMode = IDLE_MODE;
	// attach interrupt
	//attachInterrupt(_irq, DW1000Class::handleInterrupt, CHANGE); // todo interrupt for ESP8266
	// TODO throw error if pin is not a interrupt pin
	attachInterrupt(digitalPinToInterrupt(_irq), DW1000Class::handleInterrupt, RISING); // todo interrupt for ESP8266
}

/*
LDE 是指 "Linear Delay Estimator"，即线性延迟估计器。
在DW1000芯片中，LDE是一个用于计算信号传播延迟的硬件模块。
它通过测量信号在发送和接收之间的时间差来估计距离，
这是基于超宽带（UWB）技术的定位系统中的关键功能之一。
*/
void DW1000Class::manageLDE() {
	// transfer any ldo tune values
	byte ldoTune[LEN_OTP_RDAT];
	readBytesOTP(0x04, ldoTune); // TODO #define
	if(ldoTune[0] != 0) {
		// TODO tuning available, copy over to RAM: use OTP_LDO bit
	}
	// tell the chip to load the LDE microcode
	// TODO remove clock-related code (PMSC_CTRL) as handled separately
	byte pmscctrl0[LEN_PMSC_CTRL0];
	byte otpctrl[LEN_OTP_CTRL];
	memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
	memset(otpctrl, 0, LEN_OTP_CTRL);
	readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
	readBytes(OTP_IF, OTP_CTRL_SUB, otpctrl, LEN_OTP_CTRL);
	pmscctrl0[0] = 0x01;
	pmscctrl0[1] = 0x03;
	otpctrl[0]   = 0x00;
	otpctrl[1]   = 0x80;
	writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);
	writeBytes(OTP_IF, OTP_CTRL_SUB, otpctrl, 2);
	delay(5);
	pmscctrl0[0] = 0x00;
	pmscctrl0[1] &= 0x02;
	writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);
}

/**
 * @brief 启用DW1000芯片的时钟
 * 设置时钟的寄存器为0x36
 * 
 * @param clock 指定要启用的时钟类型，可以是AUTO_CLOCK、XTI_CLOCK或PLL_CLOCK
 * 
 * AUTO - The system clock will run off the 19.2 MHz XTI clock until the PLL is calibrated
 * and locked, then it will switch over the 125 MHz PLL clock.
 * 
 * XTI - Force system clock to be the 19.2 MHz XTI clock.
 * 
 * PLL - Force system clock to the 125 MHz PLL clock. (If this clock is not present the DW1000
 * will essentially lock up with further SPI communications impossible. In this case an
 * external reset will be needed to recover)
 * （摘自用户手册）
 */
void DW1000Class::enableClock(byte clock) {
	byte pmscctrl0[LEN_PMSC_CTRL0];
	memset(pmscctrl0, 0, LEN_PMSC_CTRL0);//将pmscctrl0清零
	readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0); //读取0x36中的0x00子地址的值并传入pmscctrl0
	//用于修改区域SYSCLKS
	if(clock == AUTO_CLOCK) {
		_currentSPI = &_fastSPI;
		pmscctrl0[0] = AUTO_CLOCK;
		pmscctrl0[1] &= 0xFE;
	} else if(clock == XTI_CLOCK) {
		_currentSPI = &_slowSPI;
		pmscctrl0[0] &= 0xFC;
		pmscctrl0[0] |= XTI_CLOCK;
	} else if(clock == PLL_CLOCK) {
		_currentSPI = &_fastSPI;
		pmscctrl0[0] &= 0xFC;
		pmscctrl0[0] |= PLL_CLOCK;
	} else {
		// TODO deliver proper warning
	}
	writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);//将pmscctrl0写入0x36中的0x00的子地址
}

void DW1000Class::enableDebounceClock() {
	//初始化这个数组
	byte pmscctrl0[LEN_PMSC_CTRL0];
	memset(pmscctrl0, 0, LEN_PMSC_CTRL0);

	readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);

	//GPIO De-bounce Clock Enable.
	/*
	GPIO De-bounce Clock 是一种用于去抖动（debounce）GPIO（通用输入输出）引脚的时钟信号。
	在电子电路中，当机械开关或按钮连接到GPIO引脚时，由于机械触点的弹跳（bounce）现象，
	可能会导致引脚电平在短时间内多次变化，从而产生错误的触发信号。
	为了解决这个问题，通常会使用去抖动技术。

	去抖动时钟的作用是在检测到GPIO引脚电平变化后，延迟一段时间再进行采样，以确保电平已经稳定。
	这个延迟时间通常由去抖动时钟的频率决定。
	通过这种方式，可以过滤掉由于触点弹跳引起的虚假电平变化，从而得到稳定可靠的输入信号。
	*/
	//这个时钟时钟是一个kilohertz时钟 似乎也和LED有关系
	setBit(pmscctrl0, LEN_PMSC_CTRL0, GPDCE_BIT, 1);

	/*
	Kilohertz clock Enable.  When this bit is set to 1 it enables the divider. 
	The divider value is set by KHZCLKDIV in Sub-Register 0x36:04 – PMSC_CTRL1. 
	启用kilohertz clock和分频器 分频系数由其他寄存器的值设置
	*/
	setBit(pmscctrl0, LEN_PMSC_CTRL0, KHZCLKEN_BIT, 1);

	writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
        _debounceClockEnabled = true;
}

//既然是LED那就不管了
void DW1000Class::enableLedBlinking() {
	byte pmscledc[LEN_PMSC_LEDC];
	memset(pmscledc, 0, LEN_PMSC_LEDC);
	readBytes(PMSC, PMSC_LEDC_SUB, pmscledc, LEN_PMSC_LEDC);
	setBit(pmscledc, LEN_PMSC_LEDC, BLNKEN, 1);
	writeBytes(PMSC, PMSC_LEDC_SUB, pmscledc, LEN_PMSC_LEDC);
}

//没什么好说的 就是用来改GPIO的功能 详见用户手册
void DW1000Class::setGPIOMode(uint8_t msgp, uint8_t mode) {
	byte gpiomode[LEN_GPIO_MODE];
	memset(gpiomode, 0, LEN_GPIO_MODE);

	readBytes(GPIO_CTRL, GPIO_MODE_SUB, gpiomode, LEN_GPIO_MODE);
	for (char i = 0; i < 2; i++){
		setBit(gpiomode, LEN_GPIO_MODE, msgp + i, (mode >> i) & 1);
	}
	writeBytes(GPIO_CTRL, GPIO_MODE_SUB, gpiomode, LEN_GPIO_MODE);
}

// 暂时用不到
void DW1000Class::deepSleep() {
	byte aon_wcfg[LEN_AON_WCFG];
	memset(aon_wcfg, 0, LEN_AON_WCFG);
	readBytes(AON, AON_WCFG_SUB, aon_wcfg, LEN_AON_WCFG);
	setBit(aon_wcfg, LEN_AON_WCFG, ONW_LDC_BIT, true);
	setBit(aon_wcfg, LEN_AON_WCFG, ONW_LDD0_BIT, true);
	writeBytes(AON, AON_WCFG_SUB, aon_wcfg, LEN_AON_WCFG);

	byte pmsc_ctrl1[LEN_PMSC_CTRL1];
	memset(pmsc_ctrl1, 0, LEN_PMSC_CTRL1);
	readBytes(PMSC, PMSC_CTRL1_SUB, pmsc_ctrl1, LEN_PMSC_CTRL1);
	setBit(pmsc_ctrl1, LEN_PMSC_CTRL1, ATXSLP_BIT, false);
	setBit(pmsc_ctrl1, LEN_PMSC_CTRL1, ARXSLP_BIT, false);
	writeBytes(PMSC, PMSC_CTRL1_SUB, pmsc_ctrl1, LEN_PMSC_CTRL1);

	byte aon_cfg0[LEN_AON_CFG0];
	memset(aon_cfg0, 0, LEN_AON_CFG0);
	readBytes(AON, AON_CFG0_SUB, aon_cfg0, LEN_AON_CFG0);
	setBit(aon_cfg0, LEN_AON_CFG0, WAKE_SPI_BIT, true);
	setBit(aon_cfg0, LEN_AON_CFG0, WAKE_PIN_BIT, true);
	setBit(aon_cfg0, LEN_AON_CFG0, WAKE_CNT_BIT, false);
	setBit(aon_cfg0, LEN_AON_CFG0, SLEEP_EN_BIT, true);
	writeBytes(AON, AON_CFG0_SUB, aon_cfg0, LEN_AON_CFG0);

	byte aon_ctrl[LEN_AON_CTRL];
	memset(aon_ctrl, 0, LEN_AON_CTRL);
	readBytes(AON, AON_CTRL_SUB, aon_ctrl, LEN_AON_CTRL);
	setBit(aon_ctrl, LEN_AON_CTRL, UPL_CFG_BIT, true);
	setBit(aon_ctrl, LEN_AON_CTRL, SAVE_BIT, true);
	writeBytes(AON, AON_CTRL_SUB, aon_ctrl, LEN_AON_CTRL);
}

void DW1000Class::spiWakeup(){
        digitalWrite(_ss, LOW);
        delay(2);
        digitalWrite(_ss, HIGH);
        if (_debounceClockEnabled){
                DW1000Class::enableDebounceClock();
        }
}

//字面意思
void DW1000Class::reset() {
	if(_rst == 0xff) {
		softReset();
	} else {
		// dw1000 data sheet v2.08 §5.6.1 page 20, the RSTn pin should not be driven high but left floating.
		pinMode(_rst, OUTPUT);
		digitalWrite(_rst, LOW);
		delay(2);  // dw1000 data sheet v2.08 §5.6.1 page 20: nominal 50ns, to be safe take more time
		pinMode(_rst, INPUT);
		delay(10); // dwm1000 data sheet v1.2 page 5: nominal 3 ms, to be safe take more time
		// force into idle mode (although it should be already after reset)
		idle();
	}
}

void DW1000Class::softReset() {

	/*
	These bits should be cleared to zero to force a reset 
	and then returned to one for normal operation. 
	The correct procedure to achieve this reset is to:  
 
	(a) Set SYSCLKS to 01  
	(b) Clear SOFTRESET to all zero’s  
	(c) Set SOFTRESET to all ones  

	参考用户手册
	*/
	
	byte pmscctrl0[LEN_PMSC_CTRL0]; // 4字节
	readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);

	pmscctrl0[0] = 0x01; // (a) Set SYSCLKS to 01  
	writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
	pmscctrl0[3] = 0x00; // (b) Clear SOFTRESET to all zero’s  
	writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
	delay(10);
	pmscctrl0[0] = 0x00; // return SYSCLKS to AUTO mode
	pmscctrl0[3] = 0xF0; // (c) Set SOFTRESET to all ones  
	writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
	// force into idle mode
	idle();
}

//这个函数只是对本地数据进行操作 并没有写入寄存器
void DW1000Class::enableMode(const byte mode[]) {
	setDataRate(mode[0]); //0:110k  1:850k  2:6.8m
	setPulseFrequency(mode[1]);
	setPreambleLength(mode[2]);
}

// 大概都是一些默认配置 不用去动他
void DW1000Class::tune() {
	// these registers are going to be tuned/configured
	byte agctune1[LEN_AGC_TUNE1];
	byte agctune2[LEN_AGC_TUNE2];
	byte agctune3[LEN_AGC_TUNE3];
	byte drxtune0b[LEN_DRX_TUNE0b];
	byte drxtune1a[LEN_DRX_TUNE1a];
	byte drxtune1b[LEN_DRX_TUNE1b];
	byte drxtune2[LEN_DRX_TUNE2];
	byte drxtune4H[LEN_DRX_TUNE4H];
	byte ldecfg1[LEN_LDE_CFG1];
	byte ldecfg2[LEN_LDE_CFG2];
	byte lderepc[LEN_LDE_REPC];
	byte txpower[LEN_TX_POWER];
	byte rfrxctrlh[LEN_RF_RXCTRLH];
	byte rftxctrl[LEN_RF_TXCTRL];
	byte tcpgdelay[LEN_TC_PGDELAY];
	byte fspllcfg[LEN_FS_PLLCFG];
	byte fsplltune[LEN_FS_PLLTUNE];
	byte fsxtalt[LEN_FS_XTALT];

	// 这些AGC看上去就只是一些默认的配置，不用去动他
	// AGC_TUNE1
	// 根据脉冲发射频率来调节agctune1
	if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(agctune1, 0x8870, LEN_AGC_TUNE1);
	} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(agctune1, 0x889B, LEN_AGC_TUNE1);
	} else {
		// TODO proper error/warning handling
	}
	// AGC_TUNE2
	writeValueToBytes(agctune2, 0x2502A907L, LEN_AGC_TUNE2);
	// AGC_TUNE3
	writeValueToBytes(agctune3, 0x0035, LEN_AGC_TUNE3);

	//保持默认即可
	// DRX_TUNE0b (already optimized according to Table 20 of user manual)
	if(_dataRate == TRX_RATE_110KBPS) {
		writeValueToBytes(drxtune0b, 0x0016, LEN_DRX_TUNE0b);
	} else if(_dataRate == TRX_RATE_850KBPS) {
		writeValueToBytes(drxtune0b, 0x0006, LEN_DRX_TUNE0b);
	} else if(_dataRate == TRX_RATE_6800KBPS) {
		writeValueToBytes(drxtune0b, 0x0001, LEN_DRX_TUNE0b);
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE1a
	if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(drxtune1a, 0x0087, LEN_DRX_TUNE1a);
	} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(drxtune1a, 0x008D, LEN_DRX_TUNE1a);
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE1b
	if(_preambleLength == TX_PREAMBLE_LEN_1536 || _preambleLength == TX_PREAMBLE_LEN_2048 ||
		 _preambleLength == TX_PREAMBLE_LEN_4096) {
		if(_dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(drxtune1b, 0x0064, LEN_DRX_TUNE1b);
		} else {
			// TODO proper error/warning handling
		}
	} else if(_preambleLength != TX_PREAMBLE_LEN_64) {
		if(_dataRate == TRX_RATE_850KBPS || _dataRate == TRX_RATE_6800KBPS) {
			writeValueToBytes(drxtune1b, 0x0020, LEN_DRX_TUNE1b);
		} else {
			// TODO proper error/warning handling
		}
	} else {
		if(_dataRate == TRX_RATE_6800KBPS) {
			writeValueToBytes(drxtune1b, 0x0010, LEN_DRX_TUNE1b);
		} else {
			// TODO proper error/warning handling
		}
	}
	
	// 保持默认
	// DRX_TUNE2
	if(_pacSize == PAC_SIZE_8) {
		if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x311A002DL, LEN_DRX_TUNE2);
		} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x313B006BL, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else if(_pacSize == PAC_SIZE_16) {
		if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x331A0052L, LEN_DRX_TUNE2);
		} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x333B00BEL, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else if(_pacSize == PAC_SIZE_32) {
		if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x351A009AL, LEN_DRX_TUNE2);
		} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x353B015EL, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else if(_pacSize == PAC_SIZE_64) {
		if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			writeValueToBytes(drxtune2, 0x371A011DL, LEN_DRX_TUNE2);
		} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			writeValueToBytes(drxtune2, 0x373B0296L, LEN_DRX_TUNE2);
		} else {
			// TODO proper error/warning handling
		}
	} else {
		// TODO proper error/warning handling
	}
	// DRX_TUNE4H
	if(_preambleLength == TX_PREAMBLE_LEN_64) {
		writeValueToBytes(drxtune4H, 0x0010, LEN_DRX_TUNE4H);
	} else {
		writeValueToBytes(drxtune4H, 0x0028, LEN_DRX_TUNE4H);
	}
	// RF_RXCTRLH
	if(_channel != CHANNEL_4 && _channel != CHANNEL_7) {
		writeValueToBytes(rfrxctrlh, 0xD8, LEN_RF_RXCTRLH);
	} else {
		writeValueToBytes(rfrxctrlh, 0xBC, LEN_RF_RXCTRLH);
	}
	// RX_TXCTRL
	if(_channel == CHANNEL_1) {
		writeValueToBytes(rftxctrl, 0x00005C40L, LEN_RF_TXCTRL);
	} else if(_channel == CHANNEL_2) {
		writeValueToBytes(rftxctrl, 0x00045CA0L, LEN_RF_TXCTRL);
	} else if(_channel == CHANNEL_3) {
		writeValueToBytes(rftxctrl, 0x00086CC0L, LEN_RF_TXCTRL);
	} else if(_channel == CHANNEL_4) {
		writeValueToBytes(rftxctrl, 0x00045C80L, LEN_RF_TXCTRL);
	} else if(_channel == CHANNEL_5) {
		writeValueToBytes(rftxctrl, 0x001E3FE0L, LEN_RF_TXCTRL);
	} else if(_channel == CHANNEL_7) {
		writeValueToBytes(rftxctrl, 0x001E7DE0L, LEN_RF_TXCTRL);
	} else {
		// TODO proper error/warning handling
	}
	// TC_PGDELAY
	if(_channel == CHANNEL_1) {
		writeValueToBytes(tcpgdelay, 0xC9, LEN_TC_PGDELAY);
	} else if(_channel == CHANNEL_2) {
		writeValueToBytes(tcpgdelay, 0xC2, LEN_TC_PGDELAY);
	} else if(_channel == CHANNEL_3) {
		writeValueToBytes(tcpgdelay, 0xC5, LEN_TC_PGDELAY);
	} else if(_channel == CHANNEL_4) {
		writeValueToBytes(tcpgdelay, 0x95, LEN_TC_PGDELAY);
	} else if(_channel == CHANNEL_5) {
		writeValueToBytes(tcpgdelay, 0xC0, LEN_TC_PGDELAY);
	} else if(_channel == CHANNEL_7) {
		writeValueToBytes(tcpgdelay, 0x93, LEN_TC_PGDELAY);
	} else {
		// TODO proper error/warning handling
	}
	// FS_PLLCFG and FS_PLLTUNE
	if(_channel == CHANNEL_1) {
		writeValueToBytes(fspllcfg, 0x09000407L, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0x1E, LEN_FS_PLLTUNE);
	} else if(_channel == CHANNEL_2 || _channel == CHANNEL_4) {
		writeValueToBytes(fspllcfg, 0x08400508L, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0x26, LEN_FS_PLLTUNE);
	} else if(_channel == CHANNEL_3) {
		writeValueToBytes(fspllcfg, 0x08401009L, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0x56, LEN_FS_PLLTUNE);
	} else if(_channel == CHANNEL_5 || _channel == CHANNEL_7) {
		writeValueToBytes(fspllcfg, 0x0800041DL, LEN_FS_PLLCFG);
		writeValueToBytes(fsplltune, 0xBE, LEN_FS_PLLTUNE);
	} else {
		// TODO proper error/warning handling
	}
	// LDE_CFG1
	writeValueToBytes(ldecfg1, 0xD, LEN_LDE_CFG1);
	// LDE_CFG2
	if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		writeValueToBytes(ldecfg2, 0x1607, LEN_LDE_CFG2);
	} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		writeValueToBytes(ldecfg2, 0x0607, LEN_LDE_CFG2);
	} else {
		// TODO proper error/warning handling
	}
	// LDE_REPC
	if(_preambleCode == PREAMBLE_CODE_16MHZ_1 || _preambleCode == PREAMBLE_CODE_16MHZ_2) {
		if(_dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x5998 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x5998, LEN_LDE_REPC);
		}
	} else if(_preambleCode == PREAMBLE_CODE_16MHZ_3 || _preambleCode == PREAMBLE_CODE_16MHZ_8) {
		if(_dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x51EA >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x51EA, LEN_LDE_REPC);
		}
	} else if(_preambleCode == PREAMBLE_CODE_16MHZ_4) {
		if(_dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x428E >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x428E, LEN_LDE_REPC);
		}
	} else if(_preambleCode == PREAMBLE_CODE_16MHZ_5) {
		if(_dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x451E >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x451E, LEN_LDE_REPC);
		}
	} else if(_preambleCode == PREAMBLE_CODE_16MHZ_6) {
		if(_dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x2E14 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x2E14, LEN_LDE_REPC);
		}
	} else if(_preambleCode == PREAMBLE_CODE_16MHZ_7) {
		if(_dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x8000 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x8000, LEN_LDE_REPC);
		}
	} else if(_preambleCode == PREAMBLE_CODE_64MHZ_9) {
		if(_dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x28F4 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x28F4, LEN_LDE_REPC);
		}
	} else if(_preambleCode == PREAMBLE_CODE_64MHZ_10 || _preambleCode == PREAMBLE_CODE_64MHZ_17) {
		if(_dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3332 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x3332, LEN_LDE_REPC);
		}
	} else if(_preambleCode == PREAMBLE_CODE_64MHZ_11) {
		if(_dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3AE0 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x3AE0, LEN_LDE_REPC);
		}
	} else if(_preambleCode == PREAMBLE_CODE_64MHZ_12) {
		if(_dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x3D70 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x3D70, LEN_LDE_REPC);
		}
	} else if(_preambleCode == PREAMBLE_CODE_64MHZ_18 || _preambleCode == PREAMBLE_CODE_64MHZ_19) {
		if(_dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x35C2 >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x35C2, LEN_LDE_REPC);
		}
	} else if(_preambleCode == PREAMBLE_CODE_64MHZ_20) {
		if(_dataRate == TRX_RATE_110KBPS) {
			writeValueToBytes(lderepc, ((0x47AE >> 3) & 0xFFFF), LEN_LDE_REPC);
		} else {
			writeValueToBytes(lderepc, 0x47AE, LEN_LDE_REPC);
		}
	} else {
		// TODO proper error/warning handling
	}

	// power是不是可以无脑最大呢？
	// TX_POWER (enabled smart transmit power control)
	if(_channel == CHANNEL_1 || _channel == CHANNEL_2) {
		if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(_smartPower) {
				writeValueToBytes(txpower, 0x15355575L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x75757575L, LEN_TX_POWER);
			}
		} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(_smartPower) {
				writeValueToBytes(txpower, 0x07274767L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x67676767L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(_channel == CHANNEL_3) {
		if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(_smartPower) {
				writeValueToBytes(txpower, 0x0F2F4F6FL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x6F6F6F6FL, LEN_TX_POWER);
			}
		} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(_smartPower) {
				writeValueToBytes(txpower, 0x2B4B6B8BL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x8B8B8B8BL, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(_channel == CHANNEL_4) {
		if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(_smartPower) {
				writeValueToBytes(txpower, 0x1F1F3F5FL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x5F5F5F5FL, LEN_TX_POWER);
			}
		} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(_smartPower) {
				writeValueToBytes(txpower, 0x3A5A7A9AL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x9A9A9A9AL, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(_channel == CHANNEL_5) {
		if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(_smartPower) {
				writeValueToBytes(txpower, 0x0E082848L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x48484848L, LEN_TX_POWER);
			}
		} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(_smartPower) {
				writeValueToBytes(txpower, 0x25456585L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x85858585L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(_channel == CHANNEL_7) {
		if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(_smartPower) {
				writeValueToBytes(txpower, 0x32527292L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x92929292L, LEN_TX_POWER);
			}
		} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(_smartPower) {
				writeValueToBytes(txpower, 0x5171B1D1L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0xD1D1D1D1L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else {
		// TODO proper error/warning handling
	}

	// Crystal calibration from OTP (if available)
	byte buf_otp[4];
	readBytesOTP(0x01E, buf_otp);
	if (buf_otp[0] == 0) {
		// No trim value available from OTP, use midrange value of 0x10
		writeValueToBytes(fsxtalt, ((0x10 & 0x1F) | 0x60), LEN_FS_XTALT);
	} else {
		writeValueToBytes(fsxtalt, ((buf_otp[0] & 0x1F) | 0x60), LEN_FS_XTALT);
	}
	// write configuration back to chip
	writeBytes(AGC_TUNE, AGC_TUNE1_SUB, agctune1, LEN_AGC_TUNE1);
	writeBytes(AGC_TUNE, AGC_TUNE2_SUB, agctune2, LEN_AGC_TUNE2);
	writeBytes(AGC_TUNE, AGC_TUNE3_SUB, agctune3, LEN_AGC_TUNE3);
	writeBytes(DRX_TUNE, DRX_TUNE0b_SUB, drxtune0b, LEN_DRX_TUNE0b);
	writeBytes(DRX_TUNE, DRX_TUNE1a_SUB, drxtune1a, LEN_DRX_TUNE1a);
	writeBytes(DRX_TUNE, DRX_TUNE1b_SUB, drxtune1b, LEN_DRX_TUNE1b);
	writeBytes(DRX_TUNE, DRX_TUNE2_SUB, drxtune2, LEN_DRX_TUNE2);
	writeBytes(DRX_TUNE, DRX_TUNE4H_SUB, drxtune4H, LEN_DRX_TUNE4H);
	writeBytes(LDE_IF, LDE_CFG1_SUB, ldecfg1, LEN_LDE_CFG1);
	writeBytes(LDE_IF, LDE_CFG2_SUB, ldecfg2, LEN_LDE_CFG2);
	writeBytes(LDE_IF, LDE_REPC_SUB, lderepc, LEN_LDE_REPC);
	writeBytes(TX_POWER, NO_SUB, txpower, LEN_TX_POWER);
	writeBytes(RF_CONF, RF_RXCTRLH_SUB, rfrxctrlh, LEN_RF_RXCTRLH);
	writeBytes(RF_CONF, RF_TXCTRL_SUB, rftxctrl, LEN_RF_TXCTRL);
	writeBytes(TX_CAL, TC_PGDELAY_SUB, tcpgdelay, LEN_TC_PGDELAY);
	writeBytes(FS_CTRL, FS_PLLTUNE_SUB, fsplltune, LEN_FS_PLLTUNE);
	writeBytes(FS_CTRL, FS_PLLCFG_SUB, fspllcfg, LEN_FS_PLLCFG);
	writeBytes(FS_CTRL, FS_XTALT_SUB, fsxtalt, LEN_FS_XTALT);
}

/* ###########################################################################
 * #### Interrupt handling ###################################################
 * ######################################################################### */

//中断 未曾涉及的领域

void DW1000Class::handleInterrupt() {
	// read current status and handle via callbacks
	readSystemEventStatusRegister();
	if(isClockProblem() /* TODO and others */ && _handleError != 0) {
		(*_handleError)();
	}
	if(isTransmitDone() && _handleSent != 0) {
		(*_handleSent)();
		clearTransmitStatus();
	}
	if(isReceiveTimestampAvailable() && _handleReceiveTimestampAvailable != 0) {
		(*_handleReceiveTimestampAvailable)();
		clearReceiveTimestampAvailableStatus();
	}
	if(isReceiveFailed() && _handleReceiveFailed != 0) {
		(*_handleReceiveFailed)();
		clearReceiveStatus();
		if(_permanentReceive) {
			newReceive();
			startReceive();
		}
	} else if(isReceiveTimeout() && _handleReceiveTimeout != 0) {
		(*_handleReceiveTimeout)();
		clearReceiveStatus();
		if(_permanentReceive) {
			newReceive();
			startReceive();
		}
	} else if(isReceiveDone() && _handleReceived != 0) {
		(*_handleReceived)();
		clearReceiveStatus();
		if(_permanentReceive) {
			newReceive();
			startReceive();
		}
	}
	// clear all status that is left unhandled
	clearAllStatus();
}

/* ###########################################################################
 * #### Pretty printed device information ####################################
 * ######################################################################### */

// 下面三个函数都是字面意思

void DW1000Class::getPrintableDeviceIdentifier(char msgBuffer[]) {
	byte data[LEN_DEV_ID];
	readBytes(DEV_ID, NO_SUB, data, LEN_DEV_ID);
	sprintf(msgBuffer, "%02X - model: %d, version: %d, revision: %d",
					(uint16_t)((data[3] << 8) | data[2]), data[1], (data[0] >> 4) & 0x0F, data[0] & 0x0F);
}

void DW1000Class::getPrintableExtendedUniqueIdentifier(char msgBuffer[]) {
	byte data[LEN_EUI];
	readBytes(EUI, NO_SUB, data, LEN_EUI);
	sprintf(msgBuffer, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
					data[7], data[6], data[5], data[4], data[3], data[2], data[1], data[0]);
}

void DW1000Class::getPrintableNetworkIdAndShortAddress(char msgBuffer[]) {
	byte data[LEN_PANADR];
	readBytes(PANADR, NO_SUB, data, LEN_PANADR);
	sprintf(msgBuffer, "PAN: %02X, Short Address: %02X",
					(uint16_t)((data[3] << 8) | data[2]), (uint16_t)((data[1] << 8) | data[0]));
}

void DW1000Class::getPrintableDeviceMode(char msgBuffer[]) {
	// data not read from device! data is from class
	// TODO

	uint8_t prf; //脉冲间隔
	uint16_t plen; //前导码长度
	uint16_t dr; //data rate
	uint8_t ch; //channel
	uint8_t pcode; //前导码？
	
	if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		prf = 16;
	} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		prf = 64;
	} else {
		prf = 0; // error
	}
	if(_preambleLength == TX_PREAMBLE_LEN_64) {
		plen = 64;
	} else if(_preambleLength == TX_PREAMBLE_LEN_128) {
		plen = 128;
	} else if(_preambleLength == TX_PREAMBLE_LEN_256) {
		plen = 256;
	} else if(_preambleLength == TX_PREAMBLE_LEN_512) {
		plen = 512;
	} else if(_preambleLength == TX_PREAMBLE_LEN_1024) {
		plen = 1024;
	} else if(_preambleLength == TX_PREAMBLE_LEN_1536) {
		plen = 1536;
	} else if(_preambleLength == TX_PREAMBLE_LEN_2048) {
		plen = 2048;
	} else if(_preambleLength == TX_PREAMBLE_LEN_4096) {
		plen = 4096;
	} else {
		plen = 0; // error
	}
	if(_dataRate == TRX_RATE_110KBPS) {
		dr = 110;
	} else if(_dataRate == TRX_RATE_850KBPS) {
		dr = 850;
	} else if(_dataRate == TRX_RATE_6800KBPS) {
		dr = 6800;
	} else {
		dr = 0; // error
	}
	ch    = (uint8_t)_channel;
	pcode = (uint8_t)_preambleCode;
	sprintf(msgBuffer, "Data rate: %u kb/s, PRF: %u MHz, Preamble: %u symbols (code #%u), Channel: #%u", dr, prf, plen, pcode, ch);
}

/* ###########################################################################
 * #### DW1000 register read/write ###########################################
 * ######################################################################### */

// 字面意思 一些寄存器的操作

void DW1000Class::readSystemConfigurationRegister() {
	readBytes(SYS_CFG, NO_SUB, _syscfg, LEN_SYS_CFG);
}

void DW1000Class::writeSystemConfigurationRegister() {
	writeBytes(SYS_CFG, NO_SUB, _syscfg, LEN_SYS_CFG);
}

void DW1000Class::readSystemEventStatusRegister() {
	readBytes(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
}

void DW1000Class::readNetworkIdAndDeviceAddress() {
	readBytes(PANADR, NO_SUB, _networkAndAddress, LEN_PANADR);
}

void DW1000Class::writeNetworkIdAndDeviceAddress() {
	writeBytes(PANADR, NO_SUB, _networkAndAddress, LEN_PANADR);
}

void DW1000Class::readSystemEventMaskRegister() {
	readBytes(SYS_MASK, NO_SUB, _sysmask, LEN_SYS_MASK);
}

void DW1000Class::writeSystemEventMaskRegister() {
	writeBytes(SYS_MASK, NO_SUB, _sysmask, LEN_SYS_MASK);
}

void DW1000Class::readChannelControlRegister() {
	readBytes(CHAN_CTRL, NO_SUB, _chanctrl, LEN_CHAN_CTRL);
}

void DW1000Class::writeChannelControlRegister() {
	writeBytes(CHAN_CTRL, NO_SUB, _chanctrl, LEN_CHAN_CTRL);
}

void DW1000Class::readTransmitFrameControlRegister() {
	readBytes(TX_FCTRL, NO_SUB, _txfctrl, LEN_TX_FCTRL);
}

void DW1000Class::writeTransmitFrameControlRegister() {
	writeBytes(TX_FCTRL, NO_SUB, _txfctrl, LEN_TX_FCTRL);
}

/* ###########################################################################
 * #### DW1000 operation functions ###########################################
 * ######################################################################### */

// 字面意思

void DW1000Class::setNetworkId(uint16_t val) {
	_networkAndAddress[2] = (byte)(val & 0xFF);
	_networkAndAddress[3] = (byte)((val >> 8) & 0xFF);
}

void DW1000Class::setDeviceAddress(uint16_t val) {
	_networkAndAddress[0] = (byte)(val & 0xFF);
	_networkAndAddress[1] = (byte)((val >> 8) & 0xFF);
}

// 把十六进制字符转成十六进制数字
uint8_t DW1000Class::nibbleFromChar(char c) {
	if(c >= '0' && c <= '9') {
		return c-'0';
	}
	if(c >= 'a' && c <= 'f') {
		return c-'a'+10;
	}
	if(c >= 'A' && c <= 'F') {
		return c-'A'+10;
	}
	return 255;
}

//把十六进制字符串转化成字节数组
void DW1000Class::convertToByte(char string[], byte* bytes) {
	byte    eui_byte[LEN_EUI];
	// we fill it with the char array under the form of "AA:FF:1C:...."
	for(uint16_t i = 0; i < LEN_EUI; i++) {
		eui_byte[i] = (nibbleFromChar(string[i*3]) << 4)+nibbleFromChar(string[i*3+1]);
	}
	memcpy(bytes, eui_byte, LEN_EUI);
}

//字面意思 测温度和电压  地址传递
void DW1000Class::getTempAndVbat(float& temp, float& vbat) {
	// follow the procedure from section 6.4 of the User Manual
	byte step1 = 0x80; writeBytes(RF_CONF, 0x11, &step1, 1);
	byte step2 = 0x0A; writeBytes(RF_CONF, 0x12, &step2, 1);
	byte step3 = 0x0F; writeBytes(RF_CONF, 0x12, &step3, 1);
	byte step4 = 0x01; writeBytes(TX_CAL, NO_SUB, &step4, 1);
	byte step5 = 0x00; writeBytes(TX_CAL, NO_SUB, &step5, 1);
	byte sar_lvbat = 0; readBytes(TX_CAL, 0x03, &sar_lvbat, 1);
	byte sar_ltemp = 0; readBytes(TX_CAL, 0x04, &sar_ltemp, 1);
	
	// calculate voltage and temperature
	vbat = (sar_lvbat - _vmeas3v3) / 173.0f + 3.3f;
	temp = (sar_ltemp - _tmeas23C) * 1.14f + 23.0f;
}

/*
 *set EUI 字面意思 dw1000的EUI可以用户配置
 * @param eui
 * 形式应为"AA:BB:2A:6c....
 * 不限大小写 
 *
*/
void DW1000Class::setEUI(char eui[]) {
	byte eui_byte[LEN_EUI];
	convertToByte(eui, eui_byte);
	setEUI(eui_byte);
}

// 两眼一黑 0x01中的EUI可以设置和修改？？？
// from chatgpt: dw1000的EUI由用户自行配置
void DW1000Class::setEUI(byte eui[]) {
	//we reverse the address->
	byte    reverseEUI[8];
	uint8_t     size = 8;
	for(uint8_t i    = 0; i < size; i++) {
		*(reverseEUI+i) = *(eui+size-i-1);
	}
	writeBytes(EUI, NO_SUB, reverseEUI, LEN_EUI);
}

//下面讲的都是一些系统配置（0x04）

//这边是一些frame filter的配置 详见用户手册0x04:0x00的前几个bit

//Frame Filtering BIT in the SYS_CFG register
//frame filter enable or disable
void DW1000Class::setFrameFilter(boolean val) {
	setBit(_syscfg, LEN_SYS_CFG, FFEN_BIT, val);
}

// 启用后，模块将会只接受与当前网络ID匹配的帧
void DW1000Class::setFrameFilterBehaveCoordinator(boolean val) {
	setBit(_syscfg, LEN_SYS_CFG, FFBC_BIT, val);
}

//允许信标帧
void DW1000Class::setFrameFilterAllowBeacon(boolean val) {
	setBit(_syscfg, LEN_SYS_CFG, FFAB_BIT, val);
}

//允许数据帧
void DW1000Class::setFrameFilterAllowData(boolean val) {
	setBit(_syscfg, LEN_SYS_CFG, FFAD_BIT, val);
}

//允许确定帧
void DW1000Class::setFrameFilterAllowAcknowledgement(boolean val) {
	setBit(_syscfg, LEN_SYS_CFG, FFAA_BIT, val);
}

//允许mac帧
void DW1000Class::setFrameFilterAllowMAC(boolean val) {
	setBit(_syscfg, LEN_SYS_CFG, FFAM_BIT, val);
}

//允许保留帧
void DW1000Class::setFrameFilterAllowReserved(boolean val) {
	setBit(_syscfg, LEN_SYS_CFG, FFAR_BIT, val);
}

/*
在数据处理中，双缓冲可以用于提高数据处理的效率。
例如，在处理大量数据时，可以使用一个缓冲区来存储当前正在处理的数据，另一个缓冲区用于存储下一批数据。
当当前缓冲区中的数据处理完成后，可以立即切换到下一个缓冲区，从而避免了数据处理的停顿。
*/
void DW1000Class::setDoubleBuffering(boolean val) {
	setBit(_syscfg, LEN_SYS_CFG, DIS_DRXB_BIT, !val);
}

//设置中断引脚极性
void DW1000Class::setInterruptPolarity(boolean val) {
	setBit(_syscfg, LEN_SYS_CFG, HIRQ_POL_BIT, val);
}

//看上去只是receiver自动重启吧？详见用户手册
void DW1000Class::setReceiverAutoReenable(boolean val) {
	setBit(_syscfg, LEN_SYS_CFG, RXAUTR_BIT, val);
}

//下面这些看上去是关于interrupt的 emmm还得再学学

void DW1000Class::interruptOnSent(boolean val) {
	setBit(_sysmask, LEN_SYS_MASK, TXFRS_BIT, val);
}

void DW1000Class::interruptOnReceived(boolean val) {
	setBit(_sysmask, LEN_SYS_MASK, RXDFR_BIT, val);
	setBit(_sysmask, LEN_SYS_MASK, RXFCG_BIT, val);
}

void DW1000Class::interruptOnReceiveFailed(boolean val) {
	setBit(_sysmask, LEN_SYS_STATUS, LDEERR_BIT, val);
	setBit(_sysmask, LEN_SYS_STATUS, RXFCE_BIT, val);
	setBit(_sysmask, LEN_SYS_STATUS, RXPHE_BIT, val);
	setBit(_sysmask, LEN_SYS_STATUS, RXRFSL_BIT, val);
}

void DW1000Class::interruptOnReceiveTimeout(boolean val) {
	setBit(_sysmask, LEN_SYS_MASK, RXRFTO_BIT, val);
}

void DW1000Class::interruptOnReceiveTimestampAvailable(boolean val) {
	setBit(_sysmask, LEN_SYS_MASK, LDEDONE_BIT, val);
}

void DW1000Class::interruptOnAutomaticAcknowledgeTrigger(boolean val) {
	setBit(_sysmask, LEN_SYS_MASK, AAT_BIT, val);
}

// 字面意思 设置天线延迟
void DW1000Class::setAntennaDelay(const uint16_t value) {
	_antennaDelay.setTimestamp(value);
	_antennaCalibrated = true;
}

uint16_t DW1000Class::getAntennaDelay() {
	return static_cast<uint16_t>(_antennaDelay.getTimestamp());
}

void DW1000Class::clearInterrupts() {
	memset(_sysmask, 0, LEN_SYS_MASK);
}

//idle 即空闲状态 PLL时钟锁定 可以进入TX或RX状态
void DW1000Class::idle() {
	memset(_sysctrl, 0, LEN_SYS_CTRL);

	/*
	Transceiver Off.  
	When this is set the DW1000 returns to idle mode immediately.
	Any TX or RX activity that is in progress at that time will be aborted.  
	The TRXOFF bit will clear as soon as the DW1000 sees it and returns the IC to idle mode.
	*/
	setBit(_sysctrl, LEN_SYS_CTRL, TRXOFF_BIT, true);
	
	_deviceMode = IDLE_MODE;
	writeBytes(SYS_CTRL, NO_SUB, _sysctrl, LEN_SYS_CTRL);
}


void DW1000Class::newReceive() {
	idle();
	memset(_sysctrl, 0, LEN_SYS_CTRL);
	clearReceiveStatus();
	_deviceMode = RX_MODE;
}

void DW1000Class::startReceive() {
	setBit(_sysctrl, LEN_SYS_CTRL, SFCST_BIT, !_frameCheck);
	setBit(_sysctrl, LEN_SYS_CTRL, RXENAB_BIT, true);
	writeBytes(SYS_CTRL, NO_SUB, _sysctrl, LEN_SYS_CTRL);
}

void DW1000Class::newTransmit() {
	idle();
	memset(_sysctrl, 0, LEN_SYS_CTRL);
	clearTransmitStatus();
	_deviceMode = TX_MODE;
}

void DW1000Class::startTransmit() {
	writeTransmitFrameControlRegister();
	setBit(_sysctrl, LEN_SYS_CTRL, SFCST_BIT, !_frameCheck);
	setBit(_sysctrl, LEN_SYS_CTRL, TXSTRT_BIT, true);
	writeBytes(SYS_CTRL, NO_SUB, _sysctrl, LEN_SYS_CTRL);
	if(_permanentReceive) {
		memset(_sysctrl, 0, LEN_SYS_CTRL);
		_deviceMode = RX_MODE;
		startReceive();
	} else {
		_deviceMode = IDLE_MODE;
	}
}

// 这个看起来是要先运行newconfig 然后在进行各种调试 最后在commit
void DW1000Class::newConfiguration() {
	idle();
	readNetworkIdAndDeviceAddress();
	readSystemConfigurationRegister();
	readChannelControlRegister();
	readTransmitFrameControlRegister();
	readSystemEventMaskRegister();
}

void DW1000Class::commitConfiguration() {
	// write all configurations back to device
	writeNetworkIdAndDeviceAddress();
	writeSystemConfigurationRegister();
	writeChannelControlRegister();
	writeTransmitFrameControlRegister();
	writeSystemEventMaskRegister();
	// tune according to configuration
	tune();
	// TODO check not larger two bytes integer
	byte antennaDelayBytes[DW1000Time::LENGTH_TIMESTAMP];
	if( _antennaDelay.getTimestamp() == 0 && _antennaCalibrated == false) {
		_antennaDelay.setTimestamp(16384);
		_antennaCalibrated = true;
	} // Compatibility with old versions.
	_antennaDelay.getTimestamp(antennaDelayBytes);

	writeBytes(TX_ANTD, NO_SUB, antennaDelayBytes, LEN_TX_ANTD);
	writeBytes(LDE_IF, LDE_RXANTD_SUB, antennaDelayBytes, LEN_LDE_RXANTD);
}

// 详见用户手册 0x0D bit:7
void DW1000Class::waitForResponse(boolean val) {
	setBit(_sysctrl, LEN_SYS_CTRL, WAIT4RESP_BIT, val);
}

void DW1000Class::suppressFrameCheck(boolean val) {
	_frameCheck = !val;
}

void DW1000Class::useSmartPower(boolean smartPower) {
	_smartPower = smartPower;
	setBit(_syscfg, LEN_SYS_CFG, DIS_STXP_BIT, !smartPower);
}

DW1000Time DW1000Class::setDelay(const DW1000Time& delay) {
	if(_deviceMode == TX_MODE) {
		setBit(_sysctrl, LEN_SYS_CTRL, TXDLYS_BIT, true);
	} else if(_deviceMode == RX_MODE) {
		setBit(_sysctrl, LEN_SYS_CTRL, RXDLYS_BIT, true);
	} else {
		// in idle, ignore
		return DW1000Time();
	}
	byte       delayBytes[5];
	DW1000Time futureTime;
	getSystemTimestamp(futureTime);
	futureTime += delay;
	futureTime.getTimestamp(delayBytes);
	delayBytes[0] = 0;
	delayBytes[1] &= 0xFE;
	writeBytes(DX_TIME, NO_SUB, delayBytes, LEN_DX_TIME);
	// adjust expected time with configured antenna delay
	futureTime.setTimestamp(delayBytes);
	futureTime += _antennaDelay;
	return futureTime;
}


void DW1000Class::setDataRate(byte rate) {
	rate &= 0x03; //保留最低两位，将其他位清零
	_txfctrl[1] &= 0x83; //0x83:1000 0011  按位与运算，将这几位0清零
	_txfctrl[1] |= (byte)((rate << 5) & 0xFF);  //将rate的最后三位添加到 _txfctrl 的最高三位上
	
	// special 110kbps flag
	// 字面意思
	if(rate == TRX_RATE_110KBPS) {
		setBit(_syscfg, LEN_SYS_CFG, RXM110K_BIT, true);
	} else {
		setBit(_syscfg, LEN_SYS_CFG, RXM110K_BIT, false);
	}

	/*
	DWSFD_BIT enables a non-standard Decawave proprietary SFD sequence.  

	When DWSFD is 0, and TNSSFD and RNSSFD are deasserted low, 
	then the SFD sequence used by the DW1000 
	will be the one prescribed by the IEEE 802.15.4-2011 standard. 
	*/
	// SFD mode and type (non-configurable, as in Table )
	if(rate == TRX_RATE_6800KBPS) {
		// use IEEE 802.15.4-2011 standard. 
		setBit(_chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, false);
		setBit(_chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, false);
		setBit(_chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, false);
	} else if (rate == TRX_RATE_850KBPS) {
		// use user defined SFD sequence (non-standard 16bit SFD)
		setBit(_chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, true);
		setBit(_chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, true);
		setBit(_chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, true);
	} else { // maybe 110kbps
		// 用户手册中讲0x21寄存器的时候讲到了就是要这样设置
		// non-standard 64bit SFD
		setBit(_chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, true);
		setBit(_chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, false);
		setBit(_chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, false);
	}
	byte sfdLength;
	if(rate == TRX_RATE_6800KBPS) {
		sfdLength = 0x08; //8
	} else if(rate == TRX_RATE_850KBPS) {
		sfdLength = 0x10; //16
	} else {
		sfdLength = 0x40; //64
	}

	//将sfdlength写入寄存器0x21:0x00
	writeBytes(USR_SFD, SFD_LENGTH_SUB, &sfdLength, LEN_SFD_LENGTH);
	_dataRate = rate;
}

/*
这个函数修改了 0x08 和 0x1F 变量的值，但是并没有写入寄存器
*/
void DW1000Class::setPulseFrequency(byte freq) {
	freq &= 0x03; //取最低两位

	//设置tx prf和rx prf相等
	//tx 可以用4mhz但是rx不能用4mhz
	_txfctrl[2] &= 0xFC;  //11111100 将最低两位去掉
	_txfctrl[2] |= (byte)(freq & 0xFF); 

	/*
	脉冲重复频率（Pulse Repetition Frequency，PRF）是指在测量系统中，每秒钟发射的脉冲数量。
	PRF是一个重要的参数，因为它影响着系统的测距能力和最大无模糊距离（即不产生距离模糊的最大测量范围）。

	较高的PRF可以提供更好的速度测量分辨率和更快的数据更新率，但可能会导致距离模糊问题，
	因为发射的下一个脉冲可能会在上一个回波返回之前被发射。
	而较低的PRF则可以避免距离模糊，但可能会降低速度分辨率和数据更新率。

	在不同的应用中，脉冲重复频率的选择需要在这些因素之间进行权衡，以满足具体的测量需求。
	*/
	_chanctrl[2] &= 0xF3; // 0xF3:11110011
	_chanctrl[2] |= (byte)((freq << 2) & 0xFF); //将freq最后两位写入刚刚抹掉的部分
	//RXPRF 
	/*
	This two bit field selects the PRF used in the receiver. 
	Values allowed here are 
	binary 01 to select the 16 MHz PRF or 
	binary 10 to select the 64 MHz PRF.  
	Other values are reserved and should not be set. 
	*/

	_pulseFrequency = freq;
}

byte DW1000Class::getPulseFrequency() {
	return _pulseFrequency;
}

//字面意思 设置前导码长度
//并没有写入寄存器
void DW1000Class::setPreambleLength(byte prealen) {
	prealen &= 0x0F; //00001111
	_txfctrl[2] &= 0xC3; //11000011
	_txfctrl[2] |= (byte)((prealen << 2) & 0xFF); //将prealen最后四位写入txfctrl[2]中间四位

	if(prealen == TX_PREAMBLE_LEN_64 || prealen == TX_PREAMBLE_LEN_128) {
		_pacSize = PAC_SIZE_8;
	} else if(prealen == TX_PREAMBLE_LEN_256 || prealen == TX_PREAMBLE_LEN_512) {
		_pacSize = PAC_SIZE_16;
	} else if(prealen == TX_PREAMBLE_LEN_1024) {
		_pacSize = PAC_SIZE_32;
	} else {
		_pacSize = PAC_SIZE_64;
	}
	_preambleLength = prealen;
}

// 帧长度
void DW1000Class::useExtendedFrameLength(boolean val) {
	_extendedFrameLength = (val ? FRAME_LENGTH_EXTENDED : FRAME_LENGTH_NORMAL);
	_syscfg[2] &= 0xFC;
	_syscfg[2] |= _extendedFrameLength;
}


void DW1000Class::receivePermanently(boolean val) {
	_permanentReceive = val;
	if(val) {
		// in case permanent, also reenable receiver once failed
		setReceiverAutoReenable(true);
		writeSystemConfigurationRegister();
	}
}

//字面意思
void DW1000Class::setChannel(byte channel) {
	channel &= 0xF;
	_chanctrl[0] = ((channel | (channel << 4)) & 0xFF);
	_channel = channel;
	// Set preambleCode in based of CHANNEL. see chapter 10.5, table 61, dw1000 user manual
	if(_channel == CHANNEL_1) {
		if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			setPreambleCode(PREAMBLE_CODE_16MHZ_2);
		} else {
			setPreambleCode(PREAMBLE_CODE_64MHZ_10);
		}
	} else if(_channel == CHANNEL_3) {
		if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			setPreambleCode(PREAMBLE_CODE_16MHZ_6);
		} else {
			setPreambleCode(PREAMBLE_CODE_64MHZ_10);
		}
	} else if(_channel == CHANNEL_4 || _channel == CHANNEL_7) {
		if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			setPreambleCode(PREAMBLE_CODE_16MHZ_8);
		} else {
			setPreambleCode(PREAMBLE_CODE_64MHZ_18);
		}
	} else if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		setPreambleCode(PREAMBLE_CODE_16MHZ_4);
	} else {
		setPreambleCode(PREAMBLE_CODE_64MHZ_10);
	}
}

// 设置前导码
void DW1000Class::setPreambleCode(byte preacode) {
	preacode &= 0x1F;
	_chanctrl[2] &= 0x3F;
	_chanctrl[2] |= ((preacode << 6) & 0xFF);
	_chanctrl[3] = 0x00;
	_chanctrl[3] = ((((preacode >> 2) & 0x07) | (preacode << 3)) & 0xFF);
	_preambleCode = preacode;
}

void DW1000Class::setDefaults() {
	if(_deviceMode == TX_MODE) {
		
	} else if(_deviceMode == RX_MODE) {
		
	} else if(_deviceMode == IDLE_MODE) {
		useExtendedFrameLength(false);
		useSmartPower(false);
		suppressFrameCheck(false);
		//for global frame filtering
		setFrameFilter(false);
		/* old defaults with active frame filter - better set filter in every script where you really need it
		setFrameFilter(true);
		//for data frame (poll, poll_ack, range, range report, range failed) filtering
		setFrameFilterAllowData(true);
		//for reserved (blink) frame filtering
		setFrameFilterAllowReserved(true);
		//setFrameFilterAllowMAC(true);
		//setFrameFilterAllowBeacon(true);
		//setFrameFilterAllowAcknowledgement(true);
		*/
		interruptOnSent(true);
		interruptOnReceived(true);
		interruptOnReceiveFailed(true);
		interruptOnReceiveTimestampAvailable(false);
		interruptOnAutomaticAcknowledgeTrigger(true);
		setReceiverAutoReenable(true);
		// default mode when powering up the chip
		// still explicitly selected for later tuning
		enableMode(MODE_LONGDATA_RANGE_LOWPOWER);
		
		// TODO add channel and code to mode tuples
	    // TODO add channel and code settings with checks (see DW1000 user manual 10.5 table 61)/
	    setChannel(CHANNEL_5);
		if(getPulseFrequency() == TX_PULSE_FREQ_16MHZ) {
			setPreambleCode(PREAMBLE_CODE_16MHZ_4);
		} else {
			setPreambleCode(PREAMBLE_CODE_64MHZ_10);
		}
	}
}

void DW1000Class::setData(byte data[], uint16_t n) {
	if(_frameCheck) {
		n += 2; // two bytes CRC-16
	}
	if(n > LEN_EXT_UWB_FRAMES) {
		return; // TODO proper error handling: frame/buffer size
	}
	if(n > LEN_UWB_FRAMES && !_extendedFrameLength) {
		return; // TODO proper error handling: frame/buffer size
	}
	// transmit data and length
	writeBytes(TX_BUFFER, NO_SUB, data, n);

	// 表示数据长度
	_txfctrl[0] = (byte)(n & 0xFF); // 1 byte (regular length + 1 bit)
	_txfctrl[1] &= 0xE0;
	_txfctrl[1] |= (byte)((n >> 8) & 0x03);  // 2 added bits if extended length
}

//set data reload
void DW1000Class::setData(const String& data) {
	uint16_t n = data.length()+1;
	byte* dataBytes = (byte*)malloc(n);
	data.getBytes(dataBytes, n);
	setData(dataBytes, n);
	free(dataBytes);
}

// TODO reorder
// TX mode:_txfctrl里面提取出length
// RX mode:从寄存器里面读取接收到信息的长度
uint16_t DW1000Class::getDataLength() {
	uint16_t len = 0;
	if(_deviceMode == TX_MODE) {
		// 10 bits of TX frame control register
		len = ((((uint16_t)_txfctrl[1] << 8) | (uint16_t)_txfctrl[0]) & 0x03FF);
	} else if(_deviceMode == RX_MODE) {
		// 10 bits of RX frame control register
		byte rxFrameInfo[LEN_RX_FINFO];
		readBytes(RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
		len = ((((uint16_t)rxFrameInfo[1] << 8) | (uint16_t)rxFrameInfo[0]) & 0x03FF);
	}
	if(_frameCheck && len > 2) {
		return len-2;
	}
	return len;
}

//从RX寄存器里面读取数据
void DW1000Class::getData(byte data[], uint16_t n) {
	if(n <= 0) {
		return;
	}
	readBytes(RX_BUFFER, NO_SUB, data, n);
}

//reload of get data
void DW1000Class::getData(String& data) {
	uint16_t i;
	uint16_t n = getDataLength(); // number of bytes w/o the two FCS ones
	if(n <= 0) { // TODO
		return;
	}
	byte* dataBytes = (byte*)malloc(n);
	getData(dataBytes, n);
	// clear string
	data.remove(0);
	data  = "";
	// append to string
	for(i = 0; i < n; i++) {
		data += (char)dataBytes[i];
	}
	free(dataBytes);
}

// 获得tx时间戳 时间戳为发射PHY header发射的时间戳+天线延迟 即信号从天线发射出的时间戳
void DW1000Class::getTransmitTimestamp(DW1000Time& time) {
	byte txTimeBytes[LEN_TX_STAMP];
	readBytes(TX_TIME, TX_STAMP_SUB, txTimeBytes, LEN_TX_STAMP);
	time.setTimestamp(txTimeBytes);
}

// 获得rx时间戳 时间戳为天线接收到PHR信号时的时间（计算过天线延迟）
void DW1000Class::getReceiveTimestamp(DW1000Time& time) {
	byte rxTimeBytes[LEN_RX_STAMP];
	readBytes(RX_TIME, RX_STAMP_SUB, rxTimeBytes, LEN_RX_STAMP);
	time.setTimestamp(rxTimeBytes);
	// correct timestamp (i.e. consider range bias)
	correctTimestamp(time);
}

// TODO check function, different type violations between byte and int
// 字面意思 根据某些已经设置过的参数校准时间戳
void DW1000Class::correctTimestamp(DW1000Time& timestamp) {
	// base line dBm, which is -61, 2 dBm steps, total 18 data points (down to -95 dBm)
	float rxPowerBase     = -(getReceivePower()+61.0f)*0.5f;
	int16_t   rxPowerBaseLow  = (int16_t)rxPowerBase; // TODO check type
	int16_t   rxPowerBaseHigh = rxPowerBaseLow+1; // TODO check type
	if(rxPowerBaseLow <= 0) {
		rxPowerBaseLow  = 0;
		rxPowerBaseHigh = 0;
	} else if(rxPowerBaseHigh >= 17) {
		rxPowerBaseLow  = 17;
		rxPowerBaseHigh = 17;
	}
	// select range low/high values from corresponding table
	int16_t rangeBiasHigh;
	int16_t rangeBiasLow;
	if(_channel == CHANNEL_4 || _channel == CHANNEL_7) {
		// 900 MHz receiver bandwidth
		if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseHigh] : BIAS_900_16[rxPowerBaseHigh]);
			rangeBiasHigh <<= 1;
			rangeBiasLow  = (rxPowerBaseLow < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseLow] : BIAS_900_16[rxPowerBaseLow]);
			rangeBiasLow <<= 1;
		} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseHigh] : BIAS_900_64[rxPowerBaseHigh]);
			rangeBiasHigh <<= 1;
			rangeBiasLow  = (rxPowerBaseLow < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseLow] : BIAS_900_64[rxPowerBaseLow]);
			rangeBiasLow <<= 1;
		} else {
			// TODO proper error handling
			return;
		}
	} else {
		// 500 MHz receiver bandwidth
		if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseHigh] : BIAS_500_16[rxPowerBaseHigh]);
			rangeBiasLow  = (rxPowerBaseLow < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseLow] : BIAS_500_16[rxPowerBaseLow]);
		} else if(_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseHigh] : BIAS_500_64[rxPowerBaseHigh]);
			rangeBiasLow  = (rxPowerBaseLow < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseLow] : BIAS_500_64[rxPowerBaseLow]);
		} else {
			// TODO proper error handling
			return;
		}
	}
	// linear interpolation of bias values
	float      rangeBias = rangeBiasLow+(rxPowerBase-rxPowerBaseLow)*(rangeBiasHigh-rangeBiasLow);
	// range bias [mm] to timestamp modification value conversion
	DW1000Time adjustmentTime;
	adjustmentTime.setTimestamp((int16_t)(rangeBias*DW1000Time::DISTANCE_OF_RADIO_INV*0.001f));
	// apply correction
	timestamp -= adjustmentTime;
}

// 字面意思 从寄存器读取系统时间戳
void DW1000Class::getSystemTimestamp(DW1000Time& time) {
	byte sysTimeBytes[LEN_SYS_TIME];
	readBytes(SYS_TIME, NO_SUB, sysTimeBytes, LEN_SYS_TIME);
	time.setTimestamp(sysTimeBytes);
}

// 下面三个函数只是从寄存器读取时间戳

void DW1000Class::getTransmitTimestamp(byte data[]) {
	readBytes(TX_TIME, TX_STAMP_SUB, data, LEN_TX_STAMP);
}

void DW1000Class::getReceiveTimestamp(byte data[]) {
	readBytes(RX_TIME, RX_STAMP_SUB, data, LEN_RX_STAMP);
}

void DW1000Class::getSystemTimestamp(byte data[]) {
	readBytes(SYS_TIME, NO_SUB, data, LEN_SYS_TIME);
}

boolean DW1000Class::isTransmitDone() {
	return getBit(_sysstatus, LEN_SYS_STATUS, TXFRS_BIT);
}

boolean DW1000Class::isReceiveTimestampAvailable() {
	return getBit(_sysstatus, LEN_SYS_STATUS, LDEDONE_BIT);
}

boolean DW1000Class::isReceiveDone() {
	if(_frameCheck) {
		return getBit(_sysstatus, LEN_SYS_STATUS, RXFCG_BIT);
	}
	return getBit(_sysstatus, LEN_SYS_STATUS, RXDFR_BIT);
}

boolean DW1000Class::isReceiveFailed() {
	boolean ldeErr, rxCRCErr, rxHeaderErr, rxDecodeErr;
	ldeErr      = getBit(_sysstatus, LEN_SYS_STATUS, LDEERR_BIT);
	rxCRCErr    = getBit(_sysstatus, LEN_SYS_STATUS, RXFCE_BIT);
	rxHeaderErr = getBit(_sysstatus, LEN_SYS_STATUS, RXPHE_BIT);
	rxDecodeErr = getBit(_sysstatus, LEN_SYS_STATUS, RXRFSL_BIT);
	if(ldeErr || rxCRCErr || rxHeaderErr || rxDecodeErr) {
		return true;
	}
	return false;
}

//Checks to see any of the three timeout bits in sysstatus are high (RXRFTO (Frame Wait timeout), RXPTO (Preamble timeout), RXSFDTO (Start frame delimiter(?) timeout).
boolean DW1000Class::isReceiveTimeout() {
	return (getBit(_sysstatus, LEN_SYS_STATUS, RXRFTO_BIT) | getBit(_sysstatus, LEN_SYS_STATUS, RXPTO_BIT) | getBit(_sysstatus, LEN_SYS_STATUS, RXSFDTO_BIT));
}

boolean DW1000Class::isClockProblem() {
	boolean clkllErr, rfllErr;
	clkllErr = getBit(_sysstatus, LEN_SYS_STATUS, CLKPLL_LL_BIT);
	rfllErr  = getBit(_sysstatus, LEN_SYS_STATUS, RFPLL_LL_BIT);
	if(clkllErr || rfllErr) {
		return true;
	}
	return false;
}

void DW1000Class::clearAllStatus() {
	//Latched bits in status register are reset by writing 1 to them
	memset(_sysstatus, 0xff, LEN_SYS_STATUS);
	writeBytes(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
}

void DW1000Class::clearReceiveTimestampAvailableStatus() {
	setBit(_sysstatus, LEN_SYS_STATUS, LDEDONE_BIT, true);
	writeBytes(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
}

void DW1000Class::clearReceiveStatus() {
	// clear latched RX bits (i.e. write 1 to clear)
	setBit(_sysstatus, LEN_SYS_STATUS, RXDFR_BIT, true);
	setBit(_sysstatus, LEN_SYS_STATUS, LDEDONE_BIT, true);
	setBit(_sysstatus, LEN_SYS_STATUS, LDEERR_BIT, true);
	setBit(_sysstatus, LEN_SYS_STATUS, RXPHE_BIT, true);
	setBit(_sysstatus, LEN_SYS_STATUS, RXFCE_BIT, true);
	setBit(_sysstatus, LEN_SYS_STATUS, RXFCG_BIT, true);
	setBit(_sysstatus, LEN_SYS_STATUS, RXRFSL_BIT, true);
	writeBytes(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
}

void DW1000Class::clearTransmitStatus() {
	// clear latched TX bits
	setBit(_sysstatus, LEN_SYS_STATUS, TXFRB_BIT, true);
	setBit(_sysstatus, LEN_SYS_STATUS, TXPRS_BIT, true);
	setBit(_sysstatus, LEN_SYS_STATUS, TXPHS_BIT, true);
	setBit(_sysstatus, LEN_SYS_STATUS, TXFRS_BIT, true);
	writeBytes(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
}

float DW1000Class::getReceiveQuality() {
	byte         noiseBytes[LEN_STD_NOISE];
	byte         fpAmpl2Bytes[LEN_FP_AMPL2];
	uint16_t     noise, f2;
	readBytes(RX_FQUAL, STD_NOISE_SUB, noiseBytes, LEN_STD_NOISE);
	readBytes(RX_FQUAL, FP_AMPL2_SUB, fpAmpl2Bytes, LEN_FP_AMPL2);
	noise = (uint16_t)noiseBytes[0] | ((uint16_t)noiseBytes[1] << 8);
	f2    = (uint16_t)fpAmpl2Bytes[0] | ((uint16_t)fpAmpl2Bytes[1] << 8);
	return (float)f2/noise;
}

float DW1000Class::getFirstPathPower() {
	byte         fpAmpl1Bytes[LEN_FP_AMPL1];
	byte         fpAmpl2Bytes[LEN_FP_AMPL2];
	byte         fpAmpl3Bytes[LEN_FP_AMPL3];
	byte         rxFrameInfo[LEN_RX_FINFO];
	uint16_t     f1, f2, f3, N;
	float        A, corrFac;
	readBytes(RX_TIME, FP_AMPL1_SUB, fpAmpl1Bytes, LEN_FP_AMPL1);
	readBytes(RX_FQUAL, FP_AMPL2_SUB, fpAmpl2Bytes, LEN_FP_AMPL2);
	readBytes(RX_FQUAL, FP_AMPL3_SUB, fpAmpl3Bytes, LEN_FP_AMPL3);
	readBytes(RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
	f1 = (uint16_t)fpAmpl1Bytes[0] | ((uint16_t)fpAmpl1Bytes[1] << 8);
	f2 = (uint16_t)fpAmpl2Bytes[0] | ((uint16_t)fpAmpl2Bytes[1] << 8);
	f3 = (uint16_t)fpAmpl3Bytes[0] | ((uint16_t)fpAmpl3Bytes[1] << 8);
	N  = (((uint16_t)rxFrameInfo[2] >> 4) & 0xFF) | ((uint16_t)rxFrameInfo[3] << 4);
	if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		A       = 113.77;
		corrFac = 2.3334;
	} else {
		A       = 121.74;
		corrFac = 1.1667;
	}
	float estFpPwr = 10.0*log10(((float)f1*(float)f1+(float)f2*(float)f2+(float)f3*(float)f3)/((float)N*(float)N))-A;
	if(estFpPwr <= -88) {
		return estFpPwr;
	} else {
		// approximation of Fig. 22 in user manual for dbm correction
		estFpPwr += (estFpPwr+88)*corrFac;
	}
	return estFpPwr;
}

float DW1000Class::getReceivePower() {
	byte     cirPwrBytes[LEN_CIR_PWR];
	byte     rxFrameInfo[LEN_RX_FINFO];
	uint32_t twoPower17 = 131072;
	uint16_t C, N;
	float    A, corrFac;
	readBytes(RX_FQUAL, CIR_PWR_SUB, cirPwrBytes, LEN_CIR_PWR);
	readBytes(RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
	C = (uint16_t)cirPwrBytes[0] | ((uint16_t)cirPwrBytes[1] << 8);
	N = (((uint16_t)rxFrameInfo[2] >> 4) & 0xFF) | ((uint16_t)rxFrameInfo[3] << 4);
	if(_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		A       = 113.77;
		corrFac = 2.3334;
	} else {
		A       = 121.74;
		corrFac = 1.1667;
	}
	float estRxPwr = 10.0*log10(((float)C*(float)twoPower17)/((float)N*(float)N))-A;
	if(estRxPwr <= -88) {
		return estRxPwr;
	} else {
		// approximation of Fig. 22 in user manual for dbm correction
		estRxPwr += (estRxPwr+88)*corrFac;
	}
	return estRxPwr;
}

/* ###########################################################################
 * #### Helper functions #####################################################
 * ######################################################################### */

/*
 * Set the value of a bit in an array of bytes that are considered
 * consecutive and stored from MSB to LSB.
 * @param data
 * 		The number as byte array.
 * @param n
 * 		The number of bytes in the array.
 * @param bit
 * 		The position of the bit to be set.
 * @param val
 *		The boolean value to be set to the given bit position.

 这个setbit函数只是对本地数组进行操作，并没有与模块交互

 */
void DW1000Class::setBit(byte data[], uint16_t n, uint16_t bit, boolean val) {
	uint16_t idx;
	uint8_t shift;
	
	idx = bit/8;
	if(idx >= n) {
		return; // TODO proper error handling: out of bounds
	}
	byte* targetByte = &data[idx];
	shift = bit%8;
	if(val) {
		bitSet(*targetByte, shift);
	} else {
		bitClear(*targetByte, shift);
	}
}

/*
 * Check the value of a bit in an array of bytes that are considered
 * consecutive and stored from MSB to LSB.
 * @param data
 * 		The number as byte array.
 * @param n
 * 		The number of bytes in the array.
 * @param bit
 * 		The position of the bit to be checked.
 */
boolean DW1000Class::getBit(byte data[], uint16_t n, uint16_t bit) {
	/*
	选中的代码定义了一个名为 getBit 的成员函数，该函数属于 DW1000Class 类。这个函数的作用是检查一个字节数组中特定位置的位是否为1。

	函数的参数如下：

	data[]：一个字节数组，存储待检查的数值。
	n：字节数组的长度。
	bit：需要检查的位的位置。
	函数的返回值是一个布尔值，表示指定位置的位是否为1。

	函数的实现逻辑如下：

	计算位所在的字节索引 idx，通过将 bit 除以 8 得到。
	如果索引 idx 超出了数组的长度 n，则返回 false，表示位超出了数组的范围。
	如果索引 idx 在数组范围内，则继续执行。
	*/
	uint16_t idx;
	uint8_t  shift;
	
	idx = bit/8;
	if(idx >= n) {
		return false; // TODO proper error handling: out of bounds
	}
	byte targetByte = data[idx];
	shift = bit%8;
	
	return bitRead(targetByte, shift); // TODO wrong type returned byte instead of boolean
}


void DW1000Class::writeValueToBytes(byte data[], int32_t val, uint16_t n) {
	/*
	这段代码定义了一个名为 writeValueToBytes 的函数，
	它是 DW1000Class 类的成员函数。
	这个函数的功能是将一个32位的整数值（int32_t val）拆分成字节，
	并将这些字节存储到一个字节数组（byte data[]）中。
	*/
	
	uint16_t i;
	for(i = 0; i < n; i++) {
		data[i] = ((val >> (i*8)) & 0xFF); // TODO bad types - signed unsigned problem
	}
}

// TODO incomplete doc
/*
 * Read bytes from the DW1000. Number of bytes depend on register length.
 * @param cmd
 * 		The register address (see Chapter 7 in the DW1000 user manual).
 * @param data
 *		The data array to be read into.
 * @param n
 *		The number of bytes expected to be received.
 */
void DW1000Class::readBytes(byte cmd, uint16_t offset, byte data[], uint16_t n) {
	byte header[3];
	uint8_t headerLen = 1;
	uint16_t i = 0;
	
	// build SPI header
	if(offset == NO_SUB) {
		header[0] = READ | cmd;
	} else {
		header[0] = READ_SUB | cmd;
		if(offset < 128) {
			header[1] = (byte)offset;
			headerLen++;
		} else {
			header[1] = RW_SUB_EXT | (byte)offset;
			header[2] = (byte)(offset >> 7);
			headerLen += 2;
		}
	}
	SPI.beginTransaction(*_currentSPI);
	digitalWrite(_ss, LOW);
	for(i = 0; i < headerLen; i++) {
		SPI.transfer(header[i]); // send header
	}
	for(i = 0; i < n; i++) {
		data[i] = SPI.transfer(JUNK); // read values
	}
	delayMicroseconds(5);
	digitalWrite(_ss, HIGH);
	SPI.endTransaction();
}


// always 4 bytes
// TODO why always 4 bytes? can be different, see p. 58 table 10 otp memory map
// 在模块的OTP memory中操作数据需要通过OTP寄存器来操作
/*
OTP是“One-Time Programmable”的缩写，即一次性可编程。
在电子学和计算机科学中，OTP通常指的是一种特殊类型的非易失性存储器（Non-Volatile Memory, NVM）
它允许数据被写入一次，之后就不能再被修改或擦除。
*/

/*
 * 从OTP memory读取data
 * @param address
 * OTP memory的地址
 * @param data
 * 将获取的数据写入data中
 *
*/
void DW1000Class::readBytesOTP(uint16_t address, byte data[]) {
	byte addressBytes[LEN_OTP_ADDR];
	
	// p60 - 6.3.3 Reading a value from OTP memory
	// bytes of address
	addressBytes[0] = (address & 0xFF); //取低位
	addressBytes[1] = ((address >> 8) & 0xFF); //取高位

	// set address
	// 在OTP寄存器中写入要读取的OTP memory对应的地址
	// OTP_IF的意思也许是OTP interface吧
	writeBytes(OTP_IF, OTP_ADDR_SUB, addressBytes, LEN_OTP_ADDR);
	
	// switch into read mode
	// 好奇怪啊 为什么要写两次OTPRDEN呢？屎山就不动它了（doge）
	writeByte(OTP_IF, OTP_CTRL_SUB, 0x03); // OTPRDEN | OTPREAD
	writeByte(OTP_IF, OTP_CTRL_SUB, 0x01); // OTPRDEN
	// read value/block - 4 bytes
	readBytes(OTP_IF, OTP_RDAT_SUB, data, LEN_OTP_RDAT);
	// end read mode
	writeByte(OTP_IF, OTP_CTRL_SUB, 0x00);
}

// Helper to set a single register
void DW1000Class::writeByte(byte cmd, uint16_t offset, byte data) {
	writeBytes(cmd, offset, &data, 1);
}

/*
 * Write bytes to the DW1000. Single bytes can be written to registers via sub-addressing.
 * @param cmd
 * 		寄存器地址
 * 		The register address (see Chapter 7 in the DW1000 user manual).
 * @param offset
 *		The offset to select register sub-parts for writing, or 0x00 to disable
 * 		sub-adressing.
 * @param data
 *		The data array to be written.
 * @param data_size
 * 		字节数量
 *		The number of bytes to be written (take care not to go out of bounds of
 * 		the register).
 */
void DW1000Class::writeBytes(byte cmd, uint16_t offset, byte data[], uint16_t data_size) {
	// TODO offset really bigger than byte?
	byte header[3];
	uint8_t  headerLen = 1;
	uint16_t  i = 0;
	
	// TODO proper error handling: address out of bounds
	// build SPI header
	if(offset == NO_SUB) {
		header[0] = WRITE | cmd;
	} else {
		header[0] = WRITE_SUB | cmd;
		if(offset < 128) {
			header[1] = (byte)offset;
			headerLen++;
		} else {
			header[1] = RW_SUB_EXT | (byte)offset;
			header[2] = (byte)(offset >> 7);
			headerLen += 2;
		}
	}
	SPI.beginTransaction(*_currentSPI);
	digitalWrite(_ss, LOW);
	for(i = 0; i < headerLen; i++) {
		SPI.transfer(header[i]); // send header
	}
	for(i = 0; i < data_size; i++) {
		SPI.transfer(data[i]); // write values
	}
	delayMicroseconds(5);
	digitalWrite(_ss, HIGH);
	SPI.endTransaction();
}


void DW1000Class::getPrettyBytes(byte data[], char msgBuffer[], uint16_t n) {
	uint16_t i, j, b;
	b     = sprintf(msgBuffer, "Data, bytes: %d\nB: 7 6 5 4 3 2 1 0\n", n); // TODO - type
	for(i = 0; i < n; i++) {
		byte curByte = data[i];
		snprintf(&msgBuffer[b++], 2, "%d", (i+1));
		msgBuffer[b++] = (char)((i+1) & 0xFF);
		msgBuffer[b++] = ':';
		msgBuffer[b++] = ' ';
		for(j = 0; j < 8; j++) {
			msgBuffer[b++] = ((curByte >> (7-j)) & 0x01) ? '1' : '0';
			if(j < 7) {
				msgBuffer[b++] = ' ';
			} else if(i < n-1) {
				msgBuffer[b++] = '\n';
			} else {
				msgBuffer[b++] = '\0';
			}
		}
	}
	msgBuffer[b++] = '\0';
}

void DW1000Class::getPrettyBytes(byte cmd, uint16_t offset, char msgBuffer[], uint16_t n) {
	uint16_t i, j, b;
	byte* readBuf = (byte*)malloc(n);
	readBytes(cmd, offset, readBuf, n);
	b     = sprintf(msgBuffer, "Reg: 0x%02x, bytes: %d\nB: 7 6 5 4 3 2 1 0\n", cmd, n);  // TODO - tpye
	for(i = 0; i < n; i++) {
		byte curByte = readBuf[i];
		snprintf(&msgBuffer[b++], 2, "%d", (i+1));
		msgBuffer[b++] = (char)((i+1) & 0xFF);
		msgBuffer[b++] = ':';
		msgBuffer[b++] = ' ';
		for(j = 0; j < 8; j++) {
			msgBuffer[b++] = ((curByte >> (7-j)) & 0x01) ? '1' : '0';
			if(j < 7) {
				msgBuffer[b++] = ' ';
			} else if(i < n-1) {
				msgBuffer[b++] = '\n';
			} else {
				msgBuffer[b++] = '\0';
			}
		}
	}
	msgBuffer[b++] = '\0';
	free(readBuf);
}
