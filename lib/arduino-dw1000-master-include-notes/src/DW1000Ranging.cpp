/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net> and Leopold Sayous <leosayous@gmail.com>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file DW1000Ranging.h
 * Arduino global library (source file) working with the DW1000 library 
 * for the Decawave DW1000 UWB transceiver IC.
 *
 * @TODO
 * - remove or debugmode for Serial.print
 * - move strings to flash to reduce ram usage
 * - do not safe duplicate of pin settings
 * - maybe other object structure
 * - use enums instead of preprocessor constants
 */


#include "DW1000Ranging.h"
#include "DW1000Device.h"


DW1000RangingClass DW1000Ranging;

// 用于处理多个TAG请求
byte DW1000RangingClass::_currentGrantAddress[2];
boolean DW1000RangingClass::_isCommunicating = false;

/*
队列的使用
push(val): 将元素插入队尾。
pop(): 移除队首元素（不返回被删元素）。
front(): 返回队首元素的引用。

注意！！！队列里的元素是array而不是原生数组！！！
*/
std::deque<std::array<byte, 2>> DW1000RangingClass::_requestAddress;

byte DW1000RangingClass::_expectAddress[2];


// 标识主机
boolean DW1000RangingClass::_isHost = false;

// delaytime after request denied
u_int16_t DW1000RangingClass::_requestDeniedDelay;

//other devices we are going to communicate with which are on our network:
DW1000Device DW1000RangingClass::_networkDevices[MAX_DEVICES];
byte         DW1000RangingClass::_currentAddress[8];
byte         DW1000RangingClass::_currentShortAddress[2];
byte         DW1000RangingClass::_lastSentToShortAddress[2];
volatile uint8_t DW1000RangingClass::_networkDevicesNumber = 0; // TODO short, 8bit?
int16_t      DW1000RangingClass::_lastDistantDevice    = 0; // TODO short, 8bit?
DW1000Mac    DW1000RangingClass::_globalMac;

//module type (anchor or tag)
int16_t      DW1000RangingClass::_type; // TODO enum??

// message flow state 在接收信息的时候这个东西看上去是用来校验传输是否错误的 需要调整loop中的逻辑
volatile byte    DW1000RangingClass::_expectedMsgId;

// range filter
volatile boolean DW1000RangingClass::_useRangeFilter = false;
uint16_t DW1000RangingClass::_rangeFilterValue = 15;

// message sent/received state
volatile boolean DW1000RangingClass::_sentAck     = false;
volatile boolean DW1000RangingClass::_receivedAck = false;

// protocol error state
boolean          DW1000RangingClass::_protocolFailed = false;

// timestamps to remember
int32_t            DW1000RangingClass::timer           = 0;
int16_t            DW1000RangingClass::counterForBlink = 0; // TODO 8 bit?


// data buffer
byte          DW1000RangingClass::data[LEN_DATA];
// reset line to the chip
uint8_t   DW1000RangingClass::_RST;
uint8_t   DW1000RangingClass::_SS;
// watchdog and reset period
uint32_t  DW1000RangingClass::_lastActivity;
uint32_t  DW1000RangingClass::_resetPeriod;
// reply times (same on both sides for symm. ranging)
uint16_t  DW1000RangingClass::_replyDelayTimeUS;
//timer delay
uint16_t  DW1000RangingClass::_timerDelay;
// ranging counter (per second)
uint16_t  DW1000RangingClass::_successRangingCount = 0;
uint32_t  DW1000RangingClass::_rangingCountPeriod  = 0;
//Here our handlers
void (* DW1000RangingClass::_handleNewRange)(void) = 0;
void (* DW1000RangingClass::_handleBlinkDevice)(DW1000Device*) = 0;
void (* DW1000RangingClass::_handleNewDevice)(DW1000Device*) = 0;
void (* DW1000RangingClass::_handleInactiveDevice)(DW1000Device*) = 0;

/* ###########################################################################
 * #### Init and end #######################################################
 * ######################################################################### */

//init
void DW1000RangingClass::initCommunication(uint8_t myRST, uint8_t mySS, uint8_t myIRQ) {
	// reset line to the chip
	_RST              = myRST;
	_SS               = mySS;
	_resetPeriod      = DEFAULT_RESET_PERIOD;
	// reply times (same on both sides for symm. ranging)
	_replyDelayTimeUS = DEFAULT_REPLY_DELAY_TIME;
	//we set our timer delay
	_timerDelay       = DEFAULT_TIMER_DELAY;
	
	
	DW1000.begin(myIRQ, myRST);
	DW1000.select(mySS);
}

// 配置短地址 网络id 设备模式
void DW1000RangingClass::configureNetwork(uint16_t deviceAddress, uint16_t networkId, const byte mode[]) {
	// general configuration
	DW1000.newConfiguration();
	DW1000.setDefaults();
	DW1000.setDeviceAddress(deviceAddress);
	DW1000.setNetworkId(networkId);
	DW1000.enableMode(mode);
	DW1000.commitConfiguration();
	
}

void DW1000RangingClass::generalStart() {
	// attach callback for (successfully) sent and received messages
	DW1000.attachSentHandler(handleSent);
	DW1000.attachReceivedHandler(handleReceived);
	// anchor starts in receiving mode, awaiting a ranging poll message
	
	
	if(DEBUG) {
		// DEBUG monitoring
		Serial.println("DW1000-arduino");
		// initialize the driver
		
		
		Serial.println("configuration..");
		// DEBUG chip info and registers pretty printed
		char msg[90];
		DW1000.getPrintableDeviceIdentifier(msg);
		Serial.print("Device ID: ");
		Serial.println(msg);
		DW1000.getPrintableExtendedUniqueIdentifier(msg);
		Serial.print("Unique ID: ");
		Serial.print(msg);
		char string[6];
		sprintf(string, "%02X:%02X", _currentShortAddress[0], _currentShortAddress[1]);
		Serial.print(" short: ");
		Serial.println(string);
		
		DW1000.getPrintableNetworkIdAndShortAddress(msg);
		Serial.print("Network ID & Device Address: ");
		Serial.println(msg);
		DW1000.getPrintableDeviceMode(msg);
		Serial.print("Device mode: ");
		Serial.println(msg);
	}
	
	
	// anchor starts in receiving mode, awaiting a ranging poll message
	receiver();
	// for first time ranging frequency computation
	_rangingCountPeriod = millis(); // 从esp32获取毫秒数
}

/*
 * start as anchor 和 start as tag 几乎是一样的 只是type不同
 *@param address 设备EUI地址
 *@param mode 设备模式
 *@param randomShortAddress 是否随机生成短地址
*/
void DW1000RangingClass::startAsAnchor(char address[], const byte mode[], const bool randomShortAddress) {
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("device address: ");
	Serial.println(address);

	// 设置短地址
	if (randomShortAddress) {
		//we need to define a random short address:
		randomSeed(analogRead(0));
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else {
		// we use first two bytes in addess for short address
		_currentShortAddress[0] = _currentAddress[0];
		_currentShortAddress[1] = _currentAddress[1];
	}
	
	//we configur the network for mac filtering
	//(device Address, network ID, frequency)
	DW1000Ranging.configureNetwork(_currentShortAddress[0]*256+_currentShortAddress[1], 0xDECA, mode);
	
	//general start:
	generalStart();
	
	//defined type as anchor
	_type = ANCHOR;
	
	Serial.println("### ANCHOR ###");
	
}

/*
 * start as anchor 和 start as tag 几乎是一样的 只是type不同
 * 调用generalstart来接收数据
 *@param address 设备EUI地址
 *@param mode 设备模式
 *@param randomShortAddress 是否随机生成短地址
*/
void DW1000RangingClass::startAsTag(char address[], const byte mode[], const bool randomShortAddress) {
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("device address: ");
	Serial.println(address);
	if (randomShortAddress) {
		//we need to define a random short address:
		randomSeed(analogRead(0));
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else {
		// we use first two bytes in addess for short address
		_currentShortAddress[0] = _currentAddress[0];
		_currentShortAddress[1] = _currentAddress[1];
	}
	
	//we configur the network for mac filtering
	//(device Address, network ID, frequency)
	DW1000Ranging.configureNetwork(_currentShortAddress[0]*256+_currentShortAddress[1], 0xDECA, mode);
	
	generalStart();
	//defined type as tag
	_type = TAG;
	
	Serial.println("### TAG ###");
}

/*
* 一定要保证地址和短地址都不一样
*/
boolean DW1000RangingClass::addNetworkDevices(DW1000Device* device, boolean shortAddress) {
	boolean   addDevice = true;
	//we test our network devices array to check
	//we don't already have it
	for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
		if(_networkDevices[i].isAddressEqual(device) && !shortAddress) {
			//the device already exists
			addDevice = false;
			return false;
		}
		else if(_networkDevices[i].isShortAddressEqual(device) && shortAddress) {
			//the device already exists
			addDevice = false;
			return false;
		}
		
	}
	
	if(addDevice) {
		device->setRange(0);
		memcpy(&_networkDevices[_networkDevicesNumber], device, sizeof(DW1000Device));
		_networkDevices[_networkDevicesNumber].setIndex(_networkDevicesNumber);
		_networkDevicesNumber++;
		return true;
	}
	
	return false;
}

/*
* 一定要保证地址和短地址都不一样
*/
boolean DW1000RangingClass::addNetworkDevices(DW1000Device* device) {
	boolean addDevice = true;
	//we test our network devices array to check
	//we don't already have it
	for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
		if(_networkDevices[i].isAddressEqual(device) && _networkDevices[i].isShortAddressEqual(device)) {
			//the device already exists
			addDevice = false;
			return false;
		}
		
	}
	
	if(addDevice) {
		if(_type == ANCHOR) //for now let's start with 1 TAG
		{
			_networkDevicesNumber = 0;
		}
		memcpy(&_networkDevices[_networkDevicesNumber], device, sizeof(DW1000Device));
		_networkDevices[_networkDevicesNumber].setIndex(_networkDevicesNumber);
		_networkDevicesNumber++;
		return true;
	}
	
	return false;
}

void DW1000RangingClass::removeNetworkDevices(int16_t index) {
	//if we have just 1 element
	if(_networkDevicesNumber == 1) {
		_networkDevicesNumber = 0;
	}
	else if(index == _networkDevicesNumber-1) //if we delete the last element
	{
		_networkDevicesNumber--;
	}
	else {
		//we translate all the element wich are after the one we want to delete.
		for(int16_t i = index; i < _networkDevicesNumber-1; i++) { // TODO 8bit?
			memcpy(&_networkDevices[i], &_networkDevices[i+1], sizeof(DW1000Device));
			_networkDevices[i].setIndex(i);
		}
		_networkDevicesNumber--;
	}
}

/* ###########################################################################
 * #### Setters and Getters ##################################################
 * ######################################################################### */

//setters
void DW1000RangingClass::setReplyTime(uint16_t replyDelayTimeUs) { _replyDelayTimeUS = replyDelayTimeUs; }

void DW1000RangingClass::setResetPeriod(uint32_t resetPeriod) { _resetPeriod = resetPeriod; }

//通过短地址寻找 dw1000device 找不到就返回空指针
//在存储器寻找
DW1000Device* DW1000RangingClass::searchDistantDevice(byte shortAddress[]) {
	//we compare the 2 bytes address with the others
	for(uint16_t i = 0; i < _networkDevicesNumber; i++) { // TODO 8bit?
		if(memcmp(shortAddress, _networkDevices[i].getByteShortAddress(), 2) == 0) {
			//we have found our device !
			return &_networkDevices[i];
		}
	}
	
	return nullptr;
}

//获取最后一个通信的设备
DW1000Device* DW1000RangingClass::getDistantDevice() {
	//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
	
	return &_networkDevices[_lastDistantDevice];
	
}


/* ###########################################################################
 * #### Public methods #######################################################
 * ######################################################################### */

 //标记为active
void DW1000RangingClass::checkForReset() {
	uint32_t curMillis = millis();
	if(!_sentAck && !_receivedAck) {
		// check if inactive
		if(curMillis-_lastActivity > _resetPeriod) {//reset period 设置为200
			resetInactive(); //更新active
		}
		return; // TODO cc
	}
}

// 查看是否有inactive device
void DW1000RangingClass::checkForInactiveDevices() {
	for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
		if(_networkDevices[i].isInactive()) {
			if(_handleInactiveDevice != 0) {
				(*_handleInactiveDevice)(&_networkDevices[i]);
			}
			//we need to delete the device from the array:
			removeNetworkDevices(i);
			
		}
	}
}

// TODO check return type
int16_t DW1000RangingClass::detectMessageType(byte datas[]) {
	if(datas[0] == FC_1_BLINK) {
		return BLINK;
	}
	else if(datas[0] == FC_1 && datas[1] == FC_2) {
		//we have a long MAC frame message (ranging init)
		return datas[LONG_MAC_LEN];
	}
	else if(datas[0] == FC_1 && datas[1] == FC_2_SHORT) {
		//we have a short mac frame message (poll, range, range report, etc..)
		//and range request, grant
		return datas[SHORT_MAC_LEN];
	}
}


	/*
	TAG                            ANCHOR

 |                                |
 |───BLINK───────────────────────>|  ① 设备发现  ANCHOR发现了TAG并记录 添加这个TAG设备

 |                                |
 |<──RANGING_INIT─────────────────|  ② 连接初始化  在receiveack中 ANCHOR发现了BLINK帧之后 发送RANGING_INIT 并且添加ANCHOR设备

                                    下面的流程 都塞到receivedack中的一个else里面同意处理了

request:
|todo----RANGING_REQUEST-------->|  tag请求测距
|<------GRANT----------------|  host回复测距请求 并且回复到所有ANCHOR设备

ranging_request看上去不需要添加sentack


 |───POLL────────────────────────>|  ③ 启动测距（含timePollSent）  给设备列表中的每一个设备都记录timePollSent
 |                                |
 |<──POLL_ACK─────────────────────|  ④ 回复ACK（含timePollReceived）
 |                                |
 |───RANGE───────────────────────>|  ⑤ 请求测距结果（含timePollAckReceived）
 |                                |
 |<──RANGING_REPORT───────────────|  ⑥ 返回最终距离（含timePollAckSent/timeRangeReceived）
 
 timeRangeSent

	*/
void DW1000RangingClass::loop() {


	//we check if needed to reset !
	checkForReset();

	//millis()表示从开机到现在的毫秒数 从esp32获取
	uint32_t time = millis(); // TODO other name - too close to "timer"
	if(time-timer > _timerDelay) {//timerdelay=80  这里看起来是80ms后tick一次
		timer = time;
		timerTick(); //tag在这个地方发送POLL数据（广播）  并且每20个tick发送一次信标，相当于1600ms发送一次
		//其余的tick，都在发送poll帧（如果本地已经记录设备）
	}
	
	if(_sentAck) { //发送了数据
		_sentAck = false;
		
		// TODO cc
		int messageType = detectMessageType(data);
		
		//这里还需要考虑range request和grant

		if(messageType != POLL_ACK && messageType != POLL && messageType != RANGE && messageType!=GRANT)
			//we have a range report message or error message 
			
			// or range request
			// grant

			return;
		
		//A msg was sent. We launch the ranging protocole when a message was sent
		if(_type == ANCHOR) {
			if(messageType == POLL_ACK) {
				//本次通信的设备
				DW1000Device* myDistantDevice = searchDistantDevice(_lastSentToShortAddress);
				
				if (myDistantDevice) {
					DW1000.getTransmitTimestamp(myDistantDevice->timePollAckSent);
				}
			}else if(messageType == GRANT) {
				if (_isHost){
					//host 发送了grant
					if(_isCommunicating){
						if(_requestAddress.size() > 0){
							std::array<byte, 2> address_array = _requestAddress.front();
							_requestAddress.pop_front();
							//将这个设备的shortaddress 赋值给currentgrantaddress
							_currentGrantAddress[0] = address_array[0];
							_currentGrantAddress[1] = address_array[1];
							_isCommunicating = true;

						}else{
							_isCommunicating = false;
						}
					}else{
						Serial.println("error:没有可以通信的设备");
					}


				}else{
					Serial.println("error:不是主机的设备错误发送了GRANT帧");
				}
			}
		}

		else if(_type == TAG) {
			if(messageType == POLL) {
				DW1000Time timePollSent;
				DW1000.getTransmitTimestamp(timePollSent);
				//if the last device we send the POLL is broadcast:
				if(_lastSentToShortAddress[0] == 0xFF && _lastSentToShortAddress[1] == 0xFF) {
					//we save the value for all the devices !
					//这是一个广播的轮询信号
					for(uint16_t i = 0; i < _networkDevicesNumber; i++) {
						_networkDevices[i].timePollSent = timePollSent;
					}
				}
				else {//not a broatcast
					//we search the device associated with the last send address
					DW1000Device* myDistantDevice = searchDistantDevice(_lastSentToShortAddress);
					//we save the value just for one device
					if (myDistantDevice) {
						myDistantDevice->timePollSent = timePollSent;
					}
				}
			}
			else if(messageType == RANGE) { //同上pool 逻辑是一样的
				DW1000Time timeRangeSent;
				DW1000.getTransmitTimestamp(timeRangeSent);
				//if the last device we send the POLL is broadcast:
				if(_lastSentToShortAddress[0] == 0xFF && _lastSentToShortAddress[1] == 0xFF) {
					//we save the value for all the devices !
					for(uint16_t i = 0; i < _networkDevicesNumber; i++) {
						_networkDevices[i].timeRangeSent = timeRangeSent;
					}
				}
				else {
					//we search the device associated with the last send address
					DW1000Device* myDistantDevice = searchDistantDevice(_lastSentToShortAddress);
					//we save the value just for one device
					if (myDistantDevice) {
						myDistantDevice->timeRangeSent = timeRangeSent;
					}
				}
				
			}
		}
		
	}
	
	//check for new received message
	if(_receivedAck) {
		_receivedAck = false;
		
		//we read the datas from the modules:
		// get message and parse
		DW1000.getData(data, LEN_DATA);
		
		int messageType = detectMessageType(data);
		
		//下面两个是关于初始化的
		//we have just received a BLINK message from tag
		if(messageType == BLINK && _type == ANCHOR) {//ANCHOR收到了BLINK 并发送RANGING_INIT
			byte address[8];
			byte shortAddress[2];
			_globalMac.decodeBlinkFrame(data, address, shortAddress); //将收到的BLINK帧解析并存入address和shortaddress
			
			//we crate a new device with th tag
			DW1000Device myTag(address, shortAddress);
			
			if(addNetworkDevices(&myTag)) {

				// 对mytag执行中断函数
				//这个函数需要用户进行自定义 库文件里面只是留出了这个接口
				if(_handleBlinkDevice != 0) {
					(*_handleBlinkDevice)(&myTag);
				}
				//we reply by the transmit ranging init message
				transmitRangingInit(&myTag);
				noteActivity();
			}
			_expectedMsgId = POLL; //anchor在收到BLINK之后希望收到POLL 
		}

		else if(messageType == RANGING_INIT && _type == TAG) { //tag接收到RANGING_INIT
			
			byte address[2];
			_globalMac.decodeLongMACFrame(data, address); //解析
			//we crate a new device with the anchor
			DW1000Device myAnchor(address, true);
			
			if(addNetworkDevices(&myAnchor, true)) {//同上，中断需要自定义
				if(_handleNewDevice != 0) {
					(*_handleNewDevice)(&myAnchor);
				}
			}
			
			noteActivity();
		}
		else {
			//解析short mac layer frame
			//we have a short mac layer frame !
			byte address[2]; //source address
			_globalMac.decodeShortMACFrame(data, address);
			
			//解析mac帧 并提取数据和source address
			
			//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
			DW1000Device* myDistantDevice = searchDistantDevice(address);
			
			//什么都没发生 设备不存在 或者在本地中找不到这个设备（可能是通信发生了错误） 所以直接return了
			if((_networkDevicesNumber == 0) || (myDistantDevice == nullptr)) {
				//we don't have the short address of the device in memory
				if (DEBUG) {
					Serial.println("Not found");
					/*
					Serial.print("unknown: ");
					Serial.print(address[0], HEX);
					Serial.print(":");
					Serial.println(address[1], HEX);
					*/
				}
				return;
			}
			
			
			//then we proceed to range protocole
			if(_type == ANCHOR) {

				//校验错误 通信失败
				if(messageType != _expectedMsgId) {
					// unexpected message, start over again (except if already POLL)
					_protocolFailed = true;
				}
				
				if(messageType == RANGING_REQUEST && _isHost == true) {
					//host receive a range request
					//we need to reply with a grant

					if(_isCommunicating==false){
						//memcpy(_lastSentToShortAddress, address, 2);
						memcpy(_currentGrantAddress, address, 2);
						_isCommunicating = true;
					}
					else{
						//_isCommunicating==true
						std::array<byte, 2> shortAddress;
						memcpy(shortAddress.data(), address, 2);
						// 把这个地址加入到requestAddress队列中

						//如果这个地址已经在requestAddress队列中了
						//那么就不添加了
						for(uint16_t i = 0; i < _requestAddress.size(); i++) {
							if(_requestAddress[i][0] == shortAddress[0] && _requestAddress[i][1] == shortAddress[1]) {
								return;
							}
						}

						_requestAddress.push_back(shortAddress);
					}

					transmitGrant(myDistantDevice);
					
					_expectedMsgId = POLL;

					noteActivity();
					return;

				}
				else if(messageType == GRANT && _isHost == false) {
					// anchor receive a grant frame

					//其他ANCHOR在这段时间内只接收这个地址的信息
					memcpy(_expectAddress, data+SHORT_MAC_LEN+4, 2);
					
					_expectedMsgId = POLL;

					noteActivity();
					return;

				}

				else if(messageType == POLL) { //接收到POLL信号
					//we receive a POLL which is a broacast message
					//we need to grab info about it
					
					// 提取POLL帧中包含的设备数量
					int16_t numberDevices = 0;
					memcpy(&numberDevices, data+SHORT_MAC_LEN+1, 1);

					
					for(uint16_t i = 0; i < numberDevices; i++) {
						//we need to test if this value is for us:
						//we grab the mac address of each devices:
						byte shortAddress[2];
						memcpy(shortAddress, data+SHORT_MAC_LEN+2+i*4, 2);
						// 怪不得transmitpoll函数的注释说这是不是对的 emmm还要多看看 好像这是个广播信号
						//到时候跑不起来再看吧
						//POLL帧发送的仅仅是单个设备的话，那么这里提取短地址的时候可能会直接拿到replytime从而发生错误

						//we test if the short address is our address
						if(shortAddress[0] == _currentShortAddress[0] && shortAddress[1] == _currentShortAddress[1]) {
							//we grab the replytime wich is for us
							uint16_t replyTime;
							memcpy(&replyTime, data+SHORT_MAC_LEN+2+i*4+2, 2);
							//we configure our replyTime;
							_replyDelayTimeUS = replyTime;
							
							// on POLL we (re-)start, so no protocol failure
							_protocolFailed = false;
							
							DW1000.getReceiveTimestamp(myDistantDevice->timePollReceived);
							//we note activity for our device:
							myDistantDevice->noteActivity();
							//we indicate our next receive message for our ranging protocole
							_expectedMsgId = RANGE;
							transmitPollAck(myDistantDevice);
							noteActivity();
							
							return;
						}
						
					}
					
					
				}
				else if(messageType == RANGE) {
					//we receive a RANGE which is a broacast message
					//we need to grab info about it
					uint8_t numberDevices = 0;
					memcpy(&numberDevices, data+SHORT_MAC_LEN+1, 1);
					//获取设备数量
					
					for(uint8_t i = 0; i < numberDevices; i++) {
						//we need to test if this value is for us:
						//we grab the mac address of each devices:
						byte shortAddress[2];
						memcpy(shortAddress, data+SHORT_MAC_LEN+2+i*17, 2);
						
						//we test if the short address is our address
						if(shortAddress[0] == _currentShortAddress[0] && shortAddress[1] == _currentShortAddress[1]) {
							//we grab the replytime wich is for us
							DW1000.getReceiveTimestamp(myDistantDevice->timeRangeReceived);
							noteActivity();
							_expectedMsgId = POLL;
							
							if(!_protocolFailed) {
								
								myDistantDevice->timePollSent.setTimestamp(data+SHORT_MAC_LEN+4+17*i);
								myDistantDevice->timePollAckReceived.setTimestamp(data+SHORT_MAC_LEN+9+17*i);
								myDistantDevice->timeRangeSent.setTimestamp(data+SHORT_MAC_LEN+14+17*i);
								
								// (re-)compute range as two-way ranging is done
								DW1000Time myTOF;
								computeRangeAsymmetric(myDistantDevice, &myTOF); // CHOSEN RANGING ALGORITHM
								
								float distance = myTOF.getAsMeters();
								
								if (_useRangeFilter) {
									//Skip first range
									if (myDistantDevice->getRange() != 0.0f) {
										distance = filterValue(distance, myDistantDevice->getRange(), _rangeFilterValue);
									}
								}
								
								myDistantDevice->setRXPower(DW1000.getReceivePower());
								myDistantDevice->setRange(distance);
								
								myDistantDevice->setFPPower(DW1000.getFirstPathPower());
								myDistantDevice->setQuality(DW1000.getReceiveQuality());
								
								//we send the range to TAG
								transmitRangeReport(myDistantDevice);
								
								//we have finished our range computation. We send the corresponding handler
								_lastDistantDevice = myDistantDevice->getIndex();
								if(_handleNewRange != 0) {
									(*_handleNewRange)();
								}
								
							}
							else {
								transmitRangeFailed(myDistantDevice);
							}
							
							
							return;
						}
						
					}
					
					
				}
			}
			else if(_type == TAG) {
				// get message and parse
				if(messageType != _expectedMsgId) {
					// unexpected message, start over again
					//not needed ?
					return;
					_expectedMsgId = POLL_ACK;
					return;
				}
				
				if(messageType == GRANT){
					//grant frame
					byte shortaddress[2];
					memcpy(shortaddress, data+SHORT_MAC_LEN+4, 2);
					if(shortaddress[0]==_currentShortAddress[0] && shortaddress[1]==_currentShortAddress[1]){
						//we have a grant frame for us
						_expectedMsgId = POLL_ACK;
						transmitPoll(nullptr);
					}else{
						//for others

						u_int16_t delaytime;

						memcpy(&delaytime, data+SHORT_MAC_LEN+2, 2);

						//读取设备数量
						u_int8_t device_number;
						memcpy(&device_number, data+SHORT_MAC_LEN+1, 1);
						

						//获取当前grant帧中的请求队列
						_requestAddress.clear();
						for(uint8_t i=0; i<device_number-1; i++){

							std::array<byte, 2> shortaddress;
							memcpy(shortaddress.data(), data+SHORT_MAC_LEN+6+4*i, 2);
							_requestAddress.push_back(shortaddress);
						}

						//问题来了 我并不知道此时的host是否接收到原来的request
						//所以GRANT帧里面要加东西了
						
						// todo 需要做一个延时的变量 类似activity
						
						_requestDeniedDelay = delaytime;


					}

				}

				if(messageType == POLL_ACK) {
					DW1000.getReceiveTimestamp(myDistantDevice->timePollAckReceived);
					//we note activity for our device:
					myDistantDevice->noteActivity();
					
					//in the case the message come from our last device:
					if(myDistantDevice->getIndex() == _networkDevicesNumber-1) {
						_expectedMsgId = RANGE_REPORT;
						//and transmit the next message (range) of the ranging protocole (in broadcast)
						transmitRange(nullptr);
					}
				}
				else if(messageType == RANGE_REPORT) {
					
					float curRange;
					memcpy(&curRange, data+1+SHORT_MAC_LEN, 4);
					float curRXPower;
					memcpy(&curRXPower, data+5+SHORT_MAC_LEN, 4);
					
					if (_useRangeFilter) {
						//Skip first range
						if (myDistantDevice->getRange() != 0.0f) {
							curRange = filterValue(curRange, myDistantDevice->getRange(), _rangeFilterValue);
						}
					}

					//we have a new range to save !
					myDistantDevice->setRange(curRange);
					myDistantDevice->setRXPower(curRXPower);
					
					
					//We can call our handler !
					//we have finished our range computation. We send the corresponding handler
					_lastDistantDevice = myDistantDevice->getIndex();
					if(_handleNewRange != 0) {
						(*_handleNewRange)();
					}
				}
				else if(messageType == RANGE_FAILED) {
					//not needed as we have a timer;
					return;
					_expectedMsgId = POLL_ACK;
				}
			}
		}
		
	}
}

void DW1000RangingClass::useRangeFilter(boolean enabled) {
	_useRangeFilter = enabled;
}

void DW1000RangingClass::setRangeFilterValue(uint16_t newValue) {
	if (newValue < 2) {
		_rangeFilterValue = 2;
	}else{
		_rangeFilterValue = newValue;
	}
}


/* ###########################################################################
 * #### Private methods and Handlers for transmit & Receive reply ############
 * ######################################################################### */


void DW1000RangingClass::handleSent() {
	// status change on sent success
	_sentAck = true;
}

void DW1000RangingClass::handleReceived() {
	// status change on received success
	_receivedAck = true;
}

//更新本设备的活动时间戳
void DW1000RangingClass::noteActivity() {
	// update activity timestamp, so that we do not reach "resetPeriod"
	_lastActivity = millis();
}

void DW1000RangingClass::resetInactive() {
	//if inactive
	if(_type == ANCHOR) {
		_expectedMsgId = POLL;
		receiver();
	}
	noteActivity();
}

void DW1000RangingClass::timerTick() {
	if(_networkDevicesNumber > 0 && counterForBlink != 0) {
		if(_type == TAG) { //发送poll 期待收到pollack
			_expectedMsgId = POLL_ACK;
			//send a prodcast poll
			transmitPoll(nullptr);
		}
	}

	// 发送信标
	else if(counterForBlink == 0) {
		if(_type == TAG) {
			//发送信标 发送自己的short address和EUI
			transmitBlink();
		}
		//check for inactive devices if we are a TAG or ANCHOR
		checkForInactiveDevices(); // 移除不活动的设备
	}

	counterForBlink++;

	// 每20个tick发送一次信标
	if(counterForBlink > 20) {
		counterForBlink = 0;
	}
}


void DW1000RangingClass::copyShortAddress(byte address1[], byte address2[]) {
	*address1     = *address2;
	*(address1+1) = *(address2+1);
}

/* ###########################################################################
 * #### Methods for ranging protocole   ######################################
 * ######################################################################### */

void DW1000RangingClass::transmitInit() {
	DW1000.newTransmit();
	DW1000.setDefaults();
}


void DW1000RangingClass::transmit(byte datas[]) {
	DW1000.setData(datas, LEN_DATA);
	DW1000.startTransmit();
}


void DW1000RangingClass::transmit(byte datas[], DW1000Time time) {
	DW1000.setDelay(time);
	DW1000.setData(data, LEN_DATA);
	DW1000.startTransmit();
}

void DW1000RangingClass::transmitBlink() {
	transmitInit();
	_globalMac.generateBlinkFrame(data, _currentAddress, _currentShortAddress);
	transmit(data);
}

//向传入括号的目标发送类型为ranginginit的消息
void DW1000RangingClass::transmitRangingInit(DW1000Device* myDistantDevice) {
	transmitInit();
	//we generate the mac frame for a ranging init message
	_globalMac.generateLongMACFrame(data, _currentShortAddress, myDistantDevice->getByteAddress());
	//we define the function code
	data[LONG_MAC_LEN] = RANGING_INIT;
	
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	
	transmit(data);
}

//下面两个是我自己写的

/*
给host发送ranging request
host地址为0xAA 0xAA
*/
void DW1000RangingClass::transmitRangingRequest() {
	/*
	ranging request也是一个广播信号，给ANCHOR发送 或者只是给HOST发送？
	*/
	transmitInit();	

	// 下面生成的是标准的short mac frame

	byte host_address[2] = {0xAA,0xAA};

	_globalMac.generateShortMACFrame(data, _currentShortAddress, host_address);

	data[SHORT_MAC_LEN]   = RANGING_REQUEST;

	// 接下来要发送的是自定义的一些数据
	// 暂时还不知道说些什么

	//data[SHORT_MAC_LEN+1] = 1; // 1 device

	copyShortAddress(_lastSentToShortAddress, host_address);



	transmit(data);
}

/*
grant为host发送的同意请求消息
看来还需要定义一个类似队列的东西来安排设备的顺序

*/
void DW1000RangingClass::transmitGrant(DW1000Device* myDistantDevice) {
	transmitInit();

	byte shortbroadcast[2] = {0xFF, 0xFF};
	_globalMac.generateShortMACFrame(data, _currentShortAddress, shortbroadcast);
	
	data[SHORT_MAC_LEN] = GRANT;

	// 接下来要发送的是自定义的一些数据

	// 此时许可的tag+队列中的tag数量
	u_int8_t number_devices = 1+_requestAddress.size();
	memcpy(data+SHORT_MAC_LEN+1, &number_devices, 1); 
	
	// 时隙长度（单位ms）
	uint16_t slot_length = DEFAULT_SLOT_LENGTH;
	memcpy(data+SHORT_MAC_LEN+2, &slot_length, 2);

	// 许可的tag地址
	memcpy(data+SHORT_MAC_LEN+4, myDistantDevice->getByteShortAddress(), 2);

	for(int i = 0; i < _requestAddress.size(); i++){


		byte address[2];

		memcpy(address, _requestAddress[i].data(), 2);

		memcpy(data+SHORT_MAC_LEN+6+2*i, address, 2);
		

	}
	//许可的tag地址


	transmit(data);
}

// 用来发送一个广播的轮询消息
//如果输入空指针，就向全设备广播
//否则向特定目标发送信号
void DW1000RangingClass::transmitPoll(DW1000Device* myDistantDevice) {
	
	transmitInit();
	
	if(myDistantDevice == nullptr) {
		/*
		data:
		0-8: short mac frame (broadcast) SHORT_MAC_LEN指的仅仅只是这一部分
		9: POLL
		10: number of devices
		11~: short address with reply time
		*/

		//we need to set our timerDelay:
		_timerDelay = DEFAULT_TIMER_DELAY+(uint16_t)(_networkDevicesNumber*3*DEFAULT_REPLY_DELAY_TIME/1000);
		
		byte shortBroadcast[2] = {0xFF, 0xFF};
		_globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
		data[SHORT_MAC_LEN]   = POLL;
		//we enter the number of devices
		data[SHORT_MAC_LEN+1] = _networkDevicesNumber;
		
		for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
			//each devices have a different reply delay time.
			_networkDevices[i].setReplyTime((2*i+1)*DEFAULT_REPLY_DELAY_TIME);
			//we write the short address of our device:
			memcpy(data+SHORT_MAC_LEN+2+4*i, _networkDevices[i].getByteShortAddress(), 2);
			
			//we add the replyTime
			uint16_t replyTime = _networkDevices[i].getReplyTime();
			memcpy(data+SHORT_MAC_LEN+2+2+4*i, &replyTime, 2);
			
		}
		
		copyShortAddress(_lastSentToShortAddress, shortBroadcast);
		
	}
	else {
		//we redefine our default_timer_delay for just 1 device;
		_timerDelay = DEFAULT_TIMER_DELAY;
		
		_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
		
		data[SHORT_MAC_LEN]   = POLL;
		data[SHORT_MAC_LEN+1] = 1; // 1 device
		uint16_t replyTime = myDistantDevice->getReplyTime();
		memcpy(data+SHORT_MAC_LEN+2, &replyTime, sizeof(uint16_t)); // todo is code correct? maybe wrong?
		// 可能是对的 他已经定义了目标 就不需要再后面跟着地址+时间了  前面写地址是因为他是个广播mac帧
		// 这个地方仅仅只有reply时间

		copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	}
	
	transmit(data);
}


void DW1000RangingClass::transmitPollAck(DW1000Device* myDistantDevice) {
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = POLL_ACK;
	// delay the same amount as ranging tag
	DW1000Time deltaTime = DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS);
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data, deltaTime);
}

void DW1000RangingClass::transmitRange(DW1000Device* myDistantDevice) {
	//transmit range need to accept broadcast for multiple anchor
	transmitInit();
	
	if(myDistantDevice == nullptr) {
		//we need to set our timerDelay:
		_timerDelay = DEFAULT_TIMER_DELAY+(uint16_t)(_networkDevicesNumber*3*DEFAULT_REPLY_DELAY_TIME/1000);
		
		byte shortBroadcast[2] = {0xFF, 0xFF};
		_globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
		data[SHORT_MAC_LEN]   = RANGE;
		//we enter the number of devices
		data[SHORT_MAC_LEN+1] = _networkDevicesNumber;
		
		// delay sending the message and remember expected future sent timestamp
		DW1000Time deltaTime     = DW1000Time(DEFAULT_REPLY_DELAY_TIME, DW1000Time::MICROSECONDS);
		DW1000Time timeRangeSent = DW1000.setDelay(deltaTime);
		
		for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
			//we write the short address of our device:
			memcpy(data+SHORT_MAC_LEN+2+17*i, _networkDevices[i].getByteShortAddress(), 2);
			
			
			//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
			_networkDevices[i].timeRangeSent = timeRangeSent;
			_networkDevices[i].timePollSent.getTimestamp(data+SHORT_MAC_LEN+4+17*i);
			_networkDevices[i].timePollAckReceived.getTimestamp(data+SHORT_MAC_LEN+9+17*i);
			_networkDevices[i].timeRangeSent.getTimestamp(data+SHORT_MAC_LEN+14+17*i);
			
		}
		
		copyShortAddress(_lastSentToShortAddress, shortBroadcast);
		
	}
	else {
		_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
		data[SHORT_MAC_LEN] = RANGE;
		// delay sending the message and remember expected future sent timestamp
		DW1000Time deltaTime = DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS);
		//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
		myDistantDevice->timeRangeSent = DW1000.setDelay(deltaTime);
		myDistantDevice->timePollSent.getTimestamp(data+1+SHORT_MAC_LEN);
		myDistantDevice->timePollAckReceived.getTimestamp(data+6+SHORT_MAC_LEN);
		myDistantDevice->timeRangeSent.getTimestamp(data+11+SHORT_MAC_LEN);
		copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	}
	
	
	transmit(data);
}


void DW1000RangingClass::transmitRangeReport(DW1000Device* myDistantDevice) {
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = RANGE_REPORT;
	// write final ranging result
	float curRange   = myDistantDevice->getRange();
	float curRXPower = myDistantDevice->getRXPower();
	//We add the Range and then the RXPower
	memcpy(data+1+SHORT_MAC_LEN, &curRange, 4);
	memcpy(data+5+SHORT_MAC_LEN, &curRXPower, 4);
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data, DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS));
}

void DW1000RangingClass::transmitRangeFailed(DW1000Device* myDistantDevice) {
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = RANGE_FAILED;
	
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data);
}

void DW1000RangingClass::receiver() {
	DW1000.newReceive();
	DW1000.setDefaults();
	// so we don't need to restart the receiver manually
	DW1000.receivePermanently(true);
	DW1000.startReceive();
}


/* ###########################################################################
 * #### Methods for range computation and corrections  #######################
 * ######################################################################### */


void DW1000RangingClass::computeRangeAsymmetric(DW1000Device* myDistantDevice, DW1000Time* myTOF) {
	// asymmetric two-way ranging (more computation intense, less error prone)
	DW1000Time round1 = (myDistantDevice->timePollAckReceived-myDistantDevice->timePollSent).wrap();
	DW1000Time reply1 = (myDistantDevice->timePollAckSent-myDistantDevice->timePollReceived).wrap();
	DW1000Time round2 = (myDistantDevice->timeRangeReceived-myDistantDevice->timePollAckSent).wrap();
	DW1000Time reply2 = (myDistantDevice->timeRangeSent-myDistantDevice->timePollAckReceived).wrap();
	
	myTOF->setTimestamp((round1*round2-reply1*reply2)/(round1+round2+reply1+reply2));
	/*
	Serial.print("timePollAckReceived ");myDistantDevice->timePollAckReceived.print();
	Serial.print("timePollSent ");myDistantDevice->timePollSent.print();
	Serial.print("round1 "); Serial.println((long)round1.getTimestamp());
	
	Serial.print("timePollAckSent ");myDistantDevice->timePollAckSent.print();
	Serial.print("timePollReceived ");myDistantDevice->timePollReceived.print();
	Serial.print("reply1 "); Serial.println((long)reply1.getTimestamp());
	
	Serial.print("timeRangeReceived ");myDistantDevice->timeRangeReceived.print();
	Serial.print("timePollAckSent ");myDistantDevice->timePollAckSent.print();
	Serial.print("round2 "); Serial.println((long)round2.getTimestamp());
	
	Serial.print("timeRangeSent ");myDistantDevice->timeRangeSent.print();
	Serial.print("timePollAckReceived ");myDistantDevice->timePollAckReceived.print();
	Serial.print("reply2 "); Serial.println((long)reply2.getTimestamp());
	 */
}


/* FOR DEBUGGING*/
void DW1000RangingClass::visualizeDatas(byte datas[]) {
	char string[60];
	sprintf(string, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
					datas[0], datas[1], datas[2], datas[3], datas[4], datas[5], datas[6], datas[7], datas[8], datas[9], datas[10], datas[11], datas[12], datas[13], datas[14], datas[15]);
	Serial.println(string);
}



/* ###########################################################################
 * #### Utils  ###############################################################
 * ######################################################################### */

float DW1000RangingClass::filterValue(float value, float previousValue, uint16_t numberOfElements) {
	
	float k = 2.0f / ((float)numberOfElements + 1.0f);
	return (value * k) + previousValue * (1.0f - k);
}



