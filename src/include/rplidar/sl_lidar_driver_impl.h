/*
 *  Slamtec LIDAR SDK
 *
 *  Copyright (c) 2020 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 */
 /*
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  */

#pragma once
#include "sl_lidar_driver.h"

namespace sl {
	class SL_LidarDriver :public ILidarDriver
	{
		public:
			enum {
				LEGACY_SAMPLE_DURATION = 476,
			};

			enum
			{
					NORMAL_CAPSULE = 0,
					DENSE_CAPSULE = 1,
			};

			enum {
				A2A3_LIDAR_MINUM_MAJOR_ID  = 2,
				TOF_LIDAR_MINUM_MAJOR_ID = 6,
			};
		public:
			SL_LidarDriver()
				:_channel(NULL)
				, _isConnected(false)
				, _isScanning(false)
				, _isSupportingMotorCtrl(MotorCtrlSupportNone)
				, _cached_sampleduration_std(LEGACY_SAMPLE_DURATION)
				,_cached_sampleduration_express(LEGACY_SAMPLE_DURATION)
				, _cached_scan_node_hq_count(0)
				, _cached_scan_node_hq_count_for_interval_retrieve(0)
			{}

			sl_result connect(IChannel* channel);
			void disconnect();
			bool isConnected();
			sl_result reset(uint32_t timeoutInMs = DEFAULT_TIMEOUT);
			sl_result getAllSupportedScanModes(std::vector<LidarScanMode>& outModes, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
			sl_result getTypicalScanMode(uint16_t& outMode, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
			sl_result startScan(bool force, bool useTypicalScan, uint32_t options = 0, LidarScanMode* outUsedScanMode = nullptr);
			sl_result startScanNormal(bool force, uint32_t timeout = DEFAULT_TIMEOUT);
			sl_result startScanExpress(bool force, uint16_t scanMode, uint32_t options = 0, LidarScanMode* outUsedScanMode = nullptr, uint32_t timeout = DEFAULT_TIMEOUT);
			sl_result stop(uint32_t timeout = DEFAULT_TIMEOUT);
			DEPRECATED(sl_result grabScanData(sl_lidar_response_measurement_node_t * nodebuffer, size_t& count, uint32_t timeout = DEFAULT_TIMEOUT));
			sl_result grabScanDataHq(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t& count, uint32_t timeout = DEFAULT_TIMEOUT);
			sl_result getDeviceInfo(sl_lidar_response_device_info_t& info, uint32_t timeout = DEFAULT_TIMEOUT);
			sl_result checkMotorCtrlSupport(MotorCtrlSupport & support, uint32_t timeout = DEFAULT_TIMEOUT);
			sl_result getFrequency(const LidarScanMode& scanMode, const sl_lidar_response_measurement_node_hq_t* nodes, size_t count, float& frequency);
			sl_result setLidarIpConf(const sl_lidar_ip_conf_t& conf, uint32_t timeout);
			sl_result getHealth(sl_lidar_response_device_health_t& health, uint32_t timeout = DEFAULT_TIMEOUT);
			sl_result getDeviceMacAddr(uint8_t* macAddrArray, uint32_t timeoutInMs);
			sl_result ascendScanData(sl_lidar_response_measurement_node_t * nodebuffer, size_t count);
			sl_result ascendScanData(sl_lidar_response_measurement_node_hq_t * nodebuffer, size_t count);
			sl_result getScanDataWithIntervalHq(sl_lidar_response_measurement_node_hq_t * nodebuffer, size_t & count);
			sl_result setMotorSpeed(uint16_t speed = DEFAULT_MOTOR_PWM);//
			sl_result negotiateSerialBaudRate(uint32_t requiredBaudRate, uint32_t* baudRateDetected = NULL);
	
	protected:
			sl_result startMotor();
			sl_result checkSupportConfigCommands(bool& outSupport, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
			sl_result getScanModeCount(uint16_t& modeCount, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
			sl_result setLidarConf(uint32_t type, const void* payload, size_t payloadSize, uint32_t timeout);
			sl_result getLidarConf(uint32_t type, std::vector<uint8_t> &outputBuf, const std::vector<uint8_t> &reserve = std::vector<uint8_t>(), uint32_t timeout = DEFAULT_TIMEOUT);
			sl_result getLidarSampleDuration(float& sampleDurationRes, uint16_t scanModeID, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
			sl_result getMaxDistance(float &maxDistance, uint16_t scanModeID, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
			sl_result getScanModeAnsType(uint8_t &ansType, uint16_t scanModeID, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
			sl_result getScanModeName(char* modeName, uint16_t scanModeID, uint32_t timeoutInMs = DEFAULT_TIMEOUT);
			//DEPRECATED(sl_result getSampleDuration_uS(sl_lidar_response_sample_rate_t & rateInfo, uint32_t timeout = DEFAULT_TIMEOUT));
			//DEPRECATED (sl_result checkExpressScanSupported(bool & support, uint32_t timeout = DEFAULT_TIMEOUT));
			//DEPRECATED(sl_result getFrequency(bool inExpressMode, size_t count, float & frequency, bool & is4kmode));
		private:
			sl_result  _sendCommand(uint16_t cmd, const void * payload = NULL, size_t payloadsize = 0 );
			sl_result _waitResponseHeader(sl_lidar_ans_header_t * header, uint32_t timeout = DEFAULT_TIMEOUT);
			template <typename T>
			sl_result _waitResponse(T &payload ,uint8_t ansType, _u32 timeout = DEFAULT_TIMEOUT);
			void _disableDataGrabbing();
			sl_result _waitNode(sl_lidar_response_measurement_node_t * node, uint32_t timeout = DEFAULT_TIMEOUT);
			sl_result _waitScanData(sl_lidar_response_measurement_node_t * nodebuffer, size_t & count, uint32_t timeout = DEFAULT_TIMEOUT);
			sl_result _cacheScanData();
			void _ultraCapsuleToNormal(const sl_lidar_response_ultra_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount);
			sl_result _waitCapsuledNode(sl_lidar_response_capsule_measurement_nodes_t & node, uint32_t timeout = DEFAULT_TIMEOUT);
			void _capsuleToNormal(const sl_lidar_response_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount);
			void _dense_capsuleToNormal(const sl_lidar_response_capsule_measurement_nodes_t & capsule, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount);
			sl_result _cacheCapsuledScanData();
			sl_result _waitHqNode(sl_lidar_response_hq_capsule_measurement_nodes_t & node, uint32_t timeout = DEFAULT_TIMEOUT);
			void _HqToNormal(const sl_lidar_response_hq_capsule_measurement_nodes_t & node_hq, sl_lidar_response_measurement_node_hq_t *nodebuffer, size_t &nodeCount);
			sl_result _cacheHqScanData();
			sl_result _waitUltraCapsuledNode(sl_lidar_response_ultra_capsule_measurement_nodes_t & node, uint32_t timeout = DEFAULT_TIMEOUT);
			sl_result _cacheUltraCapsuledScanData();
			sl_result _clearRxDataCache();

		private:
			IChannel *_channel;
			bool _isConnected;
			bool _isScanning;
			MotorCtrlSupport _isSupportingMotorCtrl;

			rp::hal::Locker         _lock;
			rp::hal::Event          _dataEvt;
			rp::hal::Thread         _cachethread;
			uint16_t                    _cached_sampleduration_std;
			uint16_t                    _cached_sampleduration_express;

			sl_lidar_response_measurement_node_hq_t   _cached_scan_node_hq_buf[8192];
			size_t                                   _cached_scan_node_hq_count;
			uint8_t                                    _cached_capsule_flag;

			sl_lidar_response_measurement_node_hq_t   _cached_scan_node_hq_buf_for_interval_retrieve[8192];
			size_t                                   _cached_scan_node_hq_count_for_interval_retrieve;

			sl_lidar_response_capsule_measurement_nodes_t       _cached_previous_capsuledata;
			sl_lidar_response_dense_capsule_measurement_nodes_t _cached_previous_dense_capsuledata;
			sl_lidar_response_ultra_capsule_measurement_nodes_t _cached_previous_ultracapsuledata;
			sl_lidar_response_hq_capsule_measurement_nodes_t _cached_previous_Hqdata;
			bool                                         _is_previous_capsuledataRdy;
			bool                                         _is_previous_HqdataRdy;
	};

}
