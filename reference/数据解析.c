/*深度图数据*/
#include <api.h>
typedef struct
{
	uint16_t distance_average;
	uint16_t amplitude_average;
	uint16_t amplitude_average_whole;
	uint16_t amplitude_low_count;
	uint16_t saturation_count;
	uint16_t distance_max;
	uint16_t distance_min;
	int16_t temperature;/*温度值*/
	uint16_t frame_cnt;
	uint16_t interference_num;/*受干扰的像素点*/
	uint16_t distance[MAX_PIX_NUM];  /*深度数据*/
}DepthDataTypeDef;


/*联合体封装 用于测量数据的返回*/
typedef union
{
	SimpleRoiDataTypeDef simple_roi_data[ROI_NUM];
	DepthDataTypeDef simple_depth_data;
	DepthDataTypeDef full_depth_data;
}MeasureDataTypeDef;





/**
  * @brief 解析测量数据包
  * @param[in]	buf:返回包原始数据
  * @param[out] MeasureData：测量数据
	* @param[out] RetPacketType：测量数据类型
  * @note
  * @see
  * @code
  * @retval	解析成功返回RET_OK
  */
RET_StatusTypeDef HPS3D_MeasureDataParse(uint8_t *buf, MeasureDataTypeDef *MeasureData, RetPacketTypedef *RetPacketType)
{
	/*uint8_t dev_addr = 0;*/
	RET_StatusTypeDef ret = RET_OK;
	uint32_t size = 0;
	uint8_t rid = 0;  /*命令Rid返回值*/
	uint16_t tag = 0;
	uint8_t RoiNumber = 0;
	uint32_t frame_cnt = 0;
/*	uint16_t dummy = 0;*/
	*RetPacketType = NULL_PACKET; /*默认幅值为空类型*/
	
	do
	{
		/*验证报头 第0和1个字节为报头*/  
		if ((buf[0] != PACKET_HEAD1) || (buf[1] != PACKET_HEAD2))
		{
			ret = RET_ERROR;
			break;
		}
		/*得到数据长度 第2和3个字节为报文长度*/
		size = (uint32_t)( (buf[2] << 0) | (buf[3] << 8) );
		/*进行数据包CRC16校验*/
		ret = CRC_VerifyPacket(buf,size + 4);
		if(ret != RET_OK)
		{
			break;
		}
		/*得到设备地址与rid*/
		//dev_addr = buf[4];
		rid = buf[4 + 1];
		
		/*进行简单数据包进行解析*/
		switch(rid)
		{
			case RID_SIMPLE_ROI_PACKET: /*简单ROI数据包解析*/
				*RetPacketType = SIMPLE_ROI_PACKET;
				tag = (uint16_t)(buf[4 + 3] * 256 + buf[4 + 2]);
				RoiNumber = tag & 0xff;
				frame_cnt = (uint32_t)((buf[4 + 7] << 24) | (buf[4 + 6]) << 16 | (buf[4 + 5] << 8) | (buf[4 + 4] << 0));
				for(int i = 0;i < RoiNumber; i++)
				{
					MeasureData->simple_roi_data[i].group_id = (uint8_t)(tag >> 8);
					MeasureData->simple_roi_data[i].frame_cnt = frame_cnt;
					MeasureData->simple_roi_data[i].id = (uint8_t)(buf[i*ROI_EXTEND_INFO_LEN+ROI_HEADER_INFO_LEN+1+2+4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 0 + 2 + 4]);
					MeasureData->simple_roi_data[i].amplitude = (uint16_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 3 + 2 + 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 2 + 2 + 4]);
					MeasureData->simple_roi_data[i].valid_amplitude = (uint16_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 5 + 2 + 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 4 + 2 + 4]);
					MeasureData->simple_roi_data[i].distance_average = (uint16_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 7 + 2+ 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 6 + 2 + 4]);
					MeasureData->simple_roi_data[i].distance_max = (uint16_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 9 + 2 + 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 8 + 2 + 4]);
					MeasureData->simple_roi_data[i].distance_min = (uint16_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 11 + 2+ 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 10 + 2 + 4]);
					MeasureData->simple_roi_data[i].saturation_count = (uint16_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 13 + 2 + 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 12 + 2 + 4]);
					MeasureData->simple_roi_data[i].threshold_state = (uint8_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 15 + 2 + 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 14 + 2 + 4]);
					MeasureData->simple_roi_data[i].dist_max_x = (uint16_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 17 + 2 + 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 16 + 2 + 4]);
					MeasureData->simple_roi_data[i].dist_max_y = (uint16_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 19 + 2 + 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 18 + 2 + 4]);
					MeasureData->simple_roi_data[i].dist_min_x = (uint16_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 21 + 2 + 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 20 + 2 + 4]);
					MeasureData->simple_roi_data[i].dist_min_y = (uint16_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 23 + 2 + 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 22 + 2 + 4]);
					MeasureData->simple_roi_data[i].out_of_threshold_pix_num[0] = (uint16_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 25 + 2 + 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 24 + 2 + 4]);
					MeasureData->simple_roi_data[i].out_of_threshold_pix_num[1] = (uint16_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 27 + 2 + 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 26 + 2 + 4]);
					MeasureData->simple_roi_data[i].out_of_threshold_pix_num[2] = (uint16_t)(buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 29 + 2 + 4] * 256 + buf[i * ROI_EXTEND_INFO_LEN + ROI_HEADER_INFO_LEN + 28 + 2 + 4]);
					
				}
				break;
			case RID_SIMPLE_DEPTH_PACKET: /*简单深度数据解析*/
				*RetPacketType = SIMPLE_DEPTH_PACKET;
				/*dummy = (uint16_t)(buf[3 + 4] * 256 + buf[2 + 4]); */
				MeasureData->simple_depth_data.distance_average = (uint16_t)(buf[5 + 4] * 256 + buf[4 + 4]);
				MeasureData->simple_depth_data.amplitude_average = (uint16_t)(buf[7 + 4] * 256 + buf[6 + 4]);
				MeasureData->simple_depth_data.amplitude_average_whole = (uint16_t)(buf[9 + 4] * 256 + buf[8 + 4]);
				MeasureData->simple_depth_data.amplitude_low_count = (uint16_t)(buf[11 + 4] * 256 + buf[10 + 4]);
				MeasureData->simple_depth_data.saturation_count = (uint16_t)(buf[13 + 4] * 256 + buf[12 + 4]);
				MeasureData->simple_depth_data.distance_max = (uint16_t)(buf[15 + 4] * 256 + buf[14 + 4]);
				MeasureData->simple_depth_data.distance_min = (uint16_t)(buf[17 + 4] * 256 + buf[16 + 4]);
				MeasureData->simple_depth_data.frame_cnt = (uint32_t)((buf[21 + 4] << 24) | (buf[20 + 4]) << 16 | (buf[19 + 4] << 8) | (buf[18 + 4] << 0));
				MeasureData->simple_depth_data.interference_num = (uint16_t)(buf[23 + 4] * 256 + buf[22 + 4]);
				MeasureData->simple_depth_data.temperature = (int16_t)(buf[25 + 4] * 256 + buf[24 + 4]);								
				break;		
			case RID_FULL_DEPTH_PACKET: /*完整深度数据包解析*/
				*RetPacketType = FULL_DEPTH_PACKET;
				/*dummy = (uint16_t)(buf[3 + 4] * 256 + buf[2 + 4]); */
				MeasureData->full_depth_data.distance_average = (uint16_t)(buf[5 + 4] * 256 + buf[4 + 4]);
				MeasureData->full_depth_data.amplitude_average = (uint16_t)(buf[7 + 4] * 256 + buf[6 + 4]);
				MeasureData->full_depth_data.amplitude_average_whole = (uint16_t)(buf[9 + 4] * 256 + buf[8 + 4]);
				MeasureData->full_depth_data.amplitude_low_count = (uint16_t)(buf[11 + 4] * 256 + buf[10 + 4]);
				MeasureData->full_depth_data.saturation_count = (uint16_t)(buf[13 + 4] * 256 + buf[12 + 4]);
				MeasureData->full_depth_data.distance_max = (uint16_t)(buf[15 + 4] * 256 + buf[14 + 4]);
				MeasureData->full_depth_data.distance_min = (uint16_t)(buf[17 + 4] * 256 + buf[16 + 4]);
				MeasureData->full_depth_data.frame_cnt = (uint32_t)((buf[21 + 4] << 24) | (buf[20 + 4]) << 16 | (buf[19 + 4] << 8) | (buf[18 + 4] << 0));
				MeasureData->full_depth_data.interference_num = (uint16_t)(buf[23 + 4] * 256 + buf[22 + 4]);
				MeasureData->full_depth_data.temperature = (int16_t)(buf[25 + 4] * 256 + buf[24 + 4]);	
				for(int i= 0;i < PIXEL_NUM;i++)
				{
					MeasureData->full_depth_data.distance[i] = (buf[i*2 + 26 + 4 + 1]*256 +  buf[i*2 + 26 + 4]);
				}
			
				break;		
			default:
				ret = RET_ERROR;
				break;
		}
			
	}while(0);
	
	return ret;
}