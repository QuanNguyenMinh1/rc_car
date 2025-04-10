/*
 * uart_proto.c
 *
 *  Created on: Sep 4, 2024
 *      Author: Quan
 */

#include "uart_proto.h"
#include "crc16.h"
/**
 * @brief Get data from Frame
 *
 * @param pu8Src :Data received, it's can be 1 or 2 ... frame, we don't know ** Chỉ cần truyền địa chỉ của RxBuffer vào parameter pu8Src là được
 * @param u16Src_len :Length of data received, can be known by adding up the total number of uart interrupts until there is enough data ** thêm biến count vào hàm ngắt UART -> cập nhật số byte đã nhận được
 * @param pu8Dest :Data cut out of frame
 * @param pu16Dest_len :Length of data that has been trimmed from frame ** Truyền địa chỉ buffer stores message vào
 * @return int8_t to check if frame can be trimmed correctly
 * hàm này return right/ false; dựa vào param
 */
int8_t UART_get_data(uint8_t *pu8Src, uint16_t u16Src_len, uint8_t *pu8Dest, uint16_t *pu16Dest_len, uint16_t *rxBufPtr, uint8_t rxData_len)
{
//	while (*pu8Src != 0x03 && *(pu8Src + 1) != '\0')
//	{
//		pu8Src
//	}
	const uint8_t *pu8Src_start = pu8Src; // pu : pointer unsigned; pu8Src_start lưu địa chỉ là địa chỉ của Src
	const uint8_t *pu8Src_findStart = pu8Src + (u16Src_len); // 1 START BYTE + 2 BYTE ĐỘ DÀI của RECEIVED DATA -> Địa chỉ của data frame tiếp theo
	char checkFindStart = 0;
	uint8_t checkESC = 0;
	uint8_t index = 0;
	uint16_t crc_check = 0;
	// Advance src to START BYTE 				// đi tìm START BYTE từ đầu frame này tới đầu frame tiếp theo			// So địa chỉ
	while(pu8Src < pu8Src_findStart && *pu8Src != PROTO_START_BYTE) //IF chưa tìm thấy start byte thì tăng địa chỉ cho tới khi thấy; pu8Src : lưu địa chỉ -> tìm START BYTE; PROTO_START_BYTE: start byte do mình định nghĩa
	{
		pu8Src++;								// Tăng địa chỉ lên dần để đọc từng byte trong RxBuff
	}
	// Thoát ra khỏi while <=> tìm được START BYTE rồi
	if(*pu8Src == PROTO_START_BYTE)   // If you just enter and receive the start byte, then you don't have to enter the while function
	{
		checkFindStart = 1; // = 1: valid
	}
	// Set our error return val for dest_len
	if(checkFindStart == 0)
	{
	   return no_valid; // = -1: no valid
	}
	// Loop through the data
	pu8Src++; // dịch thêm 1 byte địa chỉ -> trỏ tới địa chỉ của ESC BYTE
	while(index < rxData_len) // cho tới khi index >= PROTO_DATA_SIZE_RX									// So phần tử (thứ tự trong frame); phạm vi lặp: [0; 12] -> lặp tối đa 13 lần
	{
		if (*pu8Src == PROTO_ESC_BYTE) // 0x7E																		// If cái giá trị của ESC BYTE trùng với ĐỊNH NGHĨA của người lập trình
		{
			crc_check = crc16_floating(*pu8Src, crc_check); // If the ESC BYTE is found, the next byte will be XORed with 0x20 to retrieve its ORIGINAL value. // newData: *pu8Src, oldData: crc_check
			*(pu8Dest++)  = (*(++pu8Src)) ^ 0x20; // byte kế escByte ^ 0x20 -> message ** stores message into *pu8Dest	*	0x5E xor 0x20 = 0x7E (ESC BYTE)
			crc_check = crc16_floating(*pu8Src, crc_check);
			checkESC++; // so lan tim thay ESC byte
		}
		else
		{
			crc_check = crc16_floating(*pu8Src, crc_check); // IF chưa tìm được ESC BYTE
			*(pu8Dest++) = *pu8Src; // chép qua Des
		}
		pu8Src++; // -> mỗi lần tính crc_check xong là tăng 1 cho pu8Src (tính crc từng byte một) -> nguyên 1 frame -> check xác thực cho cả frame => quan tâm crc_check cuối cùng
		index++;
	}
	// pu8src = &escByte + 1 = &byte1CRC														/* START ESC CRC1 CRC2 */
	if (*(pu8Src + 2) != PROTO_END_BYTE)
	{  // pu8Src in first CRC now
		return no_valid;
	}
	uint8_t byte2_crc = (crc_check) & 0xFF; // & 1111 1111 => lấy 8 bit cuối
	uint8_t byte1_crc = (crc_check >> 8) & 0xFF; //	>> 8 -> & 1111 1111 => còn 8 bit đầu	* crc_check chứa 2 byte => tách ra để đem từng byte đi so sánh với gtri crc nằm trong frame
	if(*(pu8Src) != byte1_crc || *(++pu8Src) != byte2_crc) // so giá trị CRC nằm trong frame nhân được với CRC tính lại từ tất cả byte trong frame được nhận được để xem trong quá trình truyền có lỗi về data không
	{
		return false_CRC;	// CHECK CRC
	}
	*pu16Dest_len =(++pu8Src - pu8Src_start - 4 - checkESC + 1); // length data received (thực tế)		// địa chỉ ô nhớ đầu tiên + 1 = địa chỉ ESC BYTE -> - địa chỉ ô nhớ đầu tiên ban đầu của frame trước
	if(*pu16Dest_len != rxData_len) // PROTO_DATA_SIZE_RX: dự kiến
	{
		return false_lenght_data;	// Check DATA LENGTH
	}
	//
	*rxBufPtr += *pu16Dest_len;
//	pu8Src = pu8SrcInitialAddress;
	return right;
}
/**
 * @brief Create Frame
 *
 * @param pu8Src :Source raw data ** truyền địa chỉ TxRawBuffer vào
 * @param u8Src_len :Length of source RAW DATA, it will be known before by programmer
 * @param pu8Dest :Create frame with source raw data, it will be like: start mode data1 data2 (ESC byte if data is duplicate start stop or ESC).. crc1 crc2 stop ** truyền địa chỉ TxBuffer vào
 * @param pu16Dest_len :Length of frame
 */
void UART_frame_data(uint8_t *pu8Src, uint8_t u8Src_len, uint8_t *pu8Dest, uint16_t *pu16Dest_len)
{
	uint8_t index = 0;
	uint8_t checkESC = 0;
	uint16_t crc = 0;
	*(pu8Dest++) = PROTO_START_BYTE; // first place is startByte
//debug
//	*(pu8Dest++) = 99; // first place is startByte

//	u8Src_len = 4; // lỗi do ghi lên u8Src_len
//	while(1){
	while(index < u8Src_len) {						// 	mình thay TxRawBuff cho *pu8Src	để hàm này tạo frame cho rawData	**	while này fill đủ 1 frame luôn
//		goto even;
//
//		even:
//		    index = 5;
		    // return if even
//		    return;
			if (*pu8Src == PROTO_START_BYTE || *pu8Src == PROTO_ESC_BYTE || *pu8Src == PROTO_END_BYTE) {
					*(pu8Dest++) = PROTO_ESC_BYTE; // 0x7E = '~'
					crc = crc16_floating(*(pu8Dest-1), crc);
					*(pu8Dest++) = (*pu8Src) ^ 0x20; // 0x7E xor 0x20 = ^
					crc = crc16_floating(*(pu8Dest-1), crc);
					checkESC++;
			}
			else {
					crc = crc16_floating(*pu8Src, crc); // encode: start + mode + data1 + data2 + (esc) + crc1 + crc2 + stop
					*(pu8Dest++) = *pu8Src; // 'a', 'h'
			}
			++pu8Src;
			index++;
	}

	// Set the CRC

	//Casting the CRC to let the word be assigned to a non-word boundary in memory		** ép kiểu sang char để lấy 8 bit thấp của đối tượng đang được ép kiểu
	*(pu8Dest++) = (char)(crc >>8); // uint16_t crc mà pu8Dest là uint8_t => tách đôi crc ra 	**	lưu 8 bit cao của CRC vào vị trí pu8Dest đang trỏ tới
//	pu8Dest++;
	*(pu8Dest++) = (char)crc; // lưu 8 bit thấp của CRC vào vị trí pu8Dest đang trỏ tới
//	pu8Dest++;
	*(pu8Dest) = PROTO_END_BYTE;
//	*(pu16Dest_len) = pu8Dest - pu8Dest_start;
	*(pu16Dest_len) = u8Src_len + checkESC + 4;   // length frame gồm length(1 byte Data, escByte (có 1 hoặc nhiều byte), startByte, 1 byte crc_1, 1 byte crc_2, 1 byte stop)
}												// 4 equal start crc crc stop // Tạo một con trỏ global lưu len của frame

/*
Escape Special Bytes: The function checks if the data contains special bytes like PROTO_START_BYTE, PROTO_ESC_BYTE, or PROTO_END_BYTE.
If any of these bytes are found, they are preceded by an escape byte, and the original byte is XORed with 0x20.
End Byte and Length Check: The function verifies the presence of the end byte (PROTO_END_BYTE) and checks if the length of the extracted data matches the expected length (PROTO_DATA_SIZE_RX).
*/

/*
| Start | Mode | Data | CRC1 | CRC2 | End |
|  0x7E | 0x01 | 0x5F | 0x12 | 0x34 | 0x7F |
*/

