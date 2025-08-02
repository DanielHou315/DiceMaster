# Protocol Design

The Raspberry Pi and the Screen Modules communicate with each other in SPI through packets that are formatted as follows. 

**Message Header**

This section of bytes is attached to each packet, storing the necessary statistics and message types for parsing of the rest of the message. 

MOSI: 2048 Bytes (2kb buffer, 480x480 jpegs are mostly 10~15kb)
Command Messages: 5 bytes
- BYTE 0: Start of Frame (SOF): 0x7E
- BYTE 1: Message Type
- BYTE 2: Message ID
- BYTE 3-4: Payload length (BIG_ENDIAN)
- The payload starts at BYTE 5 onwards. 

**Message Types**
- 0x01 - Text Batch
- 0x02 - Image Transfer Start
- 0x03 - Image Chunk
- 0x04 - Image Transfer End
- 0x05 - Backlight On
- 0x06 - Backlight Off
- 0x07 - Ping Request
- 0x08 - Ping Response
- 0x09 - Acknowledgment (ACK)
- 0x0A - Error Message (NACK)

The payload of the message is different for different types of messages, described as follows. 

### Text Section
**Text Group**
A text group with rotation support:
- BYTE 0-1: BG Color
- BYTE 4: number of lines
- BYTE 5: rotation (0=0°, 1=90°, 2=180°, 3=270°)
- Individual chunks 

**Text**

- BYTE 0-1: x cursor
- BYTE 2-3: y cursor
- BYTE 4: font id
- BYTE 5-6: Font Color
- BYTE 7: text length
- PAYLOAD STRING
**NOTE** each text block can be NO LONGER THAN 255 BYTES due to size byte limit. 

### Images
**Image Begin**
- BYTE 0: image ID
- BYTE 1: 4-bit Format, 4-bit Resolution
- BYTE 2-3: Delay Time (0-65535 ms)
- BYTE 4-6: total image size
- BYTE 7: num chunks (includes this embedded chunk 0)
- BYTE 8: rotation (0=0°, 1=90°, 2=180°, 3=270°)
- BYTE 9 onward: Image chunk 0 data (embedded in ImageStart message)

**IMAGE CHUNK**
- BYTE 0: image ID
- BYTE 1: chunk ID (starts from 1, since chunk 0 is embedded in ImageStart)
- BYTE 2-4: starting location
- BYTE 5-6: length of chunk (max 8KB - header size)
- PAYLOAD

**Image Transfer Timeout**
Images are automatically invalidated if not all chunks are received within the timeout period:
- Timeout = 100ms × number of chunks
- Each image tracks which chunks have been received
- Missing chunks after timeout will mark the image as expired
- No explicit ImageEnd message is needed

### Ping Command

**Ping Request**
- No payload (0 bytes)
- Used to check board status and connectivity

**Screen Response** (Not used yet)
- BYTE 0: SOF marker for response (0x7F)
- BYTE 1: message ID
- BYTE 2-5: status code (0=OK, 1=Warning, 2=Error)
- BYTE 6-7: padding (0)

## Error Code

**General Errors**
- 0x00 - SUCCESS
- 0x01 - UNKNOWN_MSG_TYPE
- 0x02 - INVALID_FORMAT
- 0x04 - IMAGE_ID_MISMATCH
- 0x05 - PAYLOAD_LENGTH_MISMATCH
- 0x06 - UNSUPPORTED_IMAGE_FORMAT
- 0x07 - OUT_OF_MEMORY
- 0x08 - INTERNAL_ERROR
- 0x09 - INVALID_OPTION_INDEX
- 0x0A - UNSUPPORTED_MESSAGE

**Header Decoding Errors**
- 0x10 - HEADER_TOO_SHORT
- 0x11 - INVALID_SOF_MARKER
- 0x12 - INVALID_MESSAGE_TYPE
- 0x13 - INVALID_LENGTH_FIELD
- 0x14 - HEADER_LENGTH_MISMATCH

**TextBatch Specific Errors**
- 0x20 - TEXT_PAYLOAD_TOO_SHORT
- 0x21 - TEXT_TOO_MANY_ITEMS
- 0x22 - TEXT_INVALID_ROTATION
- 0x23 - TEXT_ITEM_HEADER_TOO_SHORT
- 0x24 - TEXT_ITEM_LENGTH_MISMATCH
- 0x25 - TEXT_PAYLOAD_TRUNCATED
- 0x26 - TEXT_LENGTH_CALCULATION_ERROR

**ImageStart Specific Errors**
- 0x30 - IMAGE_START_TOO_SHORT
- 0x31 - IMAGE_START_INVALID_ROTATION
- 0x32 - IMAGE_START_INVALID_FORMAT
- 0x33 - IMAGE_START_INVALID_RESOLUTION

**ImageChunk Specific Errors**
- 0x40 - IMAGE_CHUNK_TOO_SHORT
- 0x41 - IMAGE_CHUNK_DATA_TRUNCATED
- 0x42 - IMAGE_CHUNK_INVALID_LENGTH

**ImageEnd Specific Errors**
- 0x50 - IMAGE_END_TOO_SHORT

**Ping Specific Errors**
- 0x80 - PING_REQUEST_NOT_EMPTY
- 0x81 - PING_RESPONSE_TOO_SHORT
- 0x82 - PING_RESPONSE_TEXT_TRUNCATED

**Ack/Error Specific Errors**
- 0x90 - ACK_TOO_SHORT
- 0x91 - ERROR_TOO_SHORT
- 0x92 - ERROR_TEXT_TRUNCATED


## File Structure

Raspberry PI SD Card Folder Requirements: 
SD_ROOT → activity

SD_ROOT
| – 2024FA-GER-354
|	| – 20240826
| 	|	| – activity-1
|	| – 20240827-20240828
| 	|	| – activity-1
| 	|	| – activity-2
| – ENG-125-002-FA24
|	| – discussion-15
| 	|	| – activity-1
| 	|	| – activity-2
