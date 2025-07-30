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
- BYTE 7: num chunks
- BYTE 78 rotation (0=0°, 1=90°, 2=180°, 3=270°)

**IMAGE CHUNK**
- BYTE 0: image ID
- BYTE 1: chunk ID
- BYTE 2-4: starting location
- BYTE 5-6: length of chunk (max 65535 Bytes)
- PAYLOAD

**IMAGE END**
- BYTE 0: Image ID

### Ping Command

**Ping Request**
- No payload (0 bytes)
- Used to check board status and connectivity

**Ping Response**
- BYTE 0: SOF marker for response (0x7F)
- BYTE 1-5: status code (0=OK, 1=Warning, 2=Error)
- BYTE 6-7: padding (0)


## Error Code

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
