// This comes with no warranty, implied or otherwise

// This data structure was designed to support Proportional fonts
// on Arduinos. It can however handle any ttf font that has been converted
// using the conversion program. These could be fixed width or proportional 
// fonts. Individual characters do not have to be multiples of 8 bits wide. 
// Any width is fine and does not need to be fixed.

// The data bits are packed to minimize data requirements, but the tradeoff
// is that a header is required per character.

// Ubuntu-B_21.c
// Point Size   : 21
// Memory usage : 2258 bytes
// # characters : 95

// Header Format (to make Arduino UTFT Compatible):
// ------------------------------------------------
// Character Width (Used as a marker to indicate use this format. i.e.: = 0x00)
// Character Height
// First Character (Reserved. 0x00)
// Number Of Characters (Reserved. 0x00)

const unsigned char tft_UbuntuB21[] = 
{
0x00, 0x14, 0x00, 0x00,

// Individual Character Format:
// ----------------------------
// Character Code
// Adjusted Y Offset
// Width
// Height
// xOffset
// xDelta (the distance to move the cursor. Effective width of the character.)
// Data[n]

// NOTE: You can remove any of these characters if they are not needed in
// your application. The first character number in each Glyph indicates
// the ASCII character code. Therefore, these do not have to be sequential.
// Just remove all the content for a particular character to save space.

// ' '
0x20,0x11,0x00,0x00,0x00,0x05,

// '!'
0x21,0x02,0x03,0x0F,0x01,0x05,
0xFF,0xFF,0xFF,0xFC,0x2F,0xD0,
// '"'
0x22,0x01,0x08,0x06,0x01,0x0A,
0xE7,0xE7,0xE7,0xE7,0xE7,0xE7,
// '#'
0x23,0x02,0x0D,0x0F,0x01,0x0F,
0x0E,0x70,0x73,0x83,0xBC,0x39,0xCF,0xFF,0xFF,0xFC,0x73,0x87,0xB8,0x39,0xC7,0xFF,0xFF,0xFE,0x73,0x87,0xB8,0x39,0xC1,0xCE,0x00,
// '$'
0x24,0x01,0x0A,0x13,0x01,0x0C,
0x1C,0x07,0x03,0xF1,0xFE,0xFF,0xB8,0x0E,0x03,0xE0,0x7F,0x0F,0xE0,0x7C,0x07,0x41,0xFF,0xFF,0xF9,0xFC,0x1C,0x07,0x01,0xC0,
// '%'
0x25,0x02,0x11,0x0F,0x01,0x13,
0x38,0x1C,0x3E,0x1C,0x33,0x8E,0x18,0xCE,0x0C,0x67,0x06,0x77,0x01,0xF3,0x80,0x73,0x9C,0x03,0x9F,0x01,0xD9,0xC1,0xCC,0x60,0xE6,0x30,0xE3,0x38,0x70,0xF8,0x70,0x38,
// '&'
0x26,0x02,0x0E,0x0F,0x01,0x0F,
0x0F,0x00,0x7E,0x03,0xFC,0x0E,0x70,0x39,0xC0,0x7E,0x01,0xF0,0x0F,0xCE,0x77,0xBB,0x8F,0xCE,0x1F,0x3C,0x78,0xFF,0xF1,0xFF,0xE3,0xF3,0xC0,
// '''
0x27,0x01,0x03,0x06,0x01,0x05,
0xFF,0xFF,0xC0,
// '('
0x28,0x01,0x06,0x14,0x01,0x07,
0x08,0x73,0x8E,0x71,0xCF,0x38,0xE3,0x8E,0x38,0xE3,0xC7,0x1C,0x38,0xE1,0xC2,
// ')'
0x29,0x01,0x06,0x14,0x00,0x07,
0x43,0x87,0x1C,0x38,0xE3,0xC7,0x1C,0x71,0xC7,0x1C,0xF3,0x8E,0x71,0xCE,0x10,
// '*'
0x2A,0x02,0x09,0x08,0x01,0x0B,
0x1C,0x0E,0x17,0x5F,0xFF,0xF9,0xF1,0xDC,0x28,
// '+'
0x2B,0x05,0x0B,0x0B,0x01,0x0D,
0x0E,0x01,0xC0,0x38,0x07,0x0F,0xFF,0xFF,0xFF,0xF8,0x70,0x0E,0x01,0xC0,0x38,0x00,
// ','
0x2C,0x0E,0x04,0x06,0x00,0x05,
0x77,0x77,0xE6,
// '-'
0x2D,0x09,0x06,0x03,0x01,0x08,
0xFF,0xFF,0xC0,
// '.'
0x2E,0x0D,0x04,0x04,0x01,0x06,
0x6F,0xF6,
// '/'
0x2F,0x01,0x0B,0x14,0xFF,0x09,
0x00,0xE0,0x38,0x07,0x01,0xE0,0x38,0x07,0x01,0xC0,0x38,0x0F,0x01,0xC0,0x38,0x0E,0x01,0xC0,0x38,0x0E,0x01,0xC0,0x78,0x0E,0x01,0xC0,0x70,0x00,
// '0'
0x30,0x02,0x0A,0x0F,0x01,0x0C,
0x1E,0x0F,0xC7,0xF9,0xCE,0xE1,0xF8,0x7E,0x1F,0x87,0xE1,0xF8,0x7E,0x1D,0xCE,0x7F,0x8F,0xC1,0xE0,
// '1'
0x31,0x02,0x07,0x0F,0x02,0x0C,
0x06,0x1C,0xFB,0xFF,0xED,0xC3,0x87,0x0E,0x1C,0x38,0x70,0xE1,0xC3,0x80,
// '2'
0x32,0x02,0x0A,0x0F,0x01,0x0C,
0x3F,0x1F,0xEF,0xFD,0x8F,0x01,0xC0,0x70,0x38,0x1C,0x0E,0x0F,0x07,0x81,0xC0,0xFF,0xFF,0xFF,0xFC,
// '3'
0x33,0x02,0x0A,0x0F,0x01,0x0C,
0x3E,0x3F,0xCF,0xF9,0x1E,0x03,0x81,0xC3,0xF0,0xFE,0x03,0xC0,0x70,0x1C,0x0F,0xFF,0xBF,0xE7,0xE0,
// '4'
0x34,0x02,0x0A,0x0F,0x01,0x0C,
0x07,0x03,0xC1,0xF0,0x7C,0x3F,0x0D,0xC7,0x71,0xDC,0xE7,0x3F,0xFF,0xFF,0xFF,0x07,0x01,0xC0,0x70,
// '5'
0x35,0x02,0x0A,0x0F,0x01,0x0C,
0x3F,0x8F,0xE3,0xF8,0xE0,0x38,0x1F,0x87,0xF9,0xFE,0x07,0xC0,0x70,0x1C,0x0F,0xFF,0xBF,0xE7,0xE0,
// '6'
0x36,0x02,0x0A,0x0F,0x01,0x0C,
0x03,0x87,0xE3,0xF9,0xF0,0x70,0x3F,0xCF,0xFB,0xFF,0xE3,0xF8,0x7E,0x1F,0xCF,0x7F,0x9F,0xE1,0xE0,
// '7'
0x37,0x02,0x0A,0x0F,0x01,0x0C,
0xFF,0xFF,0xFF,0xFC,0x0E,0x07,0x01,0xC0,0xE0,0x38,0x1E,0x07,0x01,0xC0,0x70,0x38,0x0E,0x03,0x80,
// '8'
0x38,0x02,0x0A,0x0F,0x01,0x0C,
0x1E,0x1F,0xE7,0xFF,0xCF,0xE1,0xF8,0x77,0xB9,0xFC,0x7F,0xBC,0xFE,0x1F,0xC7,0xFF,0xDF,0xE3,0xF0,
// '9'
0x39,0x02,0x0A,0x0F,0x01,0x0C,
0x1E,0x1F,0xC7,0xFB,0xCF,0xE1,0xF8,0x7F,0x1F,0xFF,0x7F,0xCF,0xF0,0x38,0x1E,0x7F,0x1F,0x87,0x80,
// ':'
0x3A,0x06,0x04,0x0B,0x01,0x06,
0x6F,0xF6,0x00,0x06,0xFF,0x60,
// ';'
0x3B,0x06,0x05,0x0E,0x00,0x06,
0x33,0xDE,0x60,0x00,0x00,0x73,0x9C,0xEE,0x30,
// '<'
0x3C,0x05,0x0B,0x0B,0x00,0x0C,
0x00,0x40,0x78,0x3F,0xBF,0xC7,0xE0,0xF0,0x1F,0x83,0xFC,0x0F,0xE0,0x78,0x01,0x00,
// '='
0x3D,0x07,0x0A,0x08,0x01,0x0C,
0xFF,0xFF,0xFF,0xFC,0x00,0x00,0x3F,0xFF,0xFF,0xFF,
// '>'
0x3E,0x05,0x0B,0x0B,0x01,0x0C,
0x40,0x0F,0x03,0xF8,0x1F,0xE0,0xFC,0x07,0x83,0xF1,0xFE,0xFE,0x0F,0x01,0x00,0x00,
// '?'
0x3F,0x01,0x09,0x10,0x00,0x0A,
0x7E,0x7F,0xBF,0xE8,0x70,0x38,0x3C,0x3C,0x3C,0x3C,0x1C,0x0E,0x00,0x01,0x81,0xE0,0xF0,0x30,
// '@'
0x40,0x02,0x12,0x12,0x01,0x14,
0x03,0xF8,0x03,0xFF,0x81,0xC0,0xF0,0xE0,0x0E,0x71,0xF1,0x99,0xFE,0x3C,0x73,0x8F,0x38,0xE3,0xCE,0x38,0xF3,0x8E,0x3C,0xE3,0x8F,0x3C,0xE6,0xE7,0xFF,0x98,0xFF,0x87,0x00,0x00,0xF0,0x00,0x1F,0xF8,0x00,0xFE,0x00,
// 'A'
0x41,0x02,0x0F,0x0F,0x00,0x0F,
0x07,0xC0,0x0F,0x80,0x3F,0x80,0x7F,0x00,0xEE,0x03,0xDE,0x07,0x1C,0x0E,0x38,0x3C,0x78,0x7F,0xF1,0xFF,0xF3,0xFF,0xE7,0x01,0xDE,0x03,0xF8,0x03,0x80,
// 'B'
0x42,0x02,0x0B,0x0F,0x02,0x0E,
0xFF,0x1F,0xF3,0xFF,0x70,0xEE,0x1D,0xFF,0x3F,0xE7,0xFE,0xE1,0xFC,0x1F,0x83,0xF0,0xFF,0xFD,0xFF,0xBF,0xC0,
// 'C'
0x43,0x02,0x0C,0x0F,0x01,0x0E,
0x07,0xE1,0xFF,0x3F,0xF7,0x82,0x70,0x0E,0x00,0xE0,0x0E,0x00,0xE0,0x0E,0x00,0xF0,0x07,0x82,0x7F,0xF3,0xFF,0x0F,0xE0,
// 'D'
0x44,0x02,0x0C,0x0F,0x02,0x0F,
0xFF,0x0F,0xF8,0xFF,0xEE,0x1E,0xE0,0xFE,0x07,0xE0,0x7E,0x07,0xE0,0x7E,0x07,0xE0,0xFE,0x1E,0xFF,0xCF,0xF8,0xFE,0x00,
// 'E'
0x45,0x02,0x0A,0x0F,0x02,0x0D,
0xFF,0xFF,0xFF,0xFF,0x80,0xE0,0x38,0x0F,0xFB,0xFE,0xFF,0xB8,0x0E,0x03,0x80,0xFF,0xFF,0xFF,0xFC,
// 'F'
0x46,0x02,0x09,0x0F,0x02,0x0C,
0xFF,0xFF,0xFF,0xFC,0x0E,0x07,0x03,0xFD,0xFE,0xFF,0x70,0x38,0x1C,0x0E,0x07,0x03,0x80,
// 'G'
0x47,0x02,0x0C,0x0F,0x01,0x0F,
0x0F,0xE1,0xFF,0x3F,0xF7,0x82,0x70,0x0E,0x00,0xE0,0x0E,0x07,0xE0,0x7E,0x07,0xF0,0x77,0x87,0x7F,0xF3,0xFF,0x0F,0xE0,
// 'H'
0x48,0x02,0x0B,0x0F,0x02,0x0F,
0xE0,0xFC,0x1F,0x83,0xF0,0x7E,0x0F,0xC1,0xFF,0xFF,0xFF,0xFF,0xFC,0x1F,0x83,0xF0,0x7E,0x0F,0xC1,0xF8,0x38,
// 'I'
0x49,0x02,0x03,0x0F,0x02,0x07,
0xFF,0xFF,0xFF,0xFF,0xFF,0xF8,
// 'J'
0x4A,0x02,0x09,0x0F,0x00,0x0B,
0x03,0x81,0xC0,0xE0,0x70,0x38,0x1C,0x0E,0x07,0x03,0x81,0xC0,0xE8,0xF7,0xF7,0xF9,0xF8,
// 'K'
0x4B,0x02,0x0C,0x0F,0x02,0x0E,
0xE0,0xFE,0x1E,0xE3,0xCE,0x78,0xEF,0x0F,0xE0,0xFC,0x0F,0xE0,0xEE,0x0E,0x70,0xE7,0x8E,0x3C,0xE1,0xCE,0x1E,0xE0,0xF0,
// 'L'
0x4C,0x02,0x0A,0x0F,0x02,0x0C,
0xE0,0x38,0x0E,0x03,0x80,0xE0,0x38,0x0E,0x03,0x80,0xE0,0x38,0x0E,0x03,0x80,0xFF,0xFF,0xFF,0xFC,
// 'M'
0x4D,0x02,0x11,0x0F,0x01,0x13,
0x70,0x07,0x3C,0x07,0x9E,0x03,0xCF,0x83,0xE7,0xC1,0xF3,0xF1,0xFB,0xB8,0xEF,0xCE,0xE7,0xE7,0xF3,0xF1,0xF1,0xF8,0xF8,0xFC,0x38,0x7E,0x1C,0x3F,0x00,0x1F,0x80,0x0E,
// 'N'
0x4E,0x02,0x0C,0x0F,0x02,0x10,
0xE0,0x7F,0x07,0xF0,0x7F,0x87,0xFC,0x7F,0xE7,0xEE,0x7E,0x77,0xE3,0xFE,0x3F,0xE1,0xFE,0x1F,0xE0,0xFE,0x07,0xE0,0x70,
// 'O'
0x4F,0x02,0x0F,0x0F,0x01,0x11,
0x07,0xC0,0x3F,0xE0,0xFF,0xE3,0xC1,0xE7,0x01,0xDC,0x01,0xF8,0x03,0xF0,0x07,0xE0,0x0F,0xC0,0x1D,0xC0,0x73,0xC1,0xE3,0xFF,0x83,0xFE,0x01,0xF0,0x00,
// 'P'
0x50,0x02,0x0B,0x0F,0x02,0x0E,
0xFF,0x1F,0xFB,0xFF,0x70,0xFE,0x0F,0xC1,0xF8,0x7F,0xFE,0xFF,0xDF,0xE3,0x80,0x70,0x0E,0x01,0xC0,0x38,0x00,
// 'Q'
0x51,0x02,0x0F,0x12,0x01,0x11,
0x07,0xC0,0x3F,0xE0,0xFF,0xE3,0xC1,0xE7,0x01,0xDC,0x01,0xF8,0x03,0xF0,0x07,0xE0,0x0F,0xC0,0x1F,0xC0,0x73,0xC1,0xE3,0xFF,0x87,0xFE,0x03,0xF0,0x01,0xE0,0x01,0xFC,0x00,0xF0,
// 'R'
0x52,0x02,0x0C,0x0F,0x02,0x0E,
0xFF,0x0F,0xFC,0xFF,0xCE,0x1E,0xE0,0xEE,0x0E,0xE1,0xEF,0xFC,0xFF,0x8F,0xF0,0xE7,0x8E,0x3C,0xE1,0xCE,0x0E,0xE0,0xF0,
// 'S'
0x53,0x02,0x0A,0x0F,0x01,0x0C,
0x3F,0x1F,0xEF,0xF3,0x84,0xE0,0x3C,0x07,0xE0,0xFE,0x0F,0x80,0xF0,0x1D,0x07,0xFF,0xFF,0xE7,0xF0,
// 'T'
0x54,0x02,0x0B,0x0F,0x00,0x0B,
0xFF,0xFF,0xFF,0xFF,0x87,0x00,0xE0,0x1C,0x03,0x80,0x70,0x0E,0x01,0xC0,0x38,0x07,0x00,0xE0,0x1C,0x03,0x80,
// 'U'
0x55,0x02,0x0B,0x0F,0x02,0x0F,
0xE0,0xFC,0x1F,0x83,0xF0,0x7E,0x0F,0xC1,0xF8,0x3F,0x07,0xE0,0xFC,0x1F,0x83,0xF8,0xF7,0xFC,0xFF,0x07,0xC0,
// 'V'
0x56,0x02,0x0F,0x0F,0x00,0x0F,
0xE0,0x0E,0xE0,0x39,0xC0,0x73,0xC1,0xE3,0x83,0x87,0x07,0x0F,0x1C,0x0E,0x38,0x1C,0x70,0x1D,0xC0,0x3B,0x80,0x7F,0x00,0x7C,0x00,0xF8,0x00,0xE0,0x00,
// 'W'
0x57,0x02,0x13,0x0F,0x00,0x13,
0xE0,0x00,0xFC,0x00,0x1F,0xC3,0x87,0x38,0xF0,0xE7,0x1F,0x1C,0xE3,0xE3,0x9C,0x7C,0x71,0xDD,0xDC,0x3B,0xBB,0x87,0x77,0x70,0xFC,0x7E,0x1F,0x8F,0xC1,0xF1,0xF0,0x3E,0x1E,0x07,0x83,0xC0,
// 'X'
0x58,0x02,0x0E,0x0F,0x00,0x0E,
0xF0,0x3D,0xC0,0xE3,0x87,0x0F,0x3C,0x1F,0xE0,0x3F,0x00,0x78,0x01,0xE0,0x07,0x80,0x3F,0x01,0xFE,0x0F,0x3C,0x38,0x71,0xC0,0xEF,0x03,0xC0,
// 'Y'
0x59,0x02,0x0D,0x0F,0x00,0x0D,
0xF0,0x7B,0x83,0x9E,0x3C,0x71,0xC3,0xDE,0x0E,0xE0,0x7F,0x01,0xF0,0x07,0x00,0x38,0x01,0xC0,0x0E,0x00,0x70,0x03,0x80,0x1C,0x00,
// 'Z'
0x5A,0x02,0x0B,0x0F,0x01,0x0D,
0xFF,0xFF,0xFF,0xFF,0x80,0xE0,0x38,0x0F,0x03,0xC0,0x70,0x1C,0x07,0x81,0xE0,0x38,0x0F,0xFF,0xFF,0xFF,0xF8,
// '['
0x5B,0x01,0x06,0x14,0x02,0x08,
0xFF,0xFF,0xF8,0xE3,0x8E,0x38,0xE3,0x8E,0x38,0xE3,0x8E,0x38,0xE3,0xFF,0xFF,
// '\'
0x5C,0x01,0x0B,0x14,0xFF,0x09,
0xE0,0x0E,0x01,0xC0,0x3C,0x03,0x80,0x70,0x07,0x00,0xE0,0x1C,0x01,0xC0,0x38,0x03,0x80,0x70,0x0E,0x00,0xE0,0x1C,0x03,0xC0,0x38,0x07,0x00,0x70,
// ']'
0x5D,0x01,0x06,0x14,0x00,0x08,
0xFF,0xFF,0xC7,0x1C,0x71,0xC7,0x1C,0x71,0xC7,0x1C,0x71,0xC7,0x1F,0xFF,0xFF,
// '^'
0x5E,0x02,0x0B,0x09,0x00,0x0B,
0x0E,0x03,0xE0,0x7C,0x1F,0xC3,0xB8,0xE3,0x9C,0x77,0x07,0x20,0x80,
// '_'
0x5F,0x12,0x0B,0x03,0x00,0x0B,
0xFF,0xFF,0xFF,0xFF,0x80,
// '`'
0x60,0x00,0x05,0x05,0x01,0x06,
0x47,0x1C,0x71,0x00,
// 'a'
0x61,0x06,0x0A,0x0B,0x01,0x0C,
0x7F,0x1F,0xE7,0xFC,0x07,0x3F,0xDF,0xFE,0x1F,0x87,0xFF,0xDF,0xF3,0xFC,
// 'b'
0x62,0x01,0x0B,0x10,0x01,0x0D,
0xE0,0x1C,0x03,0x80,0x70,0x0E,0x01,0xFE,0x3F,0xF7,0xFE,0xE1,0xFC,0x1F,0x83,0xF0,0x7E,0x1F,0xFF,0xBF,0xE3,0xF8,
// 'c'
0x63,0x06,0x09,0x0B,0x01,0x0B,
0x1F,0x9F,0xDF,0xFE,0x0E,0x07,0x03,0x81,0xE0,0x7F,0x9F,0xC7,0xE0,
// 'd'
0x64,0x01,0x0B,0x10,0x01,0x0D,
0x00,0xE0,0x1C,0x03,0x80,0x70,0x0E,0x3F,0xDF,0xFB,0xFF,0xF0,0xFC,0x1F,0x83,0xF0,0x7F,0x0E,0xFF,0xCF,0xF8,0xFE,
// 'e'
0x65,0x06,0x0A,0x0B,0x01,0x0C,
0x1F,0x0F,0xE7,0xFF,0x87,0xFF,0xFF,0xFE,0x03,0xC0,0x7F,0x9F,0xE1,0xF0,
// 'f'
0x66,0x01,0x08,0x10,0x01,0x08,
0x3E,0x7E,0xFE,0xE0,0xE0,0xFE,0xFE,0xFE,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,
// 'g'
0x67,0x06,0x0A,0x0F,0x01,0x0C,
0x1F,0x9F,0xF7,0xFF,0xC7,0xE1,0xF8,0x7F,0x1F,0xFF,0x7F,0xCF,0xF0,0x1C,0x0F,0x7F,0x9F,0xE7,0xE0,
// 'h'
0x68,0x01,0x0A,0x10,0x01,0x0C,
0xE0,0x38,0x0E,0x03,0x80,0xE0,0x3F,0xCF,0xFB,0xFF,0xE3,0xF8,0x7E,0x1F,0x87,0xE1,0xF8,0x7E,0x1F,0x87,
// 'i'
0x69,0x01,0x05,0x10,0x00,0x05,
0x73,0x9C,0xE0,0x39,0xCE,0x73,0x9C,0xE7,0x39,0xCE,
// 'j'
0x6A,0x01,0x07,0x14,0xFE,0x05,
0x1C,0x38,0x70,0xE0,0x03,0x87,0x0E,0x1C,0x38,0x70,0xE1,0xC3,0x87,0x0E,0x1D,0xFB,0xE7,0x80,
// 'k'
0x6B,0x01,0x0B,0x10,0x01,0x0C,
0xE0,0x1C,0x03,0x80,0x70,0x0E,0x01,0xC7,0xB9,0xE7,0x78,0xFE,0x1F,0x83,0xF8,0x77,0x8E,0x79,0xC7,0x38,0xF7,0x0F,
// 'l'
0x6C,0x01,0x05,0x10,0x01,0x06,
0xE7,0x39,0xCE,0x73,0x9C,0xE7,0x39,0xCF,0x7D,0xE7,
// 'm'
0x6D,0x06,0x0F,0x0B,0x01,0x11,
0x7E,0x79,0xFF,0xFB,0xFF,0xFF,0x3C,0xFE,0x38,0xFC,0x71,0xF8,0xE3,0xF1,0xC7,0xE3,0x8F,0xC7,0x1F,0x8E,0x38,
// 'n'
0x6E,0x06,0x0A,0x0B,0x01,0x0C,
0x7F,0x3F,0xEF,0xFF,0x8F,0xE1,0xF8,0x7E,0x1F,0x87,0xE1,0xF8,0x7E,0x1C,
// 'o'
0x6F,0x06,0x0B,0x0B,0x01,0x0D,
0x1F,0x07,0xF1,0xFF,0x78,0xFE,0x0F,0xC1,0xF8,0x3F,0x8F,0x7F,0xC7,0xF0,0x7C,0x00,
// 'p'
0x70,0x06,0x0B,0x0F,0x01,0x0D,
0x7F,0x1F,0xF3,0xFF,0x70,0xFE,0x0F,0xC1,0xF8,0x3F,0x0F,0xFF,0xDF,0xFB,0xFC,0x70,0x0E,0x01,0xC0,0x38,0x00,
// 'q'
0x71,0x06,0x0B,0x0F,0x01,0x0D,
0x1F,0xC7,0xFD,0xFF,0xF8,0x7E,0x0F,0xC1,0xF8,0x3F,0x87,0x7F,0xEF,0xFC,0x7F,0x80,0x70,0x0E,0x01,0xC0,0x38,
// 'r'
0x72,0x06,0x07,0x0B,0x01,0x08,
0x7F,0xFF,0xFF,0x0E,0x1C,0x38,0x70,0xE1,0xC3,0x80,
// 's'
0x73,0x06,0x09,0x0B,0x00,0x0A,
0x1F,0x1F,0x9F,0xCE,0x07,0xF3,0xFC,0xFE,0x07,0x7F,0xBF,0x9F,0x80,
// 't'
0x74,0x03,0x08,0x0E,0x01,0x09,
0xE0,0xE0,0xE0,0xFE,0xFE,0xFE,0xE0,0xE0,0xE0,0xE0,0xE0,0xFE,0x7E,0x3E,
// 'u'
0x75,0x06,0x0A,0x0B,0x01,0x0C,
0xE1,0xF8,0x7E,0x1F,0x87,0xE1,0xF8,0x7E,0x1F,0xC7,0xFF,0xDF,0xF3,0xF8,
// 'v'
0x76,0x06,0x0B,0x0B,0x00,0x0B,
0xE0,0xFE,0x3D,0xC7,0x38,0xE7,0xBC,0x77,0x0F,0xE0,0xF8,0x1F,0x03,0xE0,0x38,0x00,
// 'w'
0x77,0x06,0x11,0x0B,0x00,0x11,
0xE1,0xC3,0xF0,0xE1,0x9C,0x71,0xCE,0x3C,0xE3,0x3E,0x61,0xDF,0x70,0xFD,0xF8,0x3E,0xF8,0x1E,0x3C,0x0F,0x1E,0x03,0x8E,0x00,
// 'x'
0x78,0x06,0x0C,0x0B,0x00,0x0C,
0xF0,0xF7,0x9E,0x39,0xC1,0xF8,0x1F,0x80,0xF0,0x1F,0x81,0xF8,0x39,0xC7,0x9E,0xF0,0xF0,
// 'y'
0x79,0x06,0x0B,0x0F,0x00,0x0B,
0xE0,0xFE,0x3D,0xC7,0x38,0xE7,0xBC,0x77,0x0E,0xE1,0xF8,0x1F,0x03,0xE0,0x38,0x0F,0x0F,0xC1,0xF8,0x3C,0x00,
// 'z'
0x7A,0x06,0x09,0x0B,0x01,0x0B,
0xFF,0xFF,0xFF,0xE1,0xE1,0xE0,0xE0,0xF0,0xF0,0xFF,0xFF,0xFF,0xE0,
// '{'
0x7B,0x01,0x07,0x14,0x01,0x08,
0x0E,0x3C,0xF9,0xC3,0x87,0x0E,0x1C,0x79,0xE3,0xC7,0xC7,0x87,0x0E,0x1C,0x38,0x7C,0xF8,0x70,
// '|'
0x7C,0x01,0x03,0x14,0x02,0x07,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF0,
// '}'
0x7D,0x01,0x07,0x14,0x00,0x08,
0xE1,0xE3,0xE1,0xC3,0x87,0x0E,0x1C,0x3C,0x3C,0x79,0xF3,0xC7,0x0E,0x1C,0x39,0xF3,0xE7,0x00,
// '~'
0x7E,0x08,0x0B,0x05,0x01,0x0C,
0x38,0x4F,0x9F,0xFF,0x73,0xE4,0x38,

// Terminator
0xFF
};