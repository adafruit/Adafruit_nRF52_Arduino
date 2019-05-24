/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef ADAFRUIT_LITTLEFS_FILE_H_
#define ADAFRUIT_LITTLEFS_FILE_H_

#define FILE_READ 0
#define FILE_WRITE 1

#define FILE_NAME_MAX 255

// Forward declaration
class Adafruit_LittleFS;

namespace LittleFilesystem
{

class File : public Stream
{
  public:
    File (Adafruit_LittleFS &fs);
    File (char const *filename, uint8_t mode, Adafruit_LittleFS &fs);
    File & operator = (const File &rhs);
    virtual ~File ();

    bool open (char const *filename, uint8_t mode);

    //------------- Stream API -------------//
    virtual size_t write (uint8_t ch);
    virtual size_t write (uint8_t const *buf, size_t size);
    size_t write(const char *str) {
      if (str == NULL) return 0;
      return write((const uint8_t *)str, strlen(str));
    }
    size_t write(const char *buffer, size_t size) {
      return write((const uint8_t *)buffer, size);
    }

    virtual int read (void);
    int read (void *buf, uint16_t nbyte);

    virtual int peek (void);
    virtual int available (void);
    virtual void flush (void);

    bool seek (uint32_t pos);
    uint32_t position (void);
    uint32_t size (void);

    void close (void);
    operator bool (void);
    char const* name (void);
    char const* path (void);

    bool isDirectory (void);
    File openNextFile (uint8_t mode = FILE_READ);
    void rewindDirectory (void);

  private:
    Adafruit_LittleFS* _fs;

    lfs_file_t* _hdl; // file hanlde
    lfs_dir_t*  _dir; // dir handle

    char* _path;
    bool _is_dir;

    File _open_file(char const *filepath, uint8_t mode);
    File _open_dir (char const *filepath);

    friend class ::Adafruit_LittleFS;
};

}

#endif /* ADAFRUIT_LITTLEFS_FILE_H_ */
