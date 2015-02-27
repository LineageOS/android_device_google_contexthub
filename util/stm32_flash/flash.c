#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <malloc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <linux/i2c-dev.h>

#include "stm32_bl.h"
#include "stm32f4_crc.h"
#include "i2c.h"

static inline int pad(int length)
{
    return (length + 3) & ~3;
}

static inline int tot_len(int length)
{
    // [TYPE:1] [LENGTH:3] [DATA] [PAD:0-3] [CRC:4]
    return sizeof(uint32_t) + pad(length) + sizeof(uint32_t);
}

int main(int argc, char *argv[])
{
    uint8_t addr = 0x39;
    char device[] = "/dev/i2c-2";
    int file;
    struct stat buf;
    uint8_t *buffer;
    uint32_t crc;
    i2c_handle_t i2c_handle;
    handle_t *handle = &i2c_handle.handle;
    char options[] = "d:e:w:a:t:r:l:c";
    char *dev = device;
    int opt;
    uint32_t address = 0x08000000;
    char *write_filename = NULL;
    char *read_filename = NULL;
    int sector = -1;
    int do_crc = 0;
    uint8_t type = 0x11;
    ssize_t length = 0;
    uint8_t ret;

    if (argc == 1) {
        printf("Usage: %s\n", argv[0]);
        printf("  -d <device> (i2c device. default: %s)\n", device);
        printf("  -e <sector> (sector to erase.)\n");
        printf("  -w <filename> (filename to write to flash.)\n");
        printf("  -r <filename> (filename to read from flash).\n");
        printf("  -l <length> (length to read/write.)\n");
        printf("  -a <address> (address to write filename to. default: 0x%08x)\n",
               address);
        printf("  -c (add type, length, file contents, and CRC.)\n");
        printf("  -t <type> (type value for -c option. default: %d)\n", type);
        return 0;
    }

    handle->cmd_erase = CMD_ERASE_NS;
    handle->cmd_read_memory = CMD_READ_MEMORY;
    handle->cmd_write_memory = CMD_WRITE_MEMORY_NS;

    handle->write_data = write_data;
    handle->write_cmd = write_cmd;
    handle->read_data = read_data;
    handle->read_ack = read_ack;

    while ((opt = getopt(argc, argv, options)) != -1) {
        switch (opt) {
        case 'd':
            dev = optarg;
            break;
        case 'e':
            sector = strtol(optarg, NULL, 0);
            break;
        case 'w':
            write_filename = optarg;
            break;
        case 'r':
            read_filename = optarg;
            break;
        case 'l':
            length = strtol(optarg, NULL, 0);
            break;
        case 'a':
            address = strtol(optarg, NULL, 0);
            break;
        case 'c':
            do_crc = 1;
            break;
        case 't':
            type = strtol(optarg, NULL, 0);
            break;
        }
    }

    i2c_handle.fd = open(dev, O_RDWR);
    if (i2c_handle.fd < 0) {
        perror("Error opening i2c dev");
        return -1;
    }

    if (ioctl(i2c_handle.fd, I2C_SLAVE, addr) < 0) {
        perror("Error setting slave addr");
        return -1;
    }

    if (sector >= 0) {
        printf("Erasing sector %d\n", sector);
        ret = erase_sector((handle_t *)&i2c_handle, sector);
        if (ret == CMD_ACK)
            printf("Erase succeeded\n");
        else
            printf("Erase failed\n");
    }

    if (write_filename != NULL) {
        file = open(write_filename, O_RDONLY);
        if (file < 0) {
            perror("Error opening input file");
            return -1;
        }

        if (fstat(file, &buf) < 0) {
            perror("error stating file");
            return -1;
        }

        /*
         * For CRC: (when writing to eedata/shared)
         *   [TYPE:1] [LENGTH:3] [DATA] [PAD:0-3] [CRC:4]
         * Otherwise:
         *   [DATA]
         */
        buffer = calloc(tot_len(buf.st_size), 1);
        if (length == 0 || length > buf.st_size)
            length = buf.st_size;

        if (read(file, &buffer[sizeof(uint32_t)], length) < length) {
            perror("Error reading input file");
            return -1;
        }

        printf("Writing %jd bytes from %s @ 0x%08x\n", length,
               write_filename, address);

        if (do_crc) {
            /* Populate TYPE, LENGTH, and CRC */
            buffer[0] = type;
            buffer[1] = (length >> 16) & 0xFF;
            buffer[2] = (length >>  8) & 0xFF;
            buffer[3] = (length      ) & 0xFF;
            crc = ~stm32f4_crc32(buffer, sizeof(uint32_t) + length);

            memcpy(&buffer[sizeof(uint32_t) + pad(length)],
                   &crc, sizeof(uint32_t));

            ret = write_memory((handle_t *)&i2c_handle, address,
                               tot_len(length), buffer);
        } else {
            /* Skip over space reserved for TYPE and LENGTH */
            ret = write_memory((handle_t *)&i2c_handle, address,
                               length, &buffer[sizeof(uint32_t)]);
        }

        if (ret == CMD_ACK)
            printf("Write succeeded\n");
        else
            printf("Write failed\n");

        free(buffer);
        close(file);
    }

    if (read_filename != NULL) {
        file = open(read_filename, O_CREAT | O_TRUNC | O_WRONLY,
                    S_IRUSR | S_IWUSR);
        if (file < 0) {
            perror("Error opening output file");
            return -1;
        }

        if (length > 0) {
            /* If passed in a length, just read that many bytes */
            buffer = calloc(length, 1);

            ret = read_memory((handle_t *)&i2c_handle, address, length, buffer);
            if (ret == CMD_ACK) {
                write(file, buffer, length);

                printf("Read %jd bytes from %s @ 0x%08x\n",
                       length, read_filename, address);
            } else {
                printf("Read failed\n");
            }
            free(buffer);
        } else if (do_crc) {
            /* otherwise if crc specified, read type, length, data, and crc */
            uint8_t tmp_buf[sizeof(uint32_t)];
            ret = read_memory((handle_t *)&i2c_handle, address, sizeof(uint32_t), tmp_buf);
            if (ret == CMD_ACK) {
                type = tmp_buf[0];
                length = ((tmp_buf[1] << 16) & 0x00FF0000) |
                         ((tmp_buf[2] <<  8) & 0x0000FF00) |
                         ((tmp_buf[3]      ) & 0x000000FF);

                if (type != 0xFF) {
                    buffer = calloc(tot_len(length), 1);
                    ret = read_memory((handle_t *)&i2c_handle, address,
                                      tot_len(length), buffer);
                    if (ret == CMD_ACK) {
                        crc = stm32f4_crc32(buffer, tot_len(length));
                        write(file, buffer, tot_len(length));

                        printf("Read %jd bytes from %s @ 0x%08x (type %02x, crc %s)\n",
                               length, read_filename, address, type,
                               crc == STM32F4_CRC_RESIDUE ? "good" : "bad");
                    } else {
                        printf("Read of payload failed\n");
                    }
                    free(buffer);
                } else {
                    printf("Read invalid type: 0xFF\n");
                }
            } else {
                printf("Read of header failed\n");
            }
        } else {
            printf("No length or crc specified for read\n");
        }
        close(file);
    }

    return 0;
}
