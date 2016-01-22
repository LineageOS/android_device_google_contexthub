/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <aes.h>


static FILE* urandom = NULL;


static bool readFile(void *dst, uint32_t len, const char *fileName)
{
    FILE *f = fopen(fileName, "rb");
    bool ret = false;

    if (!f)
        return false;

    if (len != fread(dst, 1, len, f))
        goto out;

    if (fread(&len, 1, 1, f)) //make sure file is actually over
        goto out;

    ret = true;

out:
    fclose(f);
    return ret;
}

static void cleanup(void)
{
    if (urandom)
        fclose(urandom);
}

static void rand_bytes(void *dst, uint32_t len)
{
    if (!urandom) {
        urandom = fopen("/dev/urandom", "rb");
        if (!urandom) {
            fprintf(stderr, "Failed to open /dev/urandom. Cannot procceed!\n");
            exit(-2);
        }

        //it might not matter, but we still like to try to cleanup after ourselves
        (void)atexit(cleanup);
    }

    if (len != fread(dst, 1, len, urandom)) {
        fprintf(stderr, "Failed to read /dev/urandom. Cannot procceed!\n");
        exit(-3);
    }
}

static void appendToBuf(void **inputDataP, uint32_t *inputLenP, uint32_t *inputBufLenP, uint8_t c)
{
    if (*inputLenP == *inputBufLenP) {
        uint32_t newSz = ((uint64_t)*inputBufLenP * 17) / 16 + 1;
        void *ptr = realloc(*inputDataP, newSz);
        if (!ptr) {
            fprintf(stderr, "Failed to realloc a buffer from %lub to %lub\n", (unsigned long)*inputBufLenP, (unsigned long)newSz);
            exit(-5);
        }
        *inputDataP = ptr;
        *inputBufLenP = newSz;
    }
    ((uint8_t*)*inputDataP)[(*inputLenP)++] = c;;
}

static void readInput(void **inputDataP, uint32_t *inputLenP, uint32_t *inputBufLenP)
{
    int c;

    while ((c = getchar()) != EOF)
        appendToBuf(inputDataP, inputLenP, inputBufLenP, c);
}

int main(int argc, char **argv)
{
    uint32_t *inputData = NULL, inputLen = 0, inputBufLen = 0, origLen;
    uint32_t i, j, iv[AES_BLOCK_WORDS], key[AES_KEY_WORDS];
    const char *selfExeName = argv[0];
    struct AesCbcContext ctx;
    uint64_t keyId;
    uint8_t tmp;

    if (argc == 4 && !strcmp(argv[1], "encr")) {

        if (argv[2][0] == '0' && argv[2][1] == 'x')
            keyId = strtoull(argv[2] + 2, NULL, 16);
        else
            keyId = strtoull(argv[2], NULL, 10);

        if (!keyId) {
            fprintf(stderr, "Key ID cannot be zero (given '%s')\n", argv[2]);
            goto usage;
        }

        fprintf(stderr, "Using Key ID of 0x%08lX%08lX\n", (unsigned long)(keyId >> 32), (unsigned long)(keyId & 0xffffffffull));
        rand_bytes(iv, sizeof(iv));
        fprintf(stderr, "Using IV '");
        for (i = 0; i < AES_BLOCK_WORDS; i++)
            fprintf(stderr, "%08lX", (unsigned long)iv[i]);
        fprintf(stderr, "'\n");

        //read key
        if (!readFile(key, sizeof(key), argv[3])) {
            fprintf(stderr, "Key file '%s' does not exist or is not %u bytes\n", argv[3], (unsigned int)sizeof(key));
            goto usage;
        }

        //read data
        readInput((void**)&inputData, &inputLen, &inputBufLen);
        origLen = inputLen;
        fprintf(stderr, "Read %lu bytes\n", (unsigned long)origLen);
        while (inputLen & 15) { //round to 16 bytes
            rand_bytes(&tmp, 1);
            appendToBuf((void**)&inputData, &inputLen, &inputBufLen, tmp);
        }
        fprintf(stderr, "Padded to %lu bytes\n", (unsigned long)inputLen);

        //emit header
        fwrite("Encr", 4, 1, stdout);
        fwrite(&origLen, 4, 1, stdout);
        fwrite(&keyId, 8, 1, stdout);
        for (i = 0; i < AES_BLOCK_WORDS; i++)
            fwrite(iv + i, 4, 1, stdout);

        //encrypt and emit data
        _aesCbcInitForEncr(&ctx, key, iv); //todo
        for (i = 0; i < inputLen / sizeof(uint32_t); i += AES_BLOCK_WORDS) {
            uint32_t out[AES_BLOCK_WORDS];
            _aesCbcEncr(&ctx, inputData + i, out);
            for (j = 0; j < AES_BLOCK_WORDS; j++)
                fwrite(out + j, 4, 1, stdout);
        }

        fprintf(stderr, "Done\n");
    }
    else if (argc == 3 && !strcmp(argv[1], "decr")) {
        //read key
        if (!readFile(key, sizeof(key), argv[2])) {
            fprintf(stderr, "Key file '%s' does not exist or is not %u bytes\n", argv[3], (unsigned int)sizeof(key));
            goto usage;
        }

        //read data
        readInput((void**)&inputData, &inputLen, &inputBufLen);
        origLen = inputLen;
        fprintf(stderr, "Read %lu bytes\n", (unsigned long)origLen);

        if (inputLen < 32) {
            fprintf(stderr, "Implausibly small input\n");
            goto usage;
        }

        //parse header
        if (memcmp(inputData, "Encr", 4)) {
            fprintf(stderr, "Input data lacks 'Encr' header\n");
            goto usage;
        }
        origLen = inputData[1];
        if (origLen > inputLen - 32) {
            fprintf(stderr, "Claimed output size of %lub invalid\n", (unsigned long)origLen);
            goto usage;
        }
        fprintf(stderr, "Original size %lub (%lub of padding present)\n", (unsigned long)origLen, (unsigned long)(inputLen - 32 - origLen));
        keyId = *(uint64_t*)(inputData + 2);
        if (!keyId)  {
            fprintf(stderr, "Input data has invaid key ID\n");
            goto usage;
        }
        fprintf(stderr, "Using Key ID of 0x%08lX%08lX\n", (unsigned long)(keyId >> 32), (unsigned long)(keyId & 0xffffffffull));
        for (i = 0; i < AES_BLOCK_WORDS; i++)
            iv[i] = inputData[4 + i];
        fprintf(stderr, "Using IV '");
        for (i = 0; i < AES_BLOCK_WORDS; i++)
            fprintf(stderr, "%08lX", (unsigned long)iv[i]);
        fprintf(stderr, "'\n");

        //decrypt and emit data
        _aesCbcInitForDecr(&ctx, key, iv); //todo
        for (i = 0; i < (inputLen - 32)/ sizeof(uint32_t); i += AES_BLOCK_WORDS) {
            uint32_t out[AES_BLOCK_WORDS];
            _aesCbcDecr(&ctx, inputData + i + 8, out);
            for (j = 0; j < AES_BLOCK_WORDS; j++) {
                uint32_t now = origLen >= 4 ? 4 : origLen;
                fwrite(out + j, now, 1, stdout);
                origLen -= now;
            }
        }

        fprintf(stderr, "Done\n");
    }
    else
        goto usage;
    free(inputData);
    return 0;

usage:
    fprintf(stderr, "USAGE: %s encr <KEY_ID> <KEY_FILE>< data_to_encr > file_out\n"
                    "       %s decr <KEY_FILE> < data_to_decr > file_out\n",
                    selfExeName, selfExeName);
    free(inputData);
    return -1;
}




