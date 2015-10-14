#include "../../firmware/inc/cpu/cortexm4f/appRelocFormat.h"
#include <sys/types.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

//This code assumes it is run on a LE CPU with unaligned access abilities. Sorry.


#define APP_MAGIX_0	0x676f6f47
#define APP_MAGIX_1	0x614e656c
#define APP_MAGIX_2	0x70416f6e
#define APP_MAGIX_3	0xffff0070

#define FLASH_BASE	0x10000000
#define RAM_BASE	0x80000000

#define FLASH_SIZE	0x10000000	//256MB ought to be enough for everyone
#define RAM_SIZE	0x10000000	//256MB ought to be enough for everyone

//caution: double evaluation
#define IS_IN_RANGE_E(_val, _rstart, _rend)	(((_val) >= (_rstart)) && ((_val) < (_rend)))
#define IS_IN_RANGE(_val, _rstart, _rsz)	IS_IN_RANGE_E((_val), (_rstart), ((_rstart) + (_rsz)))
#define IS_IN_RAM(_val)				IS_IN_RANGE(_val, RAM_BASE, RAM_SIZE)
#define IS_IN_FLASH(_val)			IS_IN_RANGE(_val, FLASH_BASE, FLASH_SIZE)


#define NANO_RELOC_TYPE_RAM	0
#define NANO_RELOC_TYPE_FLASH	1
#define NANO_RELOC_LAST		2 //must be <= (RELOC_TYPE_MASK >> RELOC_TYPE_SHIFT)


struct AppHeader {
	uint32_t magic[4];

	uint32_t appID[2];

	uint32_t __data_start;
	uint32_t __data_end;
	uint32_t __data_data;

	uint32_t __bss_start;
	uint32_t __bss_end;

	uint32_t __got_start;
	uint32_t __got_end;
	uint32_t __rel_start;
	uint32_t __rel_end;

	uint32_t start_task;
	uint32_t end_task;
	uint32_t handle_event;
};

struct RelocEntry {
	uint32_t where;
	uint32_t info;	//bottom 8 bits is type, top 24 is sym idx
};

#define RELOC_TYPE_ABS_S	2
#define RELOC_TYPE_ABS_D	21
#define RELOC_TYPE_SECT		23


struct SymtabEntry {
	uint32_t a;
	uint32_t addr;
	uint32_t b, c;
};



struct NanoRelocEntry {
	uint32_t ofstInRam;
	uint8_t type;
};


int main(int argc, char **argv)
{
	uint32_t i, numRelocs, numSyms, outNumRelocs = 0, packedNanoRelocSz, j, k, lastOutType = 0, origin = 0;
	struct NanoRelocEntry *nanoRelocs = NULL;
	struct RelocEntry *relocs;
	struct SymtabEntry *syms;
	uint8_t *packedNanoRelocs;
	uint32_t t, bufUsed = 0;
	struct AppHeader *hdr;
	bool verbose = false;
	uint8_t *buf = NULL;
	uint32_t bufSz = 0;
	uint64_t appId = 0;
	int c, ret = -1;


	argc--;
	argv++;

	for (i = 0; i < argc; i++) {
		if (!strcmp(argv[i], "-v")) {
			verbose = true;
			continue;
		}
		appId = strtoul(argv[i], NULL, 16);
	}

	if (!appId) {
		fprintf(stderr, "USAGE: %s [-v] 0123456789abcdef < app.bin >app.ap\n\twhere 0123456789abcdef is app ID in hex\n", argv[-1]);
		return -2;
	}

	// read file
	while ((c = getchar()) != EOF) {
		if (bufSz == bufUsed) {
			uint8_t *t;
			bufSz = (bufSz * 5) / 4 + 1;
			t = realloc(buf, bufSz);
			if (!t) {
				fprintf(stderr, "Realloc to %u fails - you're SOL\n", (unsigned)bufSz);
				goto out;
			}
			buf = t;
		}
		buf[bufUsed++] = c;
	}

	//make buffer bigger by 50% in case relocs grow out of hand
	buf = realloc(buf, 3 * bufUsed / 2);
	if (!buf) {
		fprintf(stderr, "MEMERR\n");
		exit(-7);
	}

	//sanity checks
	hdr = (struct AppHeader*)buf;
	if (bufUsed < sizeof(struct AppHeader)) {
		fprintf(stderr, "File size too small\n");
		goto out;
	}

	if (hdr->magic[0] != APP_MAGIX_0 || hdr->magic[1] != APP_MAGIX_1 || hdr->magic[2] != APP_MAGIX_2 || hdr->magic[3] != APP_MAGIX_3) {
		fprintf(stderr, "Magic value wrong\n");
		goto out;
	}
	//do some math
	relocs = (struct RelocEntry*)(buf + hdr->__rel_start - FLASH_BASE);
	syms = (struct SymtabEntry*)(buf + hdr->__rel_end - FLASH_BASE);
	numRelocs = (hdr->__rel_end - hdr->__rel_start) / sizeof(struct RelocEntry);
	numSyms = (bufUsed + FLASH_BASE - hdr->__rel_end) / sizeof(struct SymtabEntry);

	//sanity
	if (numRelocs * sizeof(struct RelocEntry) + hdr->__rel_start != hdr->__rel_end) {
		fprintf(stderr, "Relocs of nonstandard size\n");
		goto out;
	}
	if (numSyms * sizeof(struct SymtabEntry) + hdr->__rel_end != bufUsed + FLASH_BASE) {
		fprintf(stderr, "Syms of nonstandard size\n");
		goto out;
	}

	//show some info
	fprintf(stderr, "\nRead %u bytes of binary.\n", (unsigned)bufUsed);

	if (verbose)
		fprintf(stderr, "Found %u relocs and a %u-entry symbol table\n", numRelocs, numSyms);

	//handle relocs
	nanoRelocs = malloc(sizeof(struct NanoRelocEntry[numRelocs]));
	if (!nanoRelocs) {
		fprintf(stderr, "Failed to allocate a nano-reloc table\n");
		goto out;
	}

	for (i = 0; i < numRelocs; i++) {
		uint32_t relocType = relocs[i].info & 0xff;
		uint32_t whichSym = relocs[i].info >> 8;
		uint32_t *valThereP;

		if (whichSym >= numSyms) {
			fprintf(stderr, "Reloc %u references a nonexistent symbol!\nINFO:\n\tWhere: 0x%08X\n\ttype: %u\n\tsym: %u\n", 
				i, relocs[i].where, relocs[i].info & 0xff, whichSym);
			goto out;
		}

		if (verbose) {

			fprintf(stderr, "Reloc[%3u]:\n {@0x%08X, type %3d, -> sym[%3u]: {@0x%08x}, ",
				i, relocs[i].where, relocs[i].info & 0xff, whichSym, syms[whichSym].addr);

			if (IS_IN_RANGE_E(relocs[i].where, hdr->__bss_start, hdr->__bss_end))
				fprintf(stderr, "in   .bss}\n");
			else if (IS_IN_RANGE_E(relocs[i].where, hdr->__data_start, hdr->__data_end))
				fprintf(stderr, "in  .data}\n");
			else if (IS_IN_RANGE_E(relocs[i].where, hdr->__got_start, hdr->__got_end))
				fprintf(stderr, "in   .got}\n");
			else if (IS_IN_RANGE_E(relocs[i].where, FLASH_BASE, FLASH_BASE + sizeof(struct AppHeader)))
				fprintf(stderr, "in APPHDR}\n");
			else
				fprintf(stderr, "in	???}\n");

		}
		/* handle relocs inside the header */
		if (IS_IN_FLASH(relocs[i].where) && relocs[i].where - FLASH_BASE < sizeof(struct AppHeader) && relocType == RELOC_TYPE_SECT) {
			/* relocs in header are special - runtime corrects for them */
			if (syms[whichSym].addr) {
				fprintf(stderr, "Weird in-header sect reloc %u to symbol %u with nonzero addr 0x%08x\n", i, whichSym, syms[whichSym].addr);
				goto out;
			}

			valThereP = (uint32_t*)(buf + relocs[i].where - FLASH_BASE);
			if (!IS_IN_FLASH(*valThereP)) {
				fprintf(stderr, "In-header reloc %u of location 0x%08X is outside of FLASH!\nINFO:\n\ttype: %u\n\tsym: %u\n\tSym Addr: 0x%08X\n", 
					i, relocs[i].where, relocType, whichSym, syms[whichSym].addr);
				goto out;
			}

			*valThereP -= FLASH_BASE;

			if (verbose)
				fprintf(stderr, "  -> Nano reloc skipped for in-header reloc\n");

			continue; /* do not produce an output reloc */
		}

		if (!IS_IN_RAM(relocs[i].where)) {
			fprintf(stderr, "Reloc %u of location 0x%08X is outside of RAM!\nINFO:\n\ttype: %u\n\tsym: %u\n\tSym Addr: 0x%08X\n", 
				i, relocs[i].where, relocType, whichSym, syms[whichSym].addr);
			goto out;
		}

		valThereP = (uint32_t*)(buf + relocs[i].where + hdr-> __data_data - RAM_BASE - FLASH_BASE);

		nanoRelocs[outNumRelocs].ofstInRam = relocs[i].where - RAM_BASE;

		switch (relocType) {
			case RELOC_TYPE_ABS_S:
			case RELOC_TYPE_ABS_D:
				t = *valThereP;

				(*valThereP) += syms[whichSym].addr;

				if (IS_IN_FLASH(syms[whichSym].addr)) {
					(*valThereP) -= FLASH_BASE;
					nanoRelocs[outNumRelocs].type = NANO_RELOC_TYPE_FLASH;
				}
				else if (IS_IN_RAM(syms[whichSym].addr)) {
					(*valThereP) -= RAM_BASE;
					nanoRelocs[outNumRelocs].type = NANO_RELOC_TYPE_RAM;
				}
				else {
					fprintf(stderr, "Weird reloc %u to symbol %u in unknown memory space (addr 0x%08x)\n", i, whichSym, syms[whichSym].addr);
					goto out;
				}
				if (verbose)
					fprintf(stderr, "  -> Abs reference fixed up 0x%08X -> 0x%08X\n", t, *valThereP);
				break;

			case RELOC_TYPE_SECT:
				if (syms[whichSym].addr) {
					fprintf(stderr, "Weird sect reloc %u to symbol %u with nonzero addr 0x%08x\n", i, whichSym, syms[whichSym].addr);
					goto out;
				}

				t = *valThereP;

				if (IS_IN_FLASH(*valThereP)) {
					nanoRelocs[outNumRelocs].type = NANO_RELOC_TYPE_FLASH;
					*valThereP -= FLASH_BASE;
				}
				else if (IS_IN_RAM(*valThereP)) {
					nanoRelocs[outNumRelocs].type = NANO_RELOC_TYPE_RAM;
					*valThereP -= RAM_BASE;
				}
				else {
					fprintf(stderr, "Weird sec reloc %u to symbol %u in unknown memory space (addr 0x%08x)\n", i, whichSym, *valThereP);
					goto out;
				}
				if (verbose)
					fprintf(stderr, "  -> Sect reference fixed up 0x%08X -> 0x%08X\n", t, *valThereP);
				break;

			default:
				fprintf(stderr, "Weird reloc %u type %u to symbol %u\n", i, relocType, whichSym);
				goto out;
		}

		if (verbose)
			fprintf(stderr, "  -> Nano reloc calculated as 0x%08X,0x%02x\n", nanoRelocs[i].ofstInRam, nanoRelocs[i].type);
		outNumRelocs++;
	}

	//sort by type and then offset
        for (i = 0; i < outNumRelocs; i++) {
		struct NanoRelocEntry t;

		for (k = i, j = k + 1; j < outNumRelocs; j++) {
			if (nanoRelocs[j].type > nanoRelocs[k].type)
				continue;
			if ((nanoRelocs[j].type < nanoRelocs[k].type) || (nanoRelocs[j].ofstInRam < nanoRelocs[k].ofstInRam))
				k = j;
		}
		memcpy(&t, nanoRelocs + i, sizeof(struct NanoRelocEntry));
		memcpy(nanoRelocs + i, nanoRelocs + k, sizeof(struct NanoRelocEntry));
		memcpy(nanoRelocs + k, &t, sizeof(struct NanoRelocEntry));

		if (verbose)
			fprintf(stderr, "SortedReloc[%3u] = {0x%08X,0x%02X}\n", i, nanoRelocs[i].ofstInRam, nanoRelocs[i].type);
	}

	//produce output nanorelocs in packed format
	packedNanoRelocs = malloc(outNumRelocs * 6); //definitely big enough
	packedNanoRelocSz = 0;
        for (i = 0; i < outNumRelocs; i++) {

		uint32_t displacement;

		if (lastOutType != nanoRelocs[i].type) {		//output type if ti changed
			if (nanoRelocs[i].type - lastOutType == 1) {
				packedNanoRelocs[packedNanoRelocSz++] = TOKEN_RELOC_TYPE_NEXT;
				if (verbose)
					fprintf(stderr, "Out: RelocTC (1) // to 0x%02X\n", nanoRelocs[i].type);
			}
			else {
				packedNanoRelocs[packedNanoRelocSz++] = TOKEN_RELOC_TYPE_CHG;
				packedNanoRelocs[packedNanoRelocSz++] = nanoRelocs[i].type - lastOutType - 1;
				if (verbose)
					fprintf(stderr, "Out: RelocTC (0x%02X)\n", nanoRelocs[i].type - lastOutType - 1, nanoRelocs[i].type);
			}
			lastOutType = nanoRelocs[i].type;
			origin = 0;
		}
		displacement = nanoRelocs[i].ofstInRam - origin;
		origin = nanoRelocs[i].ofstInRam + 4;
		if (displacement & 3) {
			fprintf(stderr, "Unaligned relocs are not possible!\n");
			exit(-5);
		}
		displacement /= 4;

		//might be start of a run. look into that
		if (!displacement) {
			for (j = 1; j + i < outNumRelocs && j < MAX_RUN_LEN && nanoRelocs[j + i].type == lastOutType && nanoRelocs[j + i].ofstInRam - nanoRelocs[j + i - 1].ofstInRam == 4; j++);
			if (j >= MIN_RUN_LEN) {
				if (verbose)
					fprintf(stderr, "Out: Reloc0  x%u\n", j);
				packedNanoRelocs[packedNanoRelocSz++] = TOKEN_CONSECUTIVE;
				packedNanoRelocs[packedNanoRelocSz++] = j - MIN_RUN_LEN;
				origin = nanoRelocs[j + i - 1].ofstInRam + 4;	//reset origin to last one
				i += j - 1;	//loop will increment anyways, hence +1
				continue;
			}
		}

		//produce output
		if (displacement <= MAX_8_BIT_NUM) {
			if (verbose)
				fprintf(stderr, "Out: Reloc8  0x%02X\n", displacement);
			packedNanoRelocs[packedNanoRelocSz++] = displacement;
		}
		else if (displacement <= MAX_16_BIT_NUM) {
			if (verbose)
				fprintf(stderr, "Out: Reloc16 0x%06X\n", displacement);
                        displacement -= MAX_8_BIT_NUM;
			packedNanoRelocs[packedNanoRelocSz++] = TOKEN_16BIT_OFST;
			packedNanoRelocs[packedNanoRelocSz++] = displacement;
			packedNanoRelocs[packedNanoRelocSz++] = displacement >> 8;
		}
		else if (displacement <= MAX_24_BIT_NUM) {
			if (verbose)
				fprintf(stderr, "Out: Reloc24 0x%08X\n", displacement);
                        displacement -= MAX_16_BIT_NUM;
			packedNanoRelocs[packedNanoRelocSz++] = TOKEN_24BIT_OFST;
			packedNanoRelocs[packedNanoRelocSz++] = displacement;
			packedNanoRelocs[packedNanoRelocSz++] = displacement >> 8;
			packedNanoRelocs[packedNanoRelocSz++] = displacement >> 16;
		}
		else  {
			if (verbose)
				fprintf(stderr, "Out: Reloc32 0x%08X\n", displacement);
			packedNanoRelocs[packedNanoRelocSz++] = TOKEN_32BIT_OFST;
			packedNanoRelocs[packedNanoRelocSz++] = displacement;
			packedNanoRelocs[packedNanoRelocSz++] = displacement >> 8;
			packedNanoRelocs[packedNanoRelocSz++] = displacement >> 16;
			packedNanoRelocs[packedNanoRelocSz++] = displacement >> 24;
		}
	}

	//put in app id
	hdr->appID[0] = appId;
	hdr->appID[1] = appId >> 32;

	//overwrite original relocs and symtab with nanorelocs and adjust sizes
	memcpy(relocs, packedNanoRelocs, packedNanoRelocSz);
	bufUsed -= sizeof(struct RelocEntry[numRelocs]);
	bufUsed -= sizeof(struct SymtabEntry[numSyms]);
	bufUsed += packedNanoRelocSz;
	hdr->__rel_end = hdr->__rel_start + packedNanoRelocSz;

	//sanity
	if (hdr->__rel_end - FLASH_BASE != bufUsed) {
		fprintf(stderr, "Relocs end and file end not coincident\n");
		goto out;
	}

	//adjust headers for easy access (RAM)
	if (!IS_IN_RAM(hdr->__data_start) || !IS_IN_RAM(hdr->__data_end) || !IS_IN_RAM(hdr->__bss_start) || !IS_IN_RAM(hdr->__bss_end) || !IS_IN_RAM(hdr->__got_start) || !IS_IN_RAM(hdr->__got_end)) {
		fprintf(stderr, "data, bss, or got not in ram\n");
		goto out;
	}
	hdr->__data_start -= RAM_BASE;
	hdr->__data_end -= RAM_BASE;
	hdr->__bss_start -= RAM_BASE;
	hdr->__bss_end -= RAM_BASE;
	hdr->__got_start -= RAM_BASE;
	hdr->__got_end -= RAM_BASE;

	//adjust headers for easy access (FLASH)
	if (!IS_IN_FLASH(hdr->__data_data) || !IS_IN_FLASH(hdr->__rel_start) || !IS_IN_FLASH(hdr->__rel_end)) {
		fprintf(stderr, "data.data, or rel not in ram\n");
		goto out;
	}
	hdr->__data_data -= FLASH_BASE;
	hdr->__rel_start -= FLASH_BASE;
	hdr->__rel_end -= FLASH_BASE;

	//if we have any bytes to output, show stats
	if (bufUsed) {
		uint32_t codeAndRoDataSz = hdr->__data_data;
		uint32_t relocsSz = hdr->__rel_end - hdr->__rel_start;
		uint32_t gotSz = hdr->__got_end - hdr->__data_start;
		uint32_t bssSz = hdr->__bss_end - hdr->__bss_start;

		fprintf(stderr,"Final binary size %u bytes\n", bufUsed);
		fprintf(stderr, "\n");
		fprintf(stderr, "\tCode + RO data (flash):	     %6u bytes\n", (unsigned)codeAndRoDataSz);
		fprintf(stderr, "\tRelocs (flash):		     %6u bytes\n", (unsigned)relocsSz);
		fprintf(stderr, "\tGOT + RW data (flash & RAM): %6u bytes\n", (unsigned)gotSz);
		fprintf(stderr, "\tBSS (RAM):		     %6u bytes\n", (unsigned)bssSz);
		fprintf(stderr, "\n");
		fprintf(stderr,"Runtime flash use: %u bytes\n", codeAndRoDataSz + relocsSz + gotSz);
		fprintf(stderr,"Runtime RAM use: %u bytes\n", gotSz + bssSz);
	}

	//output the data
	for (i = 0; i < bufUsed; i++)
		putchar(buf[i]);

	//success!
	ret = 0;

out:
	free(nanoRelocs);
	free(buf);
	return ret;
}



