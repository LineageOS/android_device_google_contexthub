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

#define NANO_RELOC_TYPE_RAM	0
#define NANO_RELOC_TYPE_FLASH	1


struct NanoRelocEntry {
	uint32_t info;	//bottom 28 bits are ram offset, top 4 are type
};

int main(int argc, char **argv)
{
	uint32_t i, numRelocs, numSyms, outNumRelocs = 0;
	struct NanoRelocEntry *nanoRelocs = NULL;
	struct RelocEntry *relocs;
	struct SymtabEntry *syms;
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
				fprintf(stderr, "in    ???}\n");

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

		nanoRelocs[outNumRelocs].info = relocs[i].where - RAM_BASE;

		switch (relocType) {
			case RELOC_TYPE_ABS_S:
			case RELOC_TYPE_ABS_D:
				t = *valThereP;

				(*valThereP) += syms[whichSym].addr;

				if (IS_IN_FLASH(syms[whichSym].addr)) {
					(*valThereP) -= FLASH_BASE;
					nanoRelocs[outNumRelocs].info |= NANO_RELOC_TYPE_FLASH << 28;
				}
				else if (IS_IN_RAM(syms[whichSym].addr)) {
					(*valThereP) -= RAM_BASE;
					nanoRelocs[outNumRelocs].info |= NANO_RELOC_TYPE_RAM << 28;
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
					nanoRelocs[outNumRelocs].info |= NANO_RELOC_TYPE_FLASH << 28;
					*valThereP -= FLASH_BASE;
				}
				else if (IS_IN_RAM(*valThereP)) {
					nanoRelocs[outNumRelocs].info |= NANO_RELOC_TYPE_RAM << 28;
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
			fprintf(stderr, "  -> Nano reloc calculated as 0x%08X\n", nanoRelocs[i].info);
		outNumRelocs++;
	}

	//put in app id
	hdr->appID[0] = appId;
	hdr->appID[1] = appId >> 32;

	//overwrite original relocs and symtab with nanorelocs and adjust sizes
	memcpy(relocs, nanoRelocs, sizeof(struct NanoRelocEntry[outNumRelocs]));
	bufUsed -= sizeof(struct RelocEntry[numRelocs]);
	bufUsed -= sizeof(struct SymtabEntry[numSyms]);
	bufUsed += sizeof(struct NanoRelocEntry[outNumRelocs]);
	hdr->__rel_end = hdr->__rel_start + sizeof(struct NanoRelocEntry[outNumRelocs]);

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
		uint32_t relocsSz = sizeof(struct NanoRelocEntry[outNumRelocs]);
		uint32_t gotSz = hdr->__got_end - hdr->__data_start;
		uint32_t bssSz = hdr->__bss_end - hdr->__bss_start;

		fprintf(stderr,"Final binary size %u bytes\n", bufUsed);
		fprintf(stderr, "\n");
		fprintf(stderr, "\tCode + RO data (flash):      %6u bytes\n", (unsigned)codeAndRoDataSz);
		fprintf(stderr, "\tRelocs (flash):              %6u bytes\n", (unsigned)relocsSz);
		fprintf(stderr, "\tGOT + RW data (flash & RAM): %6u bytes\n", (unsigned)gotSz);
		fprintf(stderr, "\tBSS (RAM):                   %6u bytes\n", (unsigned)bssSz);
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




