
#include "intc.h"
#include "PmodSF3.h"
#include "xparameters.h"
#include "xspi.h"
#include "xil_cache.h"
#include "xil_printf.h"

#include "ff.h"          /* FatFs */
#include <string.h>
#include <stdlib.h>

/* ---------- user options -------------------------------------------------- */
#define TEST_FILE     "0:/ftou.bin"     /* raw binary file on SD card        */
#define FLASH_BASE    0x000000          /* start address in flash            */
#define USE_QUAD_IO   0                 /* 0 = PAGE_PROGRAM / RANDOM_READ    *///0 for spi and 1 for quad spi
/* -------------------------------------------------------------------------- */

#define MAX_BIN_SIZE   (1024 * 1024)    // 1 MB

#if USE_QUAD_IO
  #define WRITE_CMD  SF3_COMMAND_QUAD_WRITE
  #define READ_CMD   SF3_COMMAND_QUAD_READ
#else
  #define WRITE_CMD  SF3_COMMAND_PAGE_PROGRAM
  #define READ_CMD   SF3_COMMAND_RANDOM_READ
#endif

/* ---------- globals ------------------------------------------------------- */
static FATFS fatfs;
PmodSF3      sf3;

/* ---------- interrupt vector table (unchanged) --------------------------- */
#ifdef XPAR_INTC_0_DEVICE_ID
  #define SF3_SPI_INT_ADDR  XPAR_INTC_0_PMODSF3_0_VEC_ID
#else
  #define SF3_SPI_INT_ADDR  XPAR_FABRIC_PMODSF3_0_QSPI_INTERRUPT_INTR
#endif
const ivt_t ivt[] = {
    { SF3_SPI_INT_ADDR, (XInterruptHandler) XSpi_InterruptHandler, &sf3.SF3Spi }
};

/* ---------- prototypes ---------------------------------------------------- */
/* (copied verbatim from sdtestmod.c) */
static int SD_Init(void);
static int SD_Eject(void);
static int ReadFile(char *FileName, u32 DestinationAddress);
/* new helper */
static XStatus FlashProgramVerify(PmodSF3 *p, u32 addr,
                                  const u8 *src, u32 len);
/* -------------------------------------------------------------------------- */
static void FlashDump(PmodSF3 *p, u32 addr, u32 len);
static u8 bigbuf[MAX_BIN_SIZE];         // 1MB statically reserved buffer
int main(void)
{
    /* ---- init SD card ---------------------------------------------------- */
    if (SD_Init() != XST_SUCCESS) {
        xil_printf("SD init failed\r\n");  return 1;
    }

    /* ---- get file size & malloc buffer ---------------------------------- */
    FILINFO finfo;
    if (f_stat(TEST_FILE, &finfo)) {
        xil_printf("Cannot stat %s\r\n", TEST_FILE);  return 1;
    }
    UINT fsize = finfo.fsize;
//    u8 *buf = malloc(fsize);
//    if (!buf) { xil_printf("malloc failed\r\n"); return 1; }

    if (fsize > MAX_BIN_SIZE) {
        xil_printf("File too big (%u > %u)\r\n", fsize, MAX_BIN_SIZE);
        return 1;
    }
    u8 *buf = bigbuf;

    /* ---- read file into buffer ------------------------------------------ */
    if (ReadFile(TEST_FILE, (u32)buf) != XST_SUCCESS) {
        xil_printf("ReadFile failed\r\n"); free(buf); return 1;
    }
    xil_printf("Loaded %u bytes from SD (%s)\r\n", fsize, TEST_FILE);
    xil_printf("SD first 4 bytes: %02X %02X %02X %02X\r\n",
                  buf[0], buf[1], buf[2], buf[3]);


    /* ---- init Pmod SF3 --------------------------------------------------- */
    if (InitInterruptController(&sf3.sIntc) ||
        SF3_begin(&sf3, &sf3.sIntc, ivt,
                  XPAR_PMODSF3_0_AXI_LITE_SPI_BASEADDR)) {
        xil_printf("SF3 init failed\r\n"); free(buf); return 1;
    }

    /* ---- erase, program, verify ----------------------------------------- */
    XStatus st = FlashProgramVerify(&sf3, FLASH_BASE, buf, fsize);

//    xil_printf((st == XST_SUCCESS) ?
//               "Flash program / verify OK\r\n" :
//               "Flash program / verify FAILED\r\n");
    if (st == XST_SUCCESS) {
        xil_printf("Flash program / verify OK\r\n");
        FlashDump(&sf3, FLASH_BASE, fsize);           /* << NEW << */
    } else {
        xil_printf("Flash program / verify FAILED\r\n");
    }

    /* ---- cleanup --------------------------------------------------------- */
    free(buf);
    SD_Eject();
    return (st == XST_SUCCESS) ? 0 : 1;
}

/* ==========================================================================
 *  SD card helpers  (unchanged from sdtestmod.c except static non static)
 * ==========================================================================*/
static int SD_Init(void)
{
    FRESULT rc = f_mount(&fatfs, "0:/", 0);
    if (rc) { xil_printf("f_mount %d\r\n", rc); return XST_FAILURE; }
    return XST_SUCCESS;
}
static int SD_Eject(void)
{
    FRESULT rc = f_mount(0, "0:/", 0);
    if (rc) { xil_printf("f_unmount %d\r\n", rc); return XST_FAILURE; }
    return XST_SUCCESS;
}
static int ReadFile(char *FileName, u32 DestinationAddress)
{
    FIL fil; UINT br; FRESULT rc;
    rc = f_open(&fil, FileName, FA_READ);
    if (rc) { xil_printf("f_open %d\r\n", rc); return XST_FAILURE; }
    rc = f_read(&fil, (void*)DestinationAddress, f_size(&fil), &br);
    f_close(&fil);
    return rc ? XST_FAILURE : XST_SUCCESS;
}

/* ==========================================================================
 *  Flash helper : erase 64 KB, program page by page, verify
 * ==========================================================================*/
static XStatus FlashProgramVerify(PmodSF3 *pSF3, u32 addr,
                                  const u8 *src, u32 len)
{
    /* 1. Erase one 64 KB sector */
    if (SF3_FlashWriteEnable(pSF3) || SF3_SectorErase(pSF3, addr))
        return XST_FAILURE;

    /* 2. Buffers large enough for command + data */
    u8  wr[SF3_PAGE_SIZE + SF3_WRITE_EXTRA_BYTES];
    u8  rd[SF3_PAGE_SIZE + SF3_READ_MAX_EXTRA_BYTES];

    for (u32 off = 0; off < len; off += SF3_PAGE_SIZE) {

        /* ---- chunk size for this page ---------------------------------- */
        u32 chunk = (len - off > SF3_PAGE_SIZE) ? SF3_PAGE_SIZE : (len - off);

        /* ---- copy user data AFTER the header bytes --------------------- */
        memcpy(&wr[SF3_WRITE_EXTRA_BYTES], &src[off], chunk);

        /* ---- write enable then pageprogram ---------------------------- */
        if (SF3_FlashWriteEnable(pSF3)) return XST_FAILURE;
        u8 *wrPtr = wr;                                /* pointer to header */
        if (SF3_FlashWrite(pSF3, addr + off, chunk, WRITE_CMD, &wrPtr))
            return XST_FAILURE;

        /* ---- read back & verify --------------------------------------- */
        memset(rd, 0, sizeof(rd));
        u8 *rdPtr = rd;
        if (SF3_FlashRead(pSF3, addr + off, chunk, READ_CMD, &rdPtr))
            return XST_FAILURE;

        if (memcmp(rdPtr, &src[off], chunk)) {
            xil_printf("Mismatch @0x%06X\r\n", addr + off);
            return XST_FAILURE;
        }
    }
    return XST_SUCCESS;
}
/* ---- dump the programmed area ------------------------------------------- */
static void FlashDump(PmodSF3 *pSF3, u32 addr, u32 len)
{
    xil_printf("\r\n--- Flash dump %u bytes @0x%06X ---\r\n", len, addr);
    u8 rd[SF3_PAGE_SIZE + SF3_READ_MAX_EXTRA_BYTES];
    for (u32 off = 0; off < len; off += SF3_PAGE_SIZE) {
        u32 chunk = (len - off > SF3_PAGE_SIZE) ? SF3_PAGE_SIZE : (len - off);
        u8 *rp = rd;
        SF3_FlashRead(pSF3, addr + off, chunk, READ_CMD, &rp);

        for (u32 i = 0; i < chunk; ++i) {
            xil_printf("%02X ", rp[i]);
            if (((addr + off + i) & 0x0F) == 0x0F) xil_printf("\r\n");
        }
    }
    xil_printf("\r\n--- end dump ---\r\n");
}
