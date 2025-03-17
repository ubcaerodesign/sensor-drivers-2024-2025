/*
 * filechecking.c
 *
 *  Created on: Mar 16, 2025
 *      Author: ryanz
 */

#include "file_check.h"

// check if file exists
FRESULT fileExists(const TCHAR* path) {
    FRESULT res;
    FIL file;

    res = f_open(&file, path, FA_READ);
    if (res == FR_OK) {
        f_close(&file);
        return FR_OK; // file exists
    }

    return res; // file doesn't exist or other error
}
