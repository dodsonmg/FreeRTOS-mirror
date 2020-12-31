/*
 * FreeRTOS+FAT build 191128 - Note:  FreeRTOS+FAT is still in the lab!
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Authors include James Walmsley, Hein Tibosch and Richard Barry
 *
 * Copyright (c) 2020 Hesham Almatary
 *
 * This software was partly developed by SRI International and the University of
 * Cambridge Computer Laboratory (Department of Computer Science and
 * Technology) under DARPA contract HR0011-18-C-0016 ("ECATS"), as part of the
 * DARPA SSITH research programme.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* FreeRTOS+FAT includes. */
#include "ff_headers.h"
#include "ff_stdio.h"
#include "ff_virtioblk_disk.h"

/* FreeRTOS-libvirtio includes. */
#include "virtio.h"
#include "virtio-blk.h"
#include "helpers.h"

/* The size of each sector on the disk. */
#define virtio_blkSECTOR_SIZE                512UL

/* Only a single partition is used.  Partition numbers start at 0. */
#define virtio_blkPARTITION_NUMBER            0
#define virtio_blkHIDDEN_SECTOR_COUNT      8
#define virtio_blkPRIMARY_PARTITIONS       1
#define virtio_blkHUNDRED_64_BIT           100ULL
#define virtio_blkSECTOR_SIZE              512UL
#define virtio_blkBYTES_PER_KB             ( 1024ull )
#define virtio_blkSECTORS_PER_KB           ( virtio_blkBYTES_PER_KB / 512ull )

BaseType_t FF_VirtIODiskShowPartition( FF_Disk_t *pxDisk )
{
FF_Error_t xError;
uint64_t ullFreeSectors;
uint32_t ulTotalSizeKB, ulFreeSizeKB;
int iPercentageFree;
FF_IOManager_t *pxIOManager;
const char *pcTypeName = "unknown type";
BaseType_t xReturn = pdPASS;

    if( pxDisk == NULL )
    {
        xReturn = pdFAIL;
    }
    else
    {
        pxIOManager = pxDisk->pxIOManager;

        FF_PRINTF( "Reading FAT and calculating Free Space\n" );

        switch( pxIOManager->xPartition.ucType )
        {
            case FF_T_FAT12:
                pcTypeName = "FAT12";
                break;

            case FF_T_FAT16:
                pcTypeName = "FAT16";
                break;

            case FF_T_FAT32:
                pcTypeName = "FAT32";
                break;

            default:
                pcTypeName = "UNKOWN";
                break;
        }

        FF_GetFreeSize( pxIOManager, &xError );

        ullFreeSectors = pxIOManager->xPartition.ulFreeClusterCount * pxIOManager->xPartition.ulSectorsPerCluster;
        if( pxIOManager->xPartition.ulDataSectors == ( uint32_t )0 )
        {
            iPercentageFree = 0;
        }
        else
        {
            iPercentageFree = ( int ) ( ( virtio_blkHUNDRED_64_BIT * ullFreeSectors + pxIOManager->xPartition.ulDataSectors / 2 ) /
                ( ( uint64_t )pxIOManager->xPartition.ulDataSectors ) );
        }

        ulTotalSizeKB = pxIOManager->xPartition.ulDataSectors / virtio_blkSECTORS_PER_KB;
        ulFreeSizeKB = ( uint32_t ) ( ullFreeSectors / virtio_blkSECTORS_PER_KB );

        /* It is better not to use the 64-bit format such as %Lu because it
        might not be implemented. */
        FF_PRINTF( "Partition Nr   %8u\n", pxDisk->xStatus.bPartitionNumber );
        FF_PRINTF( "Type           %8u (%s)\n", pxIOManager->xPartition.ucType, pcTypeName );
        FF_PRINTF( "VolLabel       '%8s' \n", pxIOManager->xPartition.pcVolumeLabel );
        FF_PRINTF( "TotalSectors   %8lu\n", pxIOManager->xPartition.ulTotalSectors );
        FF_PRINTF( "SecsPerCluster %8lu\n", pxIOManager->xPartition.ulSectorsPerCluster );
        FF_PRINTF( "Size           %8lu KB\n", ulTotalSizeKB );
        FF_PRINTF( "FreeSize       %8lu KB ( %d perc free )\n", ulFreeSizeKB, iPercentageFree );
    }

    return xReturn;
}
/*-----------------------------------------------------------*/

static FF_Error_t prvPartitionAndFormatDisk( FF_Disk_t *pxDisk )
{
FF_PartitionParameters_t xPartition;
FF_Error_t xError;

    /* Create a single partition that fills all available space on the disk. */
    memset( &xPartition, '\0', sizeof( xPartition ) );
    xPartition.ulSectorCount = pxDisk->ulNumberOfSectors;
    xPartition.ulHiddenSectors = virtio_blkHIDDEN_SECTOR_COUNT;
    xPartition.xPrimaryCount = virtio_blkPRIMARY_PARTITIONS;
    xPartition.eSizeType = eSizeIsQuota;

    /* Partition the disk */
    xError = FF_Partition( pxDisk, &xPartition );
    FF_PRINTF( "FF_Partition: %s\n", ( const char * ) FF_GetErrMessage( xError ) );

    if( FF_isERR( xError ) == pdFALSE )
    {
        /* Format the partition. */
        xError = FF_Format( pxDisk, virtio_blkPARTITION_NUMBER, pdFALSE, pdFALSE );
        FF_PRINTF( "FF_VirtIODiskInit: FF_Format: %s\n", ( const char * ) FF_GetErrMessage( xError ) );
    }

    return xError;
}
/*-----------------------------------------------------------*/

BaseType_t FF_VirtIODiskDelete( FF_Disk_t *pxDisk )
{
    if( pxDisk != NULL )
    {
        pxDisk->ulSignature = 0;
        pxDisk->xStatus.bIsInitialised = 0;
        if( pxDisk->pxIOManager != NULL )
        {
            FF_DeleteIOManager( pxDisk->pxIOManager );
        }

        vPortFree( pxDisk );
    }

    return pdPASS;
}
/*-----------------------------------------------------------*/

static int32_t prvReadVirtIO( uint8_t *pucDestination,
                           uint32_t ulSectorNumber,
                           uint32_t ulSectorCount,
                           FF_Disk_t *pxDisk )
{
size_t cnt;
    /* The FF_Disk_t structure describes the media being accessed.  Attributes that
    are common to all media types are stored in the structure directly.  The pvTag
    member of the structure is used to add attributes that are specific to the media
    actually being accessed.  In the case of the VirtIO disk the pvTag member is
    used to point to the virtio_device being used as the disk. */

    cnt = virtioblk_transfer((struct virtio_device *) pxDisk->pvTag, pucDestination, ulSectorNumber, ulSectorCount, VIRTIO_BLK_T_IN);

    if (cnt == ulSectorCount)
        return FF_ERR_NONE;
    else
        return FF_ERR_DEVICE_DRIVER_FAILED;
}
/*-----------------------------------------------------------*/

static int32_t prvWriteVirtIO( uint8_t *pucSource,
                            uint32_t ulSectorNumber,
                            uint32_t ulSectorCount,
                            FF_Disk_t *pxDisk )
{
size_t cnt;

    /* The FF_Disk_t structure describes the media being accessed.  Attributes that
    are common to all media types are stored in the structure directly.  The pvTag
    member of the structure is used to add attributes that are specific to the media
    actually being accessed.  In the case of the VirtIO disk the pvTag member is
    used to point to the virtio_device being used as the disk. */

    /* Copy the data to the disk. */
    cnt = virtioblk_transfer((struct virtio_device *) pxDisk->pvTag, pucSource, ulSectorNumber, ulSectorCount, VIRTIO_BLK_T_OUT);

    if (cnt == ulSectorCount)
        return FF_ERR_NONE;
    else
        return FF_ERR_DEVICE_DRIVER_FAILED;
}
/*-----------------------------------------------------------*/

/*
  – pcName is the name to give the disk within FreeRTOS+FAT’s virtual file system.
  – xIOManagerCacheSize is the size of the IO manager’s cache, which must be a
   multiple of the sector size, and at least twice as big as the sector size.
*/
FF_Disk_t *FF_VirtIODiskInit( char *pcName,
                              size_t xIOManagerCacheSize )
{
FF_Error_t xError;
FF_Disk_t *pxDisk = NULL;
FF_CreationParameters_t xParameters;
uint64_t ulCapacity = 0;
uint32_t ulSectorSize = 0;
struct virtio_device* blk_dev = NULL;

    /* Check the validity of the xIOManagerCacheSize parameter. */
    configASSERT( ( xIOManagerCacheSize % virtio_blkSECTOR_SIZE ) == 0 );
    configASSERT( ( xIOManagerCacheSize >= ( 2 * virtio_blkSECTOR_SIZE ) ) );

    blk_dev = virtio_setup_vd( (void *) VIRTIO_BLK_MMIO_ADDRESS);
    if ( blk_dev == NULL )
    {
        FF_PRINTF("Failed to initialize virtio-blk device\n");
        return NULL;
    }

    /* virtioblk_init return the sector size if succeeded */
    ulSectorSize = virtioblk_init(blk_dev);

    /* Read the capacity of the device which is the numbers of sectors */
    ulCapacity = virtio_get_config(blk_dev,
            offset_of(struct virtio_blk_cfg, capacity),
            sizeof(ulCapacity));

    if ( ( ulSectorSize == 0 ) || (ulCapacity <= 0) )
    {
        FF_PRINTF( "FF_VirtIODiskInit failed as device has %d:%d sectorSize:Capacity\n", ulSectorSize, ulCapacity );
        return NULL;
    }

    /* Attempt to allocated the FF_Disk_t structure. */
    pxDisk = ( FF_Disk_t * ) pvPortMalloc( sizeof( FF_Disk_t ) );

    if( pxDisk != NULL )
    {
        /* It is advisable to clear the entire structure to zero after it has been
        allocated – that way the media driver will be compatible with future
        FreeRTOS+FAT versions, in which the FF_Disk_t structure may include
        additional members. */
        memset( pxDisk, 0, sizeof( FF_Disk_t ) );

        /* The pvTag member of the FF_Disk_t structure allows the structure to be
        extended to also include media specific parameters.  The only media
        specific data that needs to be stored in the FF_Disk_t structure for a
        VirtIO disk is the location of the virtio_device struct – so this is stored
        directly in the FF_Disk_t’s pvTag member. */
        pxDisk->pvTag = ( void * ) blk_dev;

        /* The signature is used by the disk read and disk write functions to
        ensure the disk being accessed is a virtio-blk disk. */
        pxDisk->ulSignature = 0x74726976; // "virt"

        /* The number of sectors is recorded for bounds checking in the read and
        write functions. */
        pxDisk->ulNumberOfSectors = ulCapacity;

        /* Create the IO manager that will be used to control the VirtIO disk –
        the FF_CreationParameters_t structure completed with the required
        parameters, then passed into the FF_CreateIOManager() function. */
        memset (&xParameters, 0, sizeof xParameters);
        xParameters.pucCacheMemory = NULL;
        xParameters.ulMemorySize = xIOManagerCacheSize;
        xParameters.ulSectorSize = ulSectorSize;
        xParameters.fnWriteBlocks = prvWriteVirtIO;
        xParameters.fnReadBlocks = prvReadVirtIO;
        xParameters.pxDisk = pxDisk;
        xParameters.pvSemaphore = ( void * ) xSemaphoreCreateRecursiveMutex();
        xParameters.xBlockDeviceIsReentrant = pdFALSE;

        pxDisk->pxIOManager = FF_CreateIOManger( &xParameters, &xError );

        if( ( pxDisk->pxIOManager != NULL ) && ( FF_isERR( xError ) == pdFALSE ) )
        {
            /* Record that the VirtIO disk has been initialised. */
            pxDisk->xStatus.bIsInitialised = pdTRUE;

            /* Create a partition on the VirtIO disk.  NOTE!  The disk is only
            being partitioned here because it is a new VirtIO disk.  It is
            known that the disk has not been used before, and cannot already
            contain any partitions.  Most media drivers will not perform
            this step because the media will already been partitioned and
            formatted. */
#if 1
            /* FIXME: When using a pre-written disk image, comment this */
            xError = prvPartitionAndFormatDisk( pxDisk );
#endif

            if( FF_isERR( xError ) == pdFALSE )
            {
                /* Record the partition number the FF_Disk_t structure is, then
                mount the partition. */
                pxDisk->xStatus.bPartitionNumber = virtio_blkPARTITION_NUMBER;

                /* Mount the partition. */
                xError = FF_Mount( pxDisk, virtio_blkPARTITION_NUMBER );
            }

            if( FF_isERR( xError ) == pdFALSE )
            {
                /* The partition mounted successfully, add it to the virtual
                file system – where it will appear as a directory off the file
                system’s root directory. */
                if ( FF_FS_Add( pcName, pxDisk ) == 0 )
                {
                    xError = pdTRUE;
                }
            }
        }
        else
        {
            /* The disk structure was allocated, but the disk’s IO manager could
            not be allocated, so free the disk again. */
            FF_VirtIODiskDelete( pxDisk );
            pxDisk = NULL;
        }
    }

    return pxDisk;
}
/*-----------------------------------------------------------*/
