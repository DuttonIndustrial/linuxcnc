//
//    Copyright (C) 2020 Curtis Dutton
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERinstTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

#include <rtapi_slab.h>
#include "rtapi.h"
#include "rtapi_string.h"
#include "rtapi_math.h"
#include "hal.h"
#include "hostmot2.h"
#include "regex.h"
#include "sigma5abs.h"



int hm2_sigma5abs_parse_md(hostmot2_t *hm2, int md_index) 
{
    int i, r = -EINVAL;
    hm2_module_descriptor_t *md = &hm2->md[md_index];
    static int last_gtag = -1;
    const char* fmtError = "%s.sigma5abs.%02d.error";
    const char* fmtTransmit = "%s.sigma5abs.%02d.transmit";
    const char* fmtbitrate = "%s.sigma5abs.%02d.bitrate";
    const char* fmttxcount = "%s.sigma5abs.%02d.tx_count";
    const char* fmttx0 = "%s.sigma5abs.%02d.tx0";
    const char* fmttx1 = "%s.sigma5abs.%02d.tx1";
    const char* fmttx2 = "%s.sigma5abs.%02d.tx2";
    const char* fmttx3 = "%s.sigma5abs.%02d.tx3";
    const char* fmtrxcount = "%s.sigma5abs.%02d.rx_count";
    const char* fmtrx0 = "%s.sigma5abs.%02d.rx0";
    const char* fmtrx1 = "%s.sigma5abs.%02d.rx1";
    const char* fmtrx2 = "%s.sigma5abs.%02d.rx2";
    const char* fmtrx3 = "%s.sigma5abs.%02d.rx3";
    const char* fmtrx4 = "%s.sigma5abs.%02d.rx4";
    const char* fmtDebug = "%s.sigma5abs.%02d.debug";

    if (!hm2_md_is_consistent_or_complain(hm2, md_index, 0, 14, 4, 0x3FFF)) {
        HM2_ERR("inconsistent Module Descriptor!\n");
        return -EINVAL;
    }
    
    if (hm2->sigma5abs.num_instances > 1 && last_gtag == md->gtag) {
        HM2_ERR(
                "found duplicate Module Descriptor for %s (inconsistent "
                "firmware), not loading driver %i %i\n",
                hm2_get_general_function_name(md->gtag), md->gtag, last_gtag
                );
        return -EINVAL;
    }
    last_gtag = md->gtag;

    if (hm2->config.num_sigma5abs > md->instances) {
        HM2_ERR(
                "config defines %d sigma5abs, but only %d are available, "
                "not loading driver\n",
                hm2->config.num_sigma5abs,
                md->instances
                );
        return -EINVAL;
    }
    
    if (hm2->config.num_sigma5abs == 0) {
        return 0;
    }


    // 
    // looks good, start, or continue, initializing
    // 
    if (hm2->config.num_sigma5abs == -1) {
        hm2->sigma5abs.num_instances = md->instances;
    } else {
        hm2->sigma5abs.num_instances = hm2->config.num_sigma5abs;
    }
        
    hm2->sigma5abs.instances = (hm2_sigma5abs_instance_t *)hal_malloc(hm2->sigma5abs.num_instances 
                                                               * sizeof(hm2_sigma5abs_instance_t));
    if (NULL == hm2->sigma5abs.instances) {
        HM2_ERR("sigma5abs: out of hal memory!\n");
        r = -ENOMEM;
        goto fail0;
    }

    


    hm2->sigma5abs.instance_stride = md->instance_stride;
    hm2->sigma5abs.transmit_addr = md->base_address + (0 * md->register_stride);
    hm2->sigma5abs.bitrate_addr = md->base_address + (1 * md->register_stride);
    hm2->sigma5abs.tx_count_addr = md->base_address + (2 * md->register_stride);
    hm2->sigma5abs.tx0_addr = md->base_address + (3 * md->register_stride);
    hm2->sigma5abs.tx1_addr = md->base_address + (4 * md->register_stride);
    hm2->sigma5abs.tx2_addr = md->base_address + (5 * md->register_stride);
    hm2->sigma5abs.tx3_addr = md->base_address + (6 * md->register_stride);
    hm2->sigma5abs.rx_count_addr = md->base_address + (7 * md->register_stride);
    hm2->sigma5abs.rx0_addr = md->base_address + (8 * md->register_stride);
    hm2->sigma5abs.rx1_addr = md->base_address + (9 * md->register_stride);
    hm2->sigma5abs.rx2_addr = md->base_address + (10 * md->register_stride);
    hm2->sigma5abs.rx3_addr = md->base_address + (11 * md->register_stride);
    hm2->sigma5abs.rx4_addr = md->base_address + (12 * md->register_stride);
    hm2->sigma5abs.debug_addr = md->base_address + (13 * md->register_stride);

    hm2->sigma5abs.transmit_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
  
    if(NULL == hm2->sigma5abs.transmit_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for transmit_reg.");
        r = -ENOMEM;
        goto fail0;
    }

    hm2->sigma5abs.bitrate_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);

    if(NULL == hm2->sigma5abs.bitrate_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for bitrae_reg.");
        r = -ENOMEM;
        goto fail0;
    }

    hm2->sigma5abs.tx_count_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
  
    if(NULL == hm2->sigma5abs.tx_count_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for tx_count_reg.");
        r = -ENOMEM;
        goto fail0;
    }

    hm2->sigma5abs.tx0_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
  
    if(NULL == hm2->sigma5abs.tx0_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for tx0_reg.");
        r = -ENOMEM;
        goto fail0;
    }

    hm2->sigma5abs.tx1_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
  
    if(NULL == hm2->sigma5abs.tx1_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for tx1_reg.");
        r = -ENOMEM;
        goto fail0;
    }

    hm2->sigma5abs.tx2_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
  
    if(NULL == hm2->sigma5abs.tx2_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for tx2_reg.");
        r = -ENOMEM;
        goto fail0;
    }
 
    hm2->sigma5abs.tx3_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
  
    if(NULL == hm2->sigma5abs.tx3_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for tx3_reg.");
        r = -ENOMEM;
        goto fail0;
    }

    hm2->sigma5abs.rx_count_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
  
    if(NULL == hm2->sigma5abs.rx_count_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for rx_count_reg.");
        r = -ENOMEM;
        goto fail0;
    }

    hm2->sigma5abs.rx0_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
  
    if(NULL == hm2->sigma5abs.rx0_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for tx0_reg.");
        r = -ENOMEM;
        goto fail0;
    }

    hm2->sigma5abs.rx1_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
  
    if(NULL == hm2->sigma5abs.rx1_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for rx1_reg.");
        r = -ENOMEM;
        goto fail0;
    }

    hm2->sigma5abs.rx2_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
  
    if(NULL == hm2->sigma5abs.rx2_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for rx2_reg.");
        r = -ENOMEM;
        goto fail0;
    }
 
    hm2->sigma5abs.rx3_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
  
    if(NULL == hm2->sigma5abs.rx3_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for rx3_reg.");
        r = -ENOMEM;
        goto fail0;
    }

    hm2->sigma5abs.rx4_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);
  
    if(NULL == hm2->sigma5abs.rx4_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for rx4_reg.");
        r = -ENOMEM;
        goto fail0;
    }


    hm2->sigma5abs.debug_reg = (rtapi_u32*)rtapi_kmalloc(hm2->sigma5abs.num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);

    if(NULL == hm2->sigma5abs.debug_reg) {
        HM2_ERR("sigma5abs: rtapi_kmalloc failed for debug_reg.");
        r = -ENOMEM;
        goto fail0;
    }

    r = hm2_register_tram_write_region(hm2, hm2->sigma5abs.tx_count_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.tx_count_reg);

    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram write region for tx_count register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_write_region(hm2, hm2->sigma5abs.tx0_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.tx0_reg);

    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram write region for tx0 register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_write_region(hm2, hm2->sigma5abs.tx1_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.tx1_reg);

    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram write region for tx1 register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_write_region(hm2, hm2->sigma5abs.tx2_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.tx2_reg);

    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram write region for tx2 register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_write_region(hm2, hm2->sigma5abs.tx3_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.tx3_reg);

    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram write region for tx3 register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_write_region(hm2, hm2->sigma5abs.bitrate_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.bitrate_reg);
    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram write region for transmit register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_write_region(hm2, hm2->sigma5abs.transmit_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.transmit_reg);
    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram write region for transmit register (%d)\n", r);
        goto fail0;
    }



    r = hm2_register_tram_read_region(hm2, hm2->sigma5abs.rx_count_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.rx_count_reg);

    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram read fregion for rx_count register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_read_region(hm2, hm2->sigma5abs.rx0_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.rx0_reg);

    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram read fregion for rx0 register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_read_region(hm2, hm2->sigma5abs.rx1_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.rx1_reg);

    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram read fregion for rx1 register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_read_region(hm2, hm2->sigma5abs.rx2_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.rx2_reg);

    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram read fregion for rx2 register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_read_region(hm2, hm2->sigma5abs.rx3_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.rx3_reg);

    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram read fregion for rx3 register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_read_region(hm2, hm2->sigma5abs.rx4_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.rx4_reg);

    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram read fregion for rx4 register (%d)\n", r);
        goto fail0;
    }

    r = hm2_register_tram_read_region(hm2, hm2->sigma5abs.debug_addr, hm2->sigma5abs.num_instances * sizeof(rtapi_u32), &hm2->sigma5abs.debug_reg);

    if (r < 0) {
        HM2_ERR("sigma5abs: error registering tram read fregion for debug register (%d)\n", r);
        goto fail0;
    }


    for (i = 0 ; i < hm2->sigma5abs.num_instances ; i++){
        hm2_sigma5abs_instance_t *inst = &hm2->sigma5abs.instances[i];

	    r = hal_pin_bit_newf(HAL_IN, &(inst->transmit), hm2->llio->comp_id, fmtTransmit, hm2->llio->name, i);
    	if (r < 0) {
                HM2_ERR("error adding transmit pin for %s(%d)\n", hm2->llio->name, i);
                r = -ENOMEM;
                goto fail0;
            }

        r = hal_pin_u32_newf(HAL_IN, &(inst->bitrate), hm2->llio->comp_id, fmtbitrate, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding bitrate pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }

        r = hal_pin_u32_newf(HAL_IN, &(inst->tx_count), hm2->llio->comp_id, fmttxcount, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding txcount pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }

        r = hal_pin_u32_newf(HAL_IN, &(inst->tx0), hm2->llio->comp_id, fmttx0, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding tx0 pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }

        r = hal_pin_u32_newf(HAL_IN, &(inst->tx1), hm2->llio->comp_id, fmttx1, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding tx1 pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }

        r = hal_pin_u32_newf(HAL_IN, &(inst->tx2), hm2->llio->comp_id, fmttx2, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding tx2 pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }

        r = hal_pin_u32_newf(HAL_IN, &(inst->tx3), hm2->llio->comp_id, fmttx3, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding tx3 pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }
    

        r = hal_pin_u32_newf(HAL_OUT, &(inst->rx_count), hm2->llio->comp_id, fmtrxcount, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding rxcount pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }
    
        r = hal_pin_u32_newf(HAL_OUT, &(inst->rx0), hm2->llio->comp_id, fmtrx0, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding rx0 pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }

        r = hal_pin_u32_newf(HAL_OUT, &(inst->rx1), hm2->llio->comp_id, fmtrx1, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding rx1 pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }

        r = hal_pin_u32_newf(HAL_OUT, &(inst->rx2), hm2->llio->comp_id, fmtrx2, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding rx2 pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }

        r = hal_pin_u32_newf(HAL_OUT, &(inst->rx3), hm2->llio->comp_id, fmtrx3, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding rx3 pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }

        r = hal_pin_u32_newf(HAL_OUT, &(inst->rx4), hm2->llio->comp_id, fmtrx4, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding rx4 pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }


        r = hal_pin_u32_newf(HAL_OUT, &(inst->decode0), hm2->llio->comp_id, "%s.sigma5abs.%02d.decode0", hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding decode0 pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }
        r = hal_pin_u32_newf(HAL_OUT, &(inst->decode1), hm2->llio->comp_id, "%s.sigma5abs.%02d.decode1", hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding decode1 pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }
        r = hal_pin_u32_newf(HAL_OUT, &(inst->decode2), hm2->llio->comp_id, "%s.sigma5abs.%02d.decode2", hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding decode2 pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }
        r = hal_pin_u32_newf(HAL_OUT, &(inst->decode3), hm2->llio->comp_id, "%s.sigma5abs.%02d.decode3", hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding decode3 pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }


        r = hal_pin_bit_newf(HAL_OUT, &(inst->error), hm2->llio->comp_id, fmtError, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding error pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }


        r = hal_pin_u32_newf(HAL_OUT, &(inst->debug), hm2->llio->comp_id, fmtDebug, hm2->llio->name, i);
        if (r < 0) {
            HM2_ERR("error adding debug pin for %s(%d)\n", hm2->llio->name, i);
            r = -ENOMEM;
            goto fail0;
        }


        *(inst->error) = false;
    	*(inst->transmit) = 0;
        *(inst->tx_count) = 0;
        *(inst->tx0) = 0;
        *(inst->tx1) = 0;
        *(inst->tx2) = 0;
        *(inst->tx3) = 0;
        *(inst->bitrate) = 0;
    }
	
    
    return hm2->sigma5abs.num_instances;

fail0:
    hm2->sigma5abs.num_instances = 0;
/*
    if(hm2->sigma5abs.transmit_reg) {
        rtapi_kfree(hm2->sigma5abs.transmit_reg);
        hm2->sigma5abs.transmit_reg = 0;
    }

    if(hm2->sigma5abs.data_reg) {
        rtapi_kfree(hm2->sigma5abs.data_reg);
        hm2->sigma5abs.data_reg = 0;
    }
*/
    return r;
}

void hm2_sigma5abs_cleanup(hostmot2_t* hm2) {
}

void hm2_sigma5abs_write(hostmot2_t *hm2) {
//    rtapi_u32 buff;
//    hm2_sigma5abs_instance_t* inst;


//    for(int i = 0; i < hm2->sigma5abs.num_instances; i++) {
//        inst = &hm2->sigma5abs.instances[i];
//
//        }
//    }
}

void hm2_sigma5abs_force_write(hostmot2_t *hm2) {
    rtapi_u32 buff;
    hm2_sigma5abs_instance_t* inst;
  
    for(int i = 0; i < hm2->sigma5abs.num_instances; i++) {
        inst = &hm2->sigma5abs.instances[i];
        
       
    	if (*inst->transmit) {
	    	buff = 0x1;
        } else {
            buff = 0x0;
        }

        hm2->llio->write(hm2->llio, hm2->sigma5abs.transmit_addr + (i * hm2->sigma5abs.instance_stride), &buff, sizeof(rtapi_u32));
    }
}


void hm2_sigma5abs_prepare_tram_write(hostmot2_t* hm2) {
    hm2_sigma5abs_instance_t* inst;

    for(int i = 0; i < hm2->sigma5abs.num_instances; i++) {
        inst = &hm2->sigma5abs.instances[i];

       //important: transmit must be written after tx values
       hm2->sigma5abs.transmit_reg[i] = *inst->transmit;
       hm2->sigma5abs.bitrate_reg[i] = *inst->bitrate;
       hm2->sigma5abs.tx_count_reg[i] = *inst->tx_count;
       hm2->sigma5abs.tx0_reg[i] = *inst->tx0;
       hm2->sigma5abs.tx1_reg[i] = *inst->tx1;
       hm2->sigma5abs.tx2_reg[i] = *inst->tx2;
       hm2->sigma5abs.tx3_reg[i] = *inst->tx3;
 
    }
}



//searches the packed binary data hdlc for a valid hdlc encoded message
//copies the decoded payload into decoded
//returns -1 if no valid hdlc data is found or if hdlc data won't fit into decoded
//returns the number of bits within the hdlc packet copied into data
int parse_hdlc(char* hdlc, 
               int hdlc_bytes_len, 
               char* decoded, 
               int decoded_max_bytes) 
{
    
    //hdlc data is (hdlcflag)(bit-stuffed-payload)(hdlcflag)
    //hdlc flag looks like (01111110)
    //bit stuffed data has a 0 appended after 5 consecutive 1 bits. we ignore those zeroes
    int current_bit = 0;
    int decoded_max_bits = decoded_max_bytes*8;
    int hdlc_started = 0;
    int decoded_count = 0;

    //we store bits in fifo as we loop through data
    //fifo is a 16 bit register we use to recognize flags and bit suffing
    int fifo = 0;
    int fifo_count = 0;
    int consecutive_1s = 0;
    int fifo_bit = 0;

    for(int index = 0; index < (hdlc_bytes_len*8) && decoded_count < decoded_max_bits; index++) {
        current_bit = (hdlc[index/8] >> (7-(index%8))) & 0x01;
        fifo = (fifo << 1) + current_bit;
        fifo_count++;

        if(hdlc_started == 1) {
            if(fifo_count == 9) {
                //push the left most fifo bit into decoded stream
                //unless it is a 0 that followed 5 consecutive 1 bits
                fifo_bit = (fifo >> 8) & 0x1;
                
                if(fifo_bit == 1) {
                    consecutive_1s++;
                }

                if((fifo_bit == 1) || (fifo_bit == 0 && consecutive_1s < 5))
                {
                    decoded[decoded_count/8] |= fifo_bit << (7-(decoded_count%8));
                    decoded_count++;
                }

                if(fifo_bit == 0) {
                    consecutive_1s = 0;
                }

                fifo_count = 8;
            }

            //truncate fifo the the rightmost 8 bits
            fifo = fifo & 0xFF;

            if(fifo == 0x7e) {
                //have we crossed the hdlc end tag
                return decoded_count;
            }

        } else {
            fifo = fifo & 0xFF;
            //fifo now holds the hdlc start tag
            if(fifo == 0x7e) {
                hdlc_started = 1;
                fifo = 0;
                fifo_count = 0;
            } 
        }
    }

    return -1;
}

void hm2_sigma5abs_process_tram_read(hostmot2_t* hm2) {
	hm2_sigma5abs_instance_t* inst;
//    sigma5abs_serial_data_t data;

    char reg[20];
    hal_u32_t tmp;
    char data[16];
    


    for(int i = 0; i < hm2->sigma5abs.num_instances; i++) {
        inst = &hm2->sigma5abs.instances[i];

        //update status register
        *(inst->debug) = (hal_u32_t)hm2->sigma5abs.debug_reg[i];
        *(inst->rx_count) = (hal_u32_t)hm2->sigma5abs.rx_count_reg[i];
        *(inst->rx0) = (hal_u32_t)hm2->sigma5abs.rx0_reg[i];
        *(inst->rx1) = (hal_u32_t)hm2->sigma5abs.rx1_reg[i];
        *(inst->rx2) = (hal_u32_t)hm2->sigma5abs.rx2_reg[i];
        *(inst->rx3) = (hal_u32_t)hm2->sigma5abs.rx3_reg[i];
        *(inst->rx4) = (hal_u32_t)hm2->sigma5abs.rx4_reg[i];


        //convert host order to big endian
        tmp = htobe32(hm2->sigma5abs.rx0_reg[i]);   
        memcpy(reg+(0*sizeof(hal_u32_t)), &tmp, sizeof(hal_u32_t));

        tmp = htobe32(hm2->sigma5abs.rx1_reg[i]);   
        memcpy(reg+(1*sizeof(hal_u32_t)), &tmp, sizeof(hal_u32_t));

        tmp = htobe32(hm2->sigma5abs.rx2_reg[i]);   
        memcpy(reg+(2*sizeof(hal_u32_t)), &tmp, sizeof(hal_u32_t));

        tmp = htobe32(hm2->sigma5abs.rx3_reg[i]);   
        memcpy(reg+(3*sizeof(hal_u32_t)), &tmp, sizeof(hal_u32_t));

        tmp = htobe32(hm2->sigma5abs.rx4_reg[i]);   
        memcpy(reg+(4*sizeof(hal_u32_t)), &tmp, sizeof(hal_u32_t));


        memset(data, 0, 16);
        if(parse_hdlc(reg, 20, data, sizeof(sigma5abs_serial_data_t)) < 0) {
            *(inst->error) = 1;
        } else {
            *(inst->error) = 0;
        }
    
        //convert bigendian to host order
        memcpy(&tmp, data+(0*sizeof(hal_u32_t)), sizeof(hal_u32_t)); 
        *(inst->decode0) = be32toh(tmp);

        memcpy(&tmp, data+(1*sizeof(hal_u32_t)), sizeof(hal_u32_t)); 
        *(inst->decode1) = be32toh(tmp);

        memcpy(&tmp, data+(2*sizeof(hal_u32_t)), sizeof(hal_u32_t)); 
        *(inst->decode2) = be32toh(tmp);

        memcpy(&tmp, data+(3*sizeof(hal_u32_t)), sizeof(hal_u32_t)); 
        *(inst->decode3) = be32toh(tmp);

        
        

        
        
        
        //copy data to hal

        
    }
}


void hm2_sigma5abs_print_module(hostmot2_t* hm2) {
    if (hm2->sigma5abs.num_instances <= 0) return;
    HM2_PRINT("Sigma5ABS: %d\n", hm2->sigma5abs.num_instances);
    HM2_PRINT("    Transmit Addr: 0x%X\n", hm2->sigma5abs.transmit_addr);
    HM2_PRINT("    Bitrate Addr: 0x%X\n", hm2->sigma5abs.bitrate_addr);
    HM2_PRINT("    Tx Count Addr: 0x%X\n", hm2->sigma5abs.tx_count_addr);
    HM2_PRINT("    Tx0 Addr: 0x%X\n", hm2->sigma5abs.tx0_addr);
    HM2_PRINT("    Tx1 Addr: 0x%X\n", hm2->sigma5abs.tx1_addr);
    HM2_PRINT("    Tx2 Addr: 0x%X\n", hm2->sigma5abs.tx2_addr);
    HM2_PRINT("    Tx3 Addr: 0x%X\n", hm2->sigma5abs.tx3_addr);
    HM2_PRINT("    Rx Count Addr: 0x%X\n", hm2->sigma5abs.rx_count_addr);
    HM2_PRINT("    Rx0 Addr: 0x%X\n", hm2->sigma5abs.rx0_addr);
    HM2_PRINT("    Rx1 Addr: 0x%X\n", hm2->sigma5abs.rx1_addr);
    HM2_PRINT("    Rx2 Addr: 0x%X\n", hm2->sigma5abs.rx2_addr);
    HM2_PRINT("    Rx3 Addr: 0x%X\n", hm2->sigma5abs.rx3_addr);
    HM2_PRINT("    Rx4 Addr: 0x%X\n", hm2->sigma5abs.rx4_addr);
    HM2_PRINT("    Debug Addr: 0x%X\n", hm2->sigma5abs.debug_addr);
}

