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
#include "rtapi_math64.h"
#include "hal.h"
#include "hostmot2.h"
#include "regex.h"


typedef struct {
    char name[HAL_NAME_LEN];
    hal_type_t type;
    hal_pin_dir_t dir;
    void** addr;
} pin_creation_data_t;


typedef struct {
    char name[HAL_NAME_LEN];
    bool read;
    rtapi_u32 reg_index;
    rtapi_u32* addr;
    rtapi_u32** reg;
} tram_creation_data_t;


/*
int convert_to_hdlc(const char* data,
                    int data_bytes_len,
                    const char sync,
                    int sync_
                    char* hdlc,
                    int hdlc_max_bytes,
                    char sync,
  */                  
                    


//searches the packed binary data hdlc for a valid hdlc encoded message
//copies the decoded payload into decoded
//on error returns negative index of bit where error occured
//on success returns the number of bits within the hdlc packet copied into data
int parse_hdlc(const char* hdlc, 
               int hdlc_bytes_len, 
               char* decoded, 
               int decoded_max_bytes) 
{
    //hdlc data is (hdlcflag)(bit-stuffed-payload)(hdlcflag)
    //hdlc flag looks like (01111110)
    //bit stuffed data has a 0 appended after 5 consecutive 1 bits. we ignore those zeroes
   

    int hdlc_max_bits = hdlc_bytes_len * 8;
    int decoded_max_bits = decoded_max_bytes*8;

    int hdlc_index = 0;
    int decoded_count = 0;

    int next_hdlc_bit = 0;
    int fifo = 0;
    int fifo_count = 0;
    int consecutive_1s = 0;
    int fifo_bit = 0;


    //find hdlc start flag 01111110
    while(hdlc_index < hdlc_max_bits) {
        next_hdlc_bit = (hdlc[hdlc_index/8] >> (7-(hdlc_index%8))) & 0x01;
        hdlc_index++;
        fifo = (fifo << 1) + next_hdlc_bit;
        
        if((fifo & 0xFF) == 0x7e) {
            fifo = 0;
            break;    
        } 
    }

    //no hdlc message was found
    if(hdlc_index == hdlc_max_bits) {
        return -hdlc_max_bits;
    }

    
    while((hdlc_index < hdlc_max_bits) && (decoded_count < decoded_max_bits)) {
        next_hdlc_bit = (hdlc[hdlc_index/8] >> (7-(hdlc_index%8))) & 0x01;
        hdlc_index++;
        fifo = (fifo << 1) + next_hdlc_bit;
        fifo_count++;

        if(fifo_count == 9) {
            //push the left most fifo bit into decoded stream
            //unless it is a 0 that followed 5 consecutive 1 bits
            fifo_bit = (fifo >> 8) & 0x1;

            if(fifo_bit == 1) {
                consecutive_1s++;
            }

            if((fifo_bit == 1) || ((fifo_bit == 0) && (consecutive_1s != 5)))
            {
                decoded[decoded_count/8] |= fifo_bit << (7-(decoded_count%8));
                decoded_count++;
            }

            if(fifo_bit == 0) {
                consecutive_1s = 0;
            }

            fifo_count--;
        }

        if((fifo & 0xFF) == 0x7e) {
            //have we crossed the hdlc end tag
            return decoded_count;
        }
    }

    return -hdlc_max_bits;
}



int crc(const char* data, int data_length) {
    //CRC-16/GENIBUS Check of 123456789 is 0xD64E

    unsigned short generator = 0x1021;
    unsigned short crc = 0xFFFF;
    unsigned short next = 0;

    for(int index = 0; index < data_length; index++) {
        next = data[index];
        next = next << 8;

        crc ^= next;

        for(int i = 0; i < 8; i++) {
            if((crc & 0x8000) != 0)
            {
                crc = (crc << 1) ^ generator;
            } else {
                crc = crc << 1;
            }
        }
    }

    return (crc ^ 0xFFFF);
}



void printdata(const char* data, int size) {
    char str[200];
    int pos = 0;

    for(int index = 0; index < size; index++) {
        pos += sprintf(str+pos, "%02X", (unsigned char)data[index]);
    }

    sprintf(str+pos, "\n");
    
    rtapi_print(str);
}



//extracts arbitrary bits from an array of bytes and assignes them to value
//returns -1 on failure, 0 on success
unsigned int extract_bits(const char* data, unsigned int bit_offset, int bit_length)
{
    unsigned int reg = 0;
    unsigned int bit = 0;

    if(bit_length > 0) {
        for(int index = bit_offset; index < bit_offset+bit_length; index++) {
            reg <<= 1;
            bit = (data[index/8] >> (7-(index%8))) & 0x01;
            reg += bit;
        }
    } else {
        for(int index = (bit_offset-bit_length-1); index >= bit_offset; index--) {
            reg <<= 1;
            bit = (data[index/8] >> (7-(index%8))) & 0x01;
            reg += bit;
        }
    }
    

    return reg;
}

    



int definePins(hostmot2_t *hm2, hm2_sigma5abs_instance_t* inst, int id) {
    int ret;
    char name[HAL_NAME_LEN];
    pin_creation_data_t pdata[] = {
        { "bitrate",  HAL_U32, HAL_IN, (void**)&(inst->bitrate)},
        { "transmit", HAL_BIT, HAL_IN, (void**)&(inst->transmit)},
        { "enable", HAL_BIT, HAL_IN, (void**)&(inst->enable)},
        { "tx-count", HAL_U32, HAL_IN, (void**)&(inst->tx_count)},
        { "tx0",      HAL_U32, HAL_IN, (void**)&(inst->tx0)},
        { "tx1",      HAL_U32, HAL_IN, (void**)&(inst->tx1)},
        { "rx-count", HAL_U32, HAL_OUT, (void**)&(inst->rx_count)},
        { "rx0",      HAL_U32, HAL_OUT, (void**)&(inst->rx0)},
        { "rx1",      HAL_U32, HAL_OUT, (void**)&(inst->rx1)},
        { "rx2",      HAL_U32, HAL_OUT, (void**)&(inst->rx2)},
        { "rx3",      HAL_U32, HAL_OUT, (void**)&(inst->rx3)},
        { "rx4",      HAL_U32, HAL_OUT, (void**)&(inst->rx4)},
        { "status",   HAL_U32, HAL_OUT, (void**)&(inst->status)},
        { "reset",        HAL_BIT, HAL_IN, (void**)&(inst->reset)},
        { "error",        HAL_BIT, HAL_OUT, (void**)&(inst->error)},
        { "debug",        HAL_U32, HAL_OUT, (void**)&(inst->debug)},
        { "crc",          HAL_U32, HAL_OUT, (void**)&(inst->crc)},
        { "rawcounts",    HAL_S32, HAL_OUT, (void**)&(inst->rawcounts)},
        { "index-enable", HAL_BIT, HAL_IO,  (void**)&(inst->index_enable)},
        { "position",     HAL_FLOAT, HAL_OUT, (void**)&(inst->position)},
        { "scale",        HAL_FLOAT, HAL_IN, (void**)&(inst->scale)},
        { "velocity",     HAL_FLOAT, HAL_OUT, (void**)&(inst->velocity)},
        { "referenced",   HAL_BIT, HAL_OUT, (void**)&(inst->referenced)},
        { "reference",    HAL_S32, HAL_OUT, (void**)&(inst->reference)},
        { "reference-offset", HAL_U32, HAL_OUT, (void**)&(inst->reference_offset)},
        { "slowclock",    HAL_U32, HAL_OUT, (void**)&(inst->slowclock)},
        { "fastclock",    HAL_U32, HAL_OUT, (void**)&(inst->fastclock)},
        { "input-z",      HAL_BIT, HAL_OUT, (void**)&(inst->input_z)},
        { "hall-a",       HAL_BIT, HAL_OUT, (void**)&(inst->hall_a)},
        { "hall-b",       HAL_BIT, HAL_OUT, (void**)&(inst->hall_b)},
        { "hall-c",       HAL_BIT, HAL_OUT, (void**)&(inst->hall_c)},
        { "turns",        HAL_U32, HAL_OUT, (void**)&(inst->turns)},
        { "magic1",        HAL_U32, HAL_OUT, (void**)&(inst->magic1)},
        { "magic2",        HAL_U32, HAL_OUT, (void**)&(inst->magic2)},
        { "magic3",        HAL_U32, HAL_OUT, (void**)&(inst->magic3)},
        };

        for(int index = 0; index < sizeof(pdata)/sizeof(pin_creation_data_t); index++) {
            if(snprintf(name, HAL_NAME_LEN, "%s.sigma5abs.%02d.%s", hm2->llio->name, id, pdata[index].name) == HAL_NAME_LEN) {
                HM2_ERR("sigma5abs: Formatted pin name too long. %s\n", pdata[index].name);
                return -1;
            }

            ret = hal_pin_new(name, pdata[index].type, pdata[index].dir, pdata[index].addr, hm2->llio->comp_id);
         	if (ret < 0) {
                HM2_ERR("error adding pin %s. error code %d\n", name, ret);
                return -1;
            }
        }
    
        return 0;
    }
	

        


int hm2_sigma5abs_configure_tram(hostmot2_t* hm2, hm2_module_descriptor_t *md) {
    int r = 0;
    hm2_sigma5abs_t* sg5 = &hm2->sigma5abs;

    tram_creation_data_t td[] = {
        { "bitrate",  false, 1,  &sg5->bitrate_addr,  &sg5->bitrate_reg },
        { "timer",    false, 2,  &sg5->timer_addr,    &sg5->timer_reg },
        { "tx_count", false, 3,  &sg5->tx_count_addr, &sg5->tx_count_reg },
        { "tx0",      false, 4,  &sg5->tx0_addr,      &sg5->tx0_reg },
        { "tx1",      false, 5,  &sg5->tx1_addr,       &sg5->tx1_reg },

        //control register must be written last
        { "control",  false, 0,  &sg5->control_addr,  &sg5->control_reg },

        { "rx_count", true,  6,  &sg5->rx_count_addr, &sg5->rx_count_reg },
        { "rx0",      true,  7,  &sg5->rx0_addr,      &sg5->rx0_reg },
        { "rx1",      true,  8,  &sg5->rx1_addr,      &sg5->rx1_reg },
        { "rx2",      true,  9,  &sg5->rx2_addr,      &sg5->rx2_reg },
        { "rx3",      true,  10, &sg5->rx3_addr,      &sg5->rx3_reg },
        { "rx4",      true,  11, &sg5->rx4_addr,      &sg5->rx4_reg },
        { "status",   true,  12, &sg5->status_addr,   &sg5->status_reg },
        { "debug",    true,  13, &sg5->debug_addr,    &sg5->debug_reg }
    };


    for(int index = 0; index < sizeof(td)/sizeof(tram_creation_data_t); index++) {
        *td[index].addr = md->base_address + (td[index].reg_index * md->register_stride);
        
        *td[index].reg = (rtapi_u32*)rtapi_kmalloc(sg5->num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);

        if(NULL == *td[index].reg) {
            HM2_ERR("sigma5abs: rtapi_kmalloc failed for %s.", td[index].name);
            return -1;
        }

        if(td[index].read) {
            r = hm2_register_tram_read_region(hm2, *td[index].addr, sg5->num_instances * sizeof(rtapi_u32), td[index].reg);

            if (r < 0) {
                HM2_ERR("sigma5abs: error registering tram read region for register %s (Code: %d)\n", td[index].name, r);
                return -1;
            }
        } else {
            r = hm2_register_tram_write_region(hm2, *td[index].addr, sg5->num_instances * sizeof(rtapi_u32), td[index].reg);
            if (r < 0) {
                HM2_ERR("sigma5abs: error registering tram write region for register %s (Code: (%d)\n", td[index].name, r);
                return -1;
            }
        }
    }

    return 0;
}


int hm2_sigma5abs_parse_md(hostmot2_t *hm2, int md_index) 
{
    int i, r = -EINVAL;
    hm2_module_descriptor_t *md = &hm2->md[md_index];
    static int last_gtag = -1;

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
        return -ENOMEM;
    }

    hm2->sigma5abs.instance_stride = md->instance_stride;

    r = hm2_sigma5abs_configure_tram(hm2, md);
    if(r < 0) {
        return r;
    }

    for (i = 0 ; i < hm2->sigma5abs.num_instances ; i++){
        hm2_sigma5abs_instance_t *inst = &hm2->sigma5abs.instances[i];

        if(definePins(hm2, inst, i) < 0) {
            return -1;
        }
    
        inst->prev_abs_counter = 0;
        inst->full_raw_counts = 0;
        inst->index_offset = 0;
        *(inst->rawcounts) = 0;
        *(inst->referenced) = 0;
        *(inst->reference) = 0;
        *(inst->reference_offset) = 0;
        *(inst->index_enable) = 0;
        *(inst->position) = 0;
        *(inst->scale) = 1.0;
        *(inst->velocity) = 0;
        *(inst->slowclock) = 0;
        *(inst->fastclock) = 0;
        *(inst->input_z) = 0;
        *(inst->hall_a) = 0;
        *(inst->hall_b) = 0;
        *(inst->hall_c) = 0;
        *(inst->turns) = 0;
        *(inst->crc) = 0;
        *(inst->reset) = false;
        *(inst->error) = false;
        *(inst->bitrate) = 13;
        *(inst->enable) = 1;
        *(inst->transmit) = 1;
        *(inst->tx0) = 0x55555553;
        *(inst->tx1) = 0xF7DF7D7E;
        *(inst->tx_count) = 64;
        *(inst->crc) = 0;
        *(inst->status) = 0;
        *(inst->debug) = 0;
    }
	
    
    return hm2->sigma5abs.num_instances;
}

void hm2_sigma5abs_cleanup(hostmot2_t* hm2) {
}

void hm2_sigma5abs_write(hostmot2_t *hm2) {
}

void hm2_sigma5abs_force_write(hostmot2_t *hm2) {
}


void hm2_sigma5abs_prepare_tram_write(hostmot2_t* hm2) {
    hm2_sigma5abs_instance_t* inst;

    for(int i = 0; i < hm2->sigma5abs.num_instances; i++) {
        inst = &hm2->sigma5abs.instances[i];

        //important: transmit must be written after tx values
        hm2->sigma5abs.bitrate_reg[i] = *inst->bitrate;
        hm2->sigma5abs.tx_count_reg[i] = *inst->tx_count;
        hm2->sigma5abs.tx0_reg[i] = *inst->tx0;
        hm2->sigma5abs.tx1_reg[i] = *inst->tx1;

        if(*inst->enable) {
            hm2->sigma5abs.control_reg[i] |= 0x2;
        } else {
            hm2->sigma5abs.control_reg[i] &= ~0x2;
        }

        if(*inst->transmit) {
            hm2->sigma5abs.control_reg[i] ^= 0x1;
        }
    }
}


void hm2_sigma5abs_process_tram_read(hostmot2_t* hm2) {
	hm2_sigma5abs_instance_t* inst;

    char reg[20];  //raw data passed from encoder with an hdlc message encoded in it
    char data[14]; //actual encoder data parsed from reg
    
    int hdlc_ret = 0; 

    hal_bit_t prev_referenced = 0;

    int ppr = 2097152; //encoder pulses per turn
    int counter_width = 24;
    rtapi_s64 counter_center = 1U << (counter_width-1);
    rtapi_s64 wrap_counts = 1U << counter_width;

    rtapi_s64 prev_full_raw_counts = 0;
    rtapi_s64 abs_counter;
    rtapi_s64 counter_rollover = 0;
    rtapi_s64 direction = 0;
    
    for(int i = 0; i < hm2->sigma5abs.num_instances; i++) {
        inst = &hm2->sigma5abs.instances[i];

        prev_referenced = *(inst->referenced);
        prev_full_raw_counts = inst->full_raw_counts;

        //update status register
        *(inst->debug) = (hal_u32_t)hm2->sigma5abs.debug_reg[i];
        *(inst->status) = (hal_u32_t)hm2->sigma5abs.status_reg[i];
        *(inst->rx_count) = (hal_u32_t)hm2->sigma5abs.rx_count_reg[i];
        *(inst->rx0) = (hal_u32_t)hm2->sigma5abs.rx0_reg[i];
        *(inst->rx1) = (hal_u32_t)hm2->sigma5abs.rx1_reg[i];
        *(inst->rx2) = (hal_u32_t)hm2->sigma5abs.rx2_reg[i];
        *(inst->rx3) = (hal_u32_t)hm2->sigma5abs.rx3_reg[i];
        *(inst->rx4) = (hal_u32_t)hm2->sigma5abs.rx4_reg[i];


        *(hal_u32_t*)(reg+(0*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx0_reg[i]);
        *(hal_u32_t*)(reg+(1*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx1_reg[i]);
        *(hal_u32_t*)(reg+(2*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx2_reg[i]);
        *(hal_u32_t*)(reg+(3*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx3_reg[i]);
        *(hal_u32_t*)(reg+(4*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx4_reg[i]);


        memset(data, 0, sizeof(data));
        hdlc_ret = parse_hdlc(reg, 20, data, 14);
    
        if(hdlc_ret == 112) {
            //check crc
            *(inst->crc) = extract_bits(data, 96, 16);

            if(crc(data, 12) == *(inst->crc)) {

                *(inst->magic1) = extract_bits(data, 0, 6); //appears to be the function code sent to the encoder
                *(inst->referenced) = !extract_bits(data, 6, 1); //becomes 0 once an index pulse is seen
                *(inst->magic2) = extract_bits(data, 7, 10); //unknown
                *(inst->slowclock)= extract_bits(data, 16, -8); //counts up every 1/5 of a second
                *(inst->fastclock) = extract_bits(data, 24, 26); //appears to be random data that changes every request
                *(inst->magic3) = extract_bits(data, 50, 6); //unknown
                abs_counter =  extract_bits(data, 56, -counter_width); //absolute turn counter 
                *(inst->input_z) =  extract_bits(data, 80, 1);
                *(inst->hall_a) =  extract_bits(data, 81, 1);
                *(inst->hall_b) = extract_bits(data, 82, 1);
                *(inst->hall_c) = extract_bits(data, 83, 1);
                *(inst->reference_offset) = extract_bits(data, 84, 9);
                *(inst->turns) = extract_bits(data, 93, -3);  //8 bit full turn counter
                

                //encoder count to turn value chart
                //2097152      7 0 1        
                //4194304        0 1 2
                //6291456          1 2 3
                //8388608            2 3 4
                //10485760             3 4 5
                //12582912               4 5 6
                //14680064                 5 6 7
                //16777216                   6 7 0

                if( (*(inst->turns) == 0 || *(inst->turns) == 7) && inst->prev_abs_counter > counter_center && abs_counter < counter_center) {
                    //positive rollover
                    counter_rollover = wrap_counts; 
                } else if( (*(inst->turns) == 0 || *(inst->turns)==7) && inst->prev_abs_counter < counter_center && abs_counter > counter_center) {
                    //negative rollover
                    counter_rollover = -wrap_counts;
                }

                inst->full_raw_counts += abs_counter - inst->prev_abs_counter + counter_rollover;
                inst->prev_abs_counter = abs_counter;
                *(inst->rawcounts) = inst->full_raw_counts;

                direction = prev_full_raw_counts < inst->full_raw_counts ? 1 : -1;

                if(!prev_referenced && *(inst->referenced)) {
                    //when the encoder first powers on it does not know the relative location of the index point.
                    //the first time that it passes the index point a flag and an offset value is recorded in the encoders memory.
                    //The offset is the number of counts passed before the encoder packet was transmitted
                    //depending upon the direction of travel we must add or subtract that offset to determine
                    //the absolute reference position.
                    *(inst->reference) = (abs_counter - (direction * *(inst->reference_offset))) % ppr;
                }
                        
               
            
                if(*(inst->index_enable) && *(inst->referenced)) {
                    //simulate an incremental encoder's behavior
                    //set the 0 point to the index that the encoder is moving towards
                    //it isn't necessary to wait for the index pulse to occur because the index position
                    //is recorded when the encoder is referenced
                    inst->index_offset = inst->full_raw_counts + (direction >= 0 ? ppr : 0) - *(inst->reference);
                    *(inst->index_enable) = 0;
                }
                
                if(*(inst->scale) == 0.0) {
                    HM2_ERR("scale of 0.0 is invalid. setting to 1.0\n");
                    *(inst->scale) = 1.0;
                }

                *(inst->position) = (hal_float_t)(inst->full_raw_counts - inst->index_offset) / *(inst->scale);


            } else {
                *(inst->error) = 1;
            }
        } else {
            *(inst->error) = 1;
        }
    }
}


void hm2_sigma5abs_print_module(hostmot2_t* hm2) {
    if (hm2->sigma5abs.num_instances <= 0) return;
    HM2_PRINT("Sigma5ABS: %d\n", hm2->sigma5abs.num_instances);
    HM2_PRINT("    Control Addr: 0x%X\n", hm2->sigma5abs.control_addr);
    HM2_PRINT("    Bitrate Addr: 0x%X\n", hm2->sigma5abs.bitrate_addr);
    HM2_PRINT("    Timer Addr: 0x%X\n", hm2->sigma5abs.timer_addr);
    HM2_PRINT("    Tx Count Addr: 0x%X\n", hm2->sigma5abs.tx_count_addr);
    HM2_PRINT("    Tx0 Addr: 0x%X\n", hm2->sigma5abs.tx0_addr);
    HM2_PRINT("    Tx1 Addr: 0x%X\n", hm2->sigma5abs.tx1_addr);
    HM2_PRINT("    Rx Count Addr: 0x%X\n", hm2->sigma5abs.rx_count_addr);
    HM2_PRINT("    Rx0 Addr: 0x%X\n", hm2->sigma5abs.rx0_addr);
    HM2_PRINT("    Rx1 Addr: 0x%X\n", hm2->sigma5abs.rx1_addr);
    HM2_PRINT("    Rx2 Addr: 0x%X\n", hm2->sigma5abs.rx2_addr);
    HM2_PRINT("    Rx3 Addr: 0x%X\n", hm2->sigma5abs.rx3_addr);
    HM2_PRINT("    Rx4 Addr: 0x%X\n", hm2->sigma5abs.rx4_addr);
    HM2_PRINT("    Status Addr: 0x%X\n", hm2->sigma5abs.status_addr);
    HM2_PRINT("    Debug Addr: 0x%X\n", hm2->sigma5abs.debug_addr);
}


