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
//if bit_length < 0 it reads bits in reverse
//does not check arguments
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

//length of data must be 14 bytes
void decode_encoder_packet(char* data, 
                          hal_u32_t* magic1, 
                          hal_bit_t* referenced, 
                          hal_u32_t* magic2, 
                          hal_u32_t* slowclock, 
                          hal_u32_t* fastclock,
                          hal_u32_t* magic3,
                          hal_u32_t* encoder_count,
                          hal_bit_t* z,
                          hal_bit_t* u,
                          hal_bit_t* v,
                          hal_bit_t* w,
                          hal_u32_t* ref_offset,
                          hal_u32_t* z_counter,
                          hal_u32_t* crc)
{
    *magic1 = extract_bits(data, 0, 6); //appears to be the function code sent to the encoder
    *referenced = (0 == extract_bits(data, 6, 1)); //becomes 0 once first index pulse is seen
    *magic2 = extract_bits(data, 7, 10); //unknown
    *slowclock = extract_bits(data, 16, -8); //counts up every 1/5 of a second
    *fastclock = extract_bits(data, 24, -24); //appears to be random data that changes every request
    *magic3 = extract_bits(data, 48, 8); //unknown
    *encoder_count =  extract_bits(data, 56, -24); //absolute turn counter
    *z =  extract_bits(data, 80, 1); //index 
    *u =  extract_bits(data, 81,1);  //hall u
    *v = extract_bits(data, 82, 1);  //hall v
    *w = extract_bits(data, 83, 1);  //hall w
    *ref_offset = extract_bits(data, 84, 9); //reference offset 
    *z_counter = extract_bits(data, 93, -3);  //8 bit z cross counter
    *crc = extract_bits(data, 96, 16);
}



//converts hall signals to rotor position
//pos can be 0 1 or 2 to compute low edge, middle or high edge
hal_s32_t hallRotorCount(hal_u32_t rotor_pulses, hal_u32_t edge, hal_bit_t U, hal_bit_t V, hal_bit_t W) { 
    hal_u32_t base = 0;
    
    switch((U << 2) + (V << 1) + W) {
        case 5:   //101 = 0/12 - 2/12 
            base = 2;
            break;
        case 1: //001 = 2/12 - 4/12
            base = 4;
            break;
        case 3: //011 = 4/12 - 6/12
            base = 6;
            break;
        case 2: //010 = 6/12 - 8/12
            base = 8;
            break;
        case 6: //110 = 8/12 - 10/12
            base = 10;
            break;
        case 4: //100 = 10/12 - 12/12
            base = 0;
           break;
    }

    return (rotor_pulses * (base + edge)) / 12;
}


int defineHalPin(hostmot2_t *hm2, int inst, const char* name, hal_type_t type, hal_pin_dir_t dir, void** addr) {
    int ret;
    char pinname[HAL_NAME_LEN];

    if(rtapi_snprintf(pinname, HAL_NAME_LEN, "%s.sigma5abs.%02d.%s", hm2->llio->name, inst, name) == HAL_NAME_LEN) {
        HM2_ERR("sigma5abs: Formatted pin name too long. %s.sigma5abs.%02d.%s\n", hm2->llio->name, inst, name);
        return -1;
    }

    ret = hal_pin_new(pinname, type, dir, addr, hm2->llio->comp_id);
    if (ret < 0) {
        HM2_ERR("sigma5abs: error adding pin %s. error code %d\n", name, ret);
        return -1;
    }

    return 0;
}


int defineHalPins(hostmot2_t *hm2, hm2_sigma5abs_instance_t* inst, int id) {
    if(0 > defineHalPin(hm2, id, "debug.magic1", HAL_U32, HAL_OUT, (void**)&(inst->magic1))) 
    { return -1; }

    if(0 > defineHalPin(hm2, id, "debug.magic2", HAL_U32, HAL_OUT, (void**)&(inst->magic2))) 
    { return -1; }
    
    if(0 > defineHalPin(hm2, id, "debug.magic3", HAL_U32, HAL_OUT, (void**)&(inst->magic3))) 
    { return -1; }

    if(0 > defineHalPin(hm2, id, "debug.slowclock",        HAL_U32, HAL_OUT, (void**)&(inst->slowclock)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.fastclock",        HAL_U32, HAL_OUT, (void**)&(inst->fastclock)))
    { return -1; }
     
    if(0 > defineHalPin(hm2, id, "debug.z-counter",            HAL_U32, HAL_OUT, (void**)&(inst->z_counter)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.crc",              HAL_U32, HAL_OUT, (void**)&(inst->crc)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.tx-count",         HAL_U32, HAL_IN, (void**)&(inst->tx_count)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.tx0",              HAL_U32, HAL_IN, (void**)&(inst->tx0)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.tx1",              HAL_U32, HAL_IN, (void**)&(inst->tx1)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.rx-count",         HAL_U32, HAL_OUT, (void**)&(inst->rx_count)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.rx0",              HAL_U32, HAL_OUT, (void**)&(inst->rx0)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.rx1",              HAL_U32, HAL_OUT, (void**)&(inst->rx1)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.rx2",              HAL_U32, HAL_OUT, (void**)&(inst->rx2)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.rx3",              HAL_U32, HAL_OUT, (void**)&(inst->rx3)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.rx4",              HAL_U32, HAL_OUT, (void**)&(inst->rx4)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.reference-offset", HAL_U32, HAL_OUT, (void**)&(inst->reference_offset)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.reference-angle", HAL_U32, HAL_OUT, (void**)&(inst->reference_angle)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.rotor-count",  HAL_U32, HAL_OUT, (void**)&(inst->rotor_count)))
    { return -1; }
    
     if(0 > defineHalPin(hm2, id, "debug.raw-angle",  HAL_U32, HAL_OUT, (void**)&(inst->raw_angle)))
    { return -1; }
 
    if(0 > defineHalPin(hm2, id, "debug.status",   HAL_U32, HAL_OUT, (void**)&(inst->status)))
    { return -1; }
 
    if(0 > defineHalPin(hm2, id, "busy",  HAL_BIT, HAL_OUT, (void**)&(inst->busy)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "bitrate",  HAL_U32, HAL_IN, (void**)&(inst->bitrate)))
    { return -1; }
  
    if(hm2->dpll_module_present) {
        if(0 > defineHalPin(hm2, id, "timer-number", HAL_S32, HAL_IN, (void**)&(inst->timer)))
        { return -1; }
    }
  
    if(0 > defineHalPin(hm2, id, "ppr",          HAL_U32, HAL_IN, (void**)&(inst->ppr)))
    { return -1; }
 
    if(0 > defineHalPin(hm2, id, "enable", HAL_BIT, HAL_IN, (void**)&(inst->enable)))
    { return -1; }
  
    if(0 > defineHalPin(hm2, id, "reset",        HAL_BIT, HAL_IN, (void**)&(inst->reset)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "fault",        HAL_BIT, HAL_OUT, (void**)&(inst->fault)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "fault-count",  HAL_U32, HAL_OUT, (void**)&(inst->fault_count)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "fault-inc",    HAL_U32, HAL_IN, (void**)&(inst->fault_inc)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "fault-dec",    HAL_U32, HAL_IN, (void**)&(inst->fault_dec)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "fault-lim",    HAL_U32, HAL_IN, (void**)&(inst->fault_lim)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "rawcounts",    HAL_S32, HAL_OUT, (void**)&(inst->rawcounts)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "index-enable", HAL_BIT, HAL_IO,  (void**)&(inst->index_enable)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "velocity",     HAL_FLOAT, HAL_OUT, (void**)&(inst->velocity)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "position",     HAL_FLOAT, HAL_OUT, (void**)&(inst->position)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "scale",        HAL_FLOAT, HAL_IN, (void**)&(inst->scale)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "referenced",   HAL_BIT, HAL_OUT, (void**)&(inst->referenced)))
    { return -1; }
   
 
    //commutation pins 
    if(0 > defineHalPin(hm2, id, "comm.lead-angle",  HAL_FLOAT, HAL_IN, (void**)&(inst->lead_angle)))
    { return -1;}

    if(0 > defineHalPin(hm2, id, "comm.pole-count",   HAL_U32, HAL_IN, (void**)&(inst->pole_count)))
    { return -1; }
 
    if(0 > defineHalPin(hm2, id, "comm.rotor-alignment",  HAL_FLOAT, HAL_IN, (void**)&(inst->rotor_alignment)))
    { return -1;}

    if(0 > defineHalPin(hm2, id, "comm.rotor-angle",  HAL_FLOAT, HAL_IN, (void**)&(inst->rotor_angle)))
    { return -1;}

    if(0 > defineHalPin(hm2, id, "comm.trapezoidal",  HAL_BIT, HAL_IN, (void**)&(inst->trapezoidal)))
    { return -1; }
 
     if(0 > defineHalPin(hm2, id, "comm.u",            HAL_BIT, HAL_OUT, (void**)&(inst->u)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "comm.v",            HAL_BIT, HAL_OUT, (void**)&(inst->v)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "comm.w",            HAL_BIT, HAL_OUT, (void**)&(inst->w)))
    { return -1; }
 
    if(0 > defineHalPin(hm2, id, "comm.z",            HAL_BIT, HAL_OUT, (void**)&(inst->z)))
    { return -1; }
  
  

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
        { "tx1",      false, 5,  &sg5->tx1_addr,      &sg5->tx1_reg },

        //control register must be written last
        { "control",  false, 0,  &sg5->control_addr,  &sg5->control_reg },

        { "rx_count", true,  6,  &sg5->rx_count_addr, &sg5->rx_count_reg },
        { "rx0",      true,  7,  &sg5->rx0_addr,      &sg5->rx0_reg },
        { "rx1",      true,  8,  &sg5->rx1_addr,      &sg5->rx1_reg },
        { "rx2",      true,  9,  &sg5->rx2_addr,      &sg5->rx2_reg },
        { "rx3",      true,  10, &sg5->rx3_addr,      &sg5->rx3_reg },
        { "rx4",      true,  11, &sg5->rx4_addr,      &sg5->rx4_reg },
        { "status",   true,  12, &sg5->status_addr,   &sg5->status_reg }
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

    if (!hm2_md_is_consistent_or_complain(hm2, md_index, 0, 13, 4, 0x1FFF)) {
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

        if(defineHalPins(hm2, inst, i) < 0) {
            return -1;
        }
    
        inst->prev_reset = 0;
        inst->rel_count = 0;
        inst->index_offset = 0;
        inst->prev_encoder_count = 0;
        inst->startup = 1;
       
        *inst->rawcounts = 0;
        *inst->referenced = 0;
        *inst->reference_angle = 0;
        *inst->index_enable = 0;
        *inst->position = 0.0;
        *inst->scale = 1.0;
        *inst->velocity = 0;
        *inst->slowclock = 0;
        *inst->fastclock = 0;

        if(hm2->dpll_module_present) {
            *inst->timer = -1;
        }

        *inst->z = 0;
        *inst->u = 0;
        *inst->v = 0;
        *inst->w = 0;
        *inst->z_counter = 0;
        *inst->crc = 0;
        *inst->reset = 0;
        *inst->fault = 1;
        *inst->fault_count = 100;
        *inst->fault_inc = 10;
        *inst->fault_dec = 1;
        *inst->fault_lim = 100;
        *inst->bitrate = 13;
        *inst->enable = 0;
        *inst->tx0 = 0x5555553F;
        *inst->tx1 = 0x7DF7D7EA;
        *inst->tx_count = 64;
        *inst->crc = 0;
        *inst->status = 0;
        *inst->lead_angle = 90.0;
        *inst->rotor_alignment = -30.0;
        *inst->pole_count = 8;
        *inst->rotor_angle = 0;
        *inst->rotor_count = 0;
        *inst->ppr = 2097152; //encoder pulses per turn
        *inst->trapezoidal = 0;
    }
	
   
    return hm2->sigma5abs.num_instances;
}

void hm2_sigma5abs_cleanup(hostmot2_t* hm2) {
}


void hm2_sigma5abs_prepare_tram_write(hostmot2_t* hm2) {
    hm2_sigma5abs_instance_t* inst;
    //control register
    //bit 0 - enable
    //bit 1 - transmit
    //bit 2 - timer enable

    for(int i = 0; i < hm2->sigma5abs.num_instances; i++) {
        inst = &hm2->sigma5abs.instances[i];

        //important: transmit must be written after tx values
        hm2->sigma5abs.bitrate_reg[i] = *inst->bitrate;
        hm2->sigma5abs.tx_count_reg[i] = *inst->tx_count;
        hm2->sigma5abs.tx0_reg[i] = *inst->tx0;
        hm2->sigma5abs.tx1_reg[i] = *inst->tx1;

        if(hm2->dpll_module_present) {
            //enable timer control
            hm2->sigma5abs.control_reg[i] |= 0x4;

            int32_t dpll_timer_num = *inst->timer;
            if(dpll_timer_num >= 0 && dpll_timer_num <= 4) {
                hm2->sigma5abs.timer_reg[i] = dpll_timer_num;
            } else {
                hm2->sigma5abs.timer_reg[i] = 0;
            }

         } else {
            //disable timer control
            hm2->sigma5abs.control_reg[i] &= ~0x4;

            //alternate transmit bit every cycle to trigger manchester trx
            hm2->sigma5abs.control_reg[i] ^= 0x2;
        }

        
        if(*inst->enable && (!*inst->fault || inst->startup)) {
            hm2->sigma5abs.control_reg[i] |= 0x1;
        } else {
            hm2->sigma5abs.control_reg[i] &= ~0x1;
        }
            
   }
}




void hm2_sigma5abs_process_rx(hostmot2_t* hm2, hm2_sigma5abs_instance_t* inst, int i, hal_float_t fPeriods, const char* prefix) {
    char reg[20] = {0};  //raw data passed from manchester trx with an hdlc message encoded in it
    char data[14] = {0}; //actual encoder data parsed from reg
  
 
    hal_bit_t prev_referenced = *inst->referenced;
    hal_u32_t prev_z_counter = *inst->z_counter;


    int counter_bits = 24;
    rtapi_s64 prev_rel_count = inst->rel_count;
    rtapi_s64 dCounts = 0;
    

    hal_u32_t encoder_count = 0;
    hal_u32_t encoder_angle = 0;
    hal_s32_t lead_angle_counts = 0;
    hal_s32_t rotor_alignment_counts = 0;


    rtapi_s64 wrap_counts = (1U << counter_bits);
    rtapi_s64 wrap_lower = (1U << (counter_bits-2));
    rtapi_s64 wrap_upper = wrap_counts - wrap_lower;
    rtapi_s64 counter_rollover = 0;

    rtapi_s64 direction = 0;

    rtapi_s64 rotor_pulses = 0; 
    

    //reset on rising edge of reset pin
    if(!inst->prev_reset && *inst->reset && *inst->fault) {
        HM2_ERR("%s.reset\n", prefix);
        inst->startup = 1;
        
    }

    inst->prev_reset = *inst->reset;

    if(inst->startup) {
        //during startup we 
        //HM2_ERR("startup %s fault_count %d\n", prefix, *inst->fault_count);

        if(*inst->fault_count == 0) {
            inst->startup = 0;
            *inst->fault = 0;
        } else if(*inst->fault_count > *inst->fault_lim) {
            *inst->fault_count = *inst->fault_lim;
        }

    } else {
        if(*inst->fault) {
            return;
        } else if (*inst->fault_count > *inst->fault_lim) {
            *inst->fault = 1;
            HM2_ERR("%s Too many encoder communication faults.\n", prefix);
            return;
        }
    }    


 
    *inst->status = (hal_u32_t)hm2->sigma5abs.status_reg[i];

    //bit 3 is busy bit
    *inst->busy = (*inst->status & 0x08) ? 1 : 0;


    //check for start conditions
    if(!*inst->fault && *inst->busy) {
        HM2_ERR("%s is still receiving at thread read time. Dpll timer settings are incorrect.\n", prefix);
        *inst->fault = 1;
        inst->startup = 0;
        return;
    }
 
    if(*inst->pole_count == 0) {
        HM2_ERR("%s.pole_count is invalid. Must be greater than 0.\n", prefix);
        *inst->fault = 1;
        inst->startup = 0;
        return;
    }

    if(*inst->ppr == 0) {
         HM2_ERR("%s.ppr is invalid. Must be greater than 0.\n", prefix);
        *inst->fault = 1;
        inst->startup = 0;
        return;
    }

    if(fabs(*inst->lead_angle) >= 360.0) {
        HM2_ERR("%s.lead_angle is invalid. Must be between -360.0 and 360.0 .\n", prefix);
        *inst->fault = 1;
        inst->startup = 0;
        return;
    }


    if(fabs(*inst->rotor_alignment) >= 360.0) {
        HM2_ERR("%s.rotor-alignment is invalid. Must be between -360.0 and 360.0 .\n", prefix);
        *inst->fault = 1;
        inst->startup = 0;
        return;
    }

       //scaled position value for motion controller 
    if(*inst->scale == 0.0) {
        HM2_ERR("%s.scale of 0.0 is invalid.\n", prefix);
        *inst->fault = 1;
        inst->startup = 0;
        return;
    }


   

    *inst->rx_count = (hal_u32_t)hm2->sigma5abs.rx_count_reg[i];
    *inst->rx0 = (hal_u32_t)hm2->sigma5abs.rx0_reg[i];
    *inst->rx1 = (hal_u32_t)hm2->sigma5abs.rx1_reg[i];
    *inst->rx2 = (hal_u32_t)hm2->sigma5abs.rx2_reg[i];
    *inst->rx3 = (hal_u32_t)hm2->sigma5abs.rx3_reg[i];
    *inst->rx4 = (hal_u32_t)hm2->sigma5abs.rx4_reg[i];

    *(hal_u32_t*)(reg+(0*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx0_reg[i]);
    *(hal_u32_t*)(reg+(1*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx1_reg[i]);
    *(hal_u32_t*)(reg+(2*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx2_reg[i]);
    *(hal_u32_t*)(reg+(3*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx3_reg[i]);
    *(hal_u32_t*)(reg+(4*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx4_reg[i]);


    if(parse_hdlc(reg, 20, data, 14) != 112) {
            *inst->fault_count += *inst->fault_inc; //no hdlc data
//            HM2_ERR("%s, BAD HDLC!\n", prefix);
        return;
    }

    decode_encoder_packet(data,
                          inst->magic1,
                          inst->referenced,
                          inst->magic2,
                          inst->slowclock,
                          inst->fastclock,
                          inst->magic3,
                          &encoder_count,
                          inst->z,    
                          inst->u,
                          inst->v,
                          inst->w,
                          inst->reference_offset,
                          inst->z_counter,
                          inst->crc);
  
    if(crc(data, 12) != *inst->crc) {                                   
        *inst->fault_count += *inst->fault_inc; //bad crc
        HM2_ERR("%s BAD CRC!\n", prefix);
        return;
    }

    if(prev_referenced && !*inst->referenced) {
        *inst->fault = 1;
        *inst->referenced = 0;
        *inst->reference_angle = 0;
        *inst->reference_offset = 0;
        HM2_ERR("%s reference was lost. Rehoming necessary.\n", prefix);
        return;
    } 

    if(inst->prev_encoder_count > wrap_upper && encoder_count < wrap_lower) {
        //positive rollover
        counter_rollover = wrap_counts; 
    } else if(inst->prev_encoder_count < wrap_lower && encoder_count > wrap_upper) {
        //negative_rollover
        counter_rollover = -wrap_counts;
    }

    dCounts = encoder_count - inst->prev_encoder_count + counter_rollover;
    inst->rel_count += dCounts;
    inst->prev_encoder_count = encoder_count;
    *inst->rawcounts = inst->rel_count;


    encoder_angle = encoder_count % *inst->ppr;

    //direction of travel since last update
    direction = prev_rel_count <= inst->rel_count ? 1 : -1;

   *inst->raw_angle =  ((*inst->ppr << 2)  //avoid negative modulo
                        + *inst->reference_angle
                        - encoder_angle
                        ) % *inst->ppr;
 

    /*
      when the encoder first powers on it does not know where the index point is.
      the first time that the index is passed a flag and an offset value is recorded in the encoders memory.
      The offset is the number of counts passed before the encoder packet was transmitted
      depending upon the direction of travel we must add or subtract that offset to determine
      the absolute reference position.
    */
    if(!prev_referenced  && *inst->referenced) {
       *inst->reference_angle = (encoder_angle
                                   - (direction * *inst->reference_offset) 
                                   + *inst->ppr //avoid negative remainder
                                                ) % *inst->ppr;
    }
                    
        
    /*
        z_count changes only when a full rotation past the index point is made
        once z_count changes and index_enable and the encoder is referenced
        we set the index to the just crossed index point
    */  
    if(*inst->index_enable && *inst->referenced && (prev_z_counter != *inst->z_counter)) {

        if(encoder_angle < *inst->reference_angle) {
            inst->index_offset = inst->rel_count + *inst->reference_angle - encoder_angle;
        } else {
            inst->index_offset = inst->rel_count + *inst->ppr - encoder_angle + *inst->reference_angle;
        }
          
        *inst->index_enable = 0;
    }
    
    
       
    *inst->position = ((hal_float_t)(inst->rel_count - inst->index_offset)) / *inst->scale;

    *inst->velocity = (dCounts / *inst->scale) / (hm2->sigma5abs.time - inst->time);


              
    rotor_pulses = *inst->ppr / (*inst->pole_count / 2);
    lead_angle_counts = (*inst->lead_angle * ((hal_float_t)rotor_pulses)) / 360.0;
    rotor_alignment_counts = (*inst->rotor_alignment * ((hal_float_t)rotor_pulses)) / 360.0;    


    /*
        motor commutation
        rotor_angle
    */
    if(!*inst->trapezoidal && *inst->referenced) {
        //sinusoidal commutation
        *inst->rotor_count = ((rotor_pulses << 2)        //avoid negative remainder
                                + encoder_angle
                                + lead_angle_counts
                                - rotor_alignment_counts
                                - *inst->reference_angle
                                ) % rotor_pulses;
    } else {
        //trapezoidal commutation
        *inst->rotor_count = ((rotor_pulses << 2) //avoid negative remainder
                                + hallRotorCount(rotor_pulses, 0, *inst->u, *inst->v, *inst->w)
                                + lead_angle_counts
                                ) % rotor_pulses;
    }


    *inst->rotor_angle = ((hal_float_t)*inst->rotor_count) / ((hal_float_t)rotor_pulses);

    if(*inst->fault_count > *inst->fault_dec) {
        *inst->fault_count -= *inst->fault_dec;
    } else {
        *inst->fault_count = 0;
    }

    inst->time = hm2->sigma5abs.time;
}



void hm2_sigma5abs_process_tram_read(hostmot2_t* hm2, long periodns) {
	hm2_sigma5abs_instance_t* inst;
    char prefix[HAL_NAME_LEN];
    
    hal_float_t fPeriod_s = (hal_float_t)(periodns * 1e-9);
    hm2->sigma5abs.time += fPeriod_s;
    

    for(int i = 0; i < hm2->sigma5abs.num_instances; i++) {
        inst = &hm2->sigma5abs.instances[i];
        rtapi_snprintf(prefix, sizeof(prefix), "%s.sigma5abs.%02d", hm2->llio->name, i);

        hm2_sigma5abs_process_rx(hm2, inst, i, fPeriod_s, prefix);
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
}


