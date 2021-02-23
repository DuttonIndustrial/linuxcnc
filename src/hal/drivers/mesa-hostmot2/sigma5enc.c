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


#define hall_filter_max 5

typedef struct {
    char name[HAL_NAME_LEN];
    hal_type_t type;
    hal_pin_dir_t dir;
    void** addr;
} pin_creation_data_t;

typedef struct {
    char name[HAL_NAME_LEN];
    hal_type_t type;
    hal_param_dir_t dir;
    void* addr;
} param_creation_data_t;

typedef struct {
    char name[HAL_NAME_LEN];
    bool read;
    rtapi_u32 reg_index;
    rtapi_u32* addr;
    rtapi_u32** reg;
} tram_creation_data_t;


//returns modular difference from b to a
hal_float_t angle_sub(hal_float_t a, hal_float_t b) {
    hal_float_t ret;

    if(a >= b) {
        ret = a - b;
    } else {
        ret = 1.0 - b + a;
    }

    return fmod(ret, 1.0);
}

//returns shortest distance from a to b.
//Positive result means that a is behind b
//Negative result means that a is ahead b
//b - d = a
hal_float_t angle_diff(hal_float_t a, hal_float_t b) {
    hal_float_t s =  angle_sub(a, b);
    hal_float_t ret;

    if(s >= 0.5) {
        ret =  1.0 - s;
    } else {
        ret = -s;
    }

    return fmod(ret, 1.0);
}

hal_float_t angle_med(hal_float_t a, hal_float_t b) {
    hal_float_t s = angle_diff(a, b) / 2;
    return fmod(a + s, 1.0);
}

hal_float_t rotor_angle_to_angle(hal_float_t rotor_angle, unsigned int pole_count) {
    return rotor_angle / (hal_float_t)(pole_count / 2);
}

hal_float_t angle_to_rotor_angle(hal_float_t angle, unsigned int pole_count) {
    return fmod(angle * (hal_float_t)(pole_count / 2), 1.0);
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

//length of data must be 12 bytes
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
                          hal_u32_t* ref_data,
                          hal_u32_t* z_counter)
{
    *magic1 = extract_bits(data, 0, 6); //appears to be the function code sent to the encoder
    *referenced = (0 == extract_bits(data, 6, 1)); //becomes 0 once first index pulse is seen
    *magic2 = extract_bits(data, 7, 10); //unknown
    *slowclock = extract_bits(data, 16, -8); //counts up every 1/5 of a second
    *fastclock = extract_bits(data, 24, -24); //appears to be random data that changes every request
    *magic3 = extract_bits(data, 48, -8); //unknown
    *encoder_count =  extract_bits(data, 56, -24); //absolute turn counter
    *z =  extract_bits(data, 80, 1); //index 
    *u =  extract_bits(data, 81, 1);  //hall u
    *v = extract_bits(data, 82,  1);  //hall v
    *w = extract_bits(data, 83,  1);  //hall w
    *ref_data = extract_bits(data, 84, -9); //reference data 
    *z_counter = extract_bits(data, 93, -3);  //8 bit z cross counter
}



//converts hall signals to rotor position
//pos can be 0 1 or 2 to compute low, middle or high edge
hal_float_t hallRotorAngle(hal_u32_t edge, hal_bit_t U, hal_bit_t V, hal_bit_t W) { 
    hal_float_t base = 0;
    
    switch((U << 2) + (V << 1) + W) {
       case 0b100: //100 = 2/12 - 4/12
            base = 2;
            break;
       case 0b101: //101 = 4/12 - 6/12 
            base = 4;
            break;
        case 0b001: //001 = 6/12 - 8/12
            base = 6;
            break;
        case 0b011: //011 = 8/12 - 10/12
            base = 8;
            break;
        case 0b010: //010 = 10/12 - 12/12
            base = 10;
            break;
        case 0b110: //110 = 12/12 - 2/12
            base = 12;
            break;
    }
    
    return fmod((base  + edge)/12.0, 1.0);
}


int defineHalPin(hostmot2_t *hm2, int inst, const char* name, hal_type_t type, hal_pin_dir_t dir, void** addr) {
    int ret;
    char fullname[HAL_NAME_LEN];

    if(rtapi_snprintf(fullname, HAL_NAME_LEN, "%s.sigma5enc.%02d.%s", hm2->llio->name, inst, name) == HAL_NAME_LEN) {
        HM2_ERR("sigma5enc: Formatted pin name too long. %s.sigma5enc.%02d.%s\n", hm2->llio->name, inst, name);
        return -1;
    }

    ret = hal_pin_new(fullname, type, dir, addr, hm2->llio->comp_id);

    if (ret < 0) {
        HM2_ERR("sigma5enc: error adding pin %s. error code %d\n", fullname, ret);
        return -1;
    }

    return 0;
}


int defineHalParam(hostmot2_t *hm2, int inst, const char* name, hal_type_t type, hal_param_dir_t dir, void* addr) {
    int ret;
    char fullname[HAL_NAME_LEN];

    if(rtapi_snprintf(fullname, HAL_NAME_LEN, "%s.sigma5enc.%02d.%s", hm2->llio->name, inst, name) == HAL_NAME_LEN) {
        HM2_ERR("sigma5enc: Formatted param name too long. %s.sigma5enc.%02d.%s\n", hm2->llio->name, inst, name);
        return -1;
    }

    ret = hal_param_new(fullname, type, dir, addr, hm2->llio->comp_id);

    if (ret < 0) {
        HM2_ERR("sigma5enc: error adding param %s. error code %d\n", fullname, ret);
        return -1;
    }

    return 0;
}


int defineHalPins(hostmot2_t *hm2, hm2_sigma5enc_instance_t* inst, int id) {
    //pins
    pin_creation_data_t pindefs[] = {
        {"debug.any_data",          HAL_BIT,    HAL_OUT, (void**)&inst->any_data},
        {"debug.busy",              HAL_BIT,    HAL_OUT, (void**)&inst->busy},
        {"debug.crc",               HAL_U32,    HAL_OUT, (void**)&inst->crc},
        {"debug.data-valid",        HAL_BIT,    HAL_OUT, (void**)&inst->data_valid},
        {"debug.fastclock",         HAL_U32,    HAL_OUT, (void**)&inst->fastclock},
        {"debug.magic1",            HAL_U32,    HAL_OUT, (void**)&inst->magic1},
        {"debug.magic2",            HAL_U32,    HAL_OUT, (void**)&inst->magic2},
        {"debug.magic3",            HAL_U32,    HAL_OUT, (void**)&inst->magic3},
        {"debug.raw-count",         HAL_U32,    HAL_OUT, (void**)&inst->raw_count},
        {"debug.raw-angle",         HAL_FLOAT, HAL_OUT, (void**)&inst->raw_angle},
        {"debug.reference-base",    HAL_FLOAT,    HAL_OUT, (void**)&inst->reference_base},
        {"debug.reference-angle",   HAL_FLOAT,  HAL_OUT, (void**)&inst->reference_angle},
        {"debug.reference-data",    HAL_U32,    HAL_OUT, (void**)&inst->reference_data},
        {"debug.hall-angle",        HAL_FLOAT,    HAL_OUT, (void**)&inst->hall_angle},
        {"debug.raw-rotor-angle",   HAL_FLOAT,    HAL_OUT, (void**)&inst->raw_rotor_angle},
        {"debug.raw-rotor-offset",  HAL_FLOAT, HAL_OUT, (void**)&inst->raw_rotor_offset},
        {"debug.rotor-offset",      HAL_FLOAT,    HAL_OUT, (void**)&inst->rotor_offset},
        {"debug.rotor-offset-pos",  HAL_FLOAT,    HAL_OUT, (void**)&inst->rotor_offset_pos},
        {"debug.rotor-offset-neg",  HAL_FLOAT,    HAL_OUT, (void**)&inst->rotor_offset_neg},
        {"debug.rx0",               HAL_U32,    HAL_OUT, (void**)&inst->rx0},
        {"debug.rx1",               HAL_U32,    HAL_OUT, (void**)&inst->rx1},
        {"debug.rx2",               HAL_U32,    HAL_OUT, (void**)&inst->rx2},
        {"debug.slowclock",         HAL_U32,    HAL_OUT, (void**)&inst->slowclock},
        {"debug.status",            HAL_U32,    HAL_OUT, (void**)&inst->status},
        {"debug.z-counter",         HAL_U32,    HAL_OUT, (void**)&inst->z_counter},
        {"fault",                   HAL_BIT,    HAL_OUT, (void**)&inst->fault},
        {"fault-count",             HAL_U32,    HAL_OUT, (void**)&inst->fault_count},
        {"index-enable",            HAL_BIT,    HAL_IO,  (void**)&inst->index_enable},
        {"position",                HAL_FLOAT,  HAL_OUT, (void**)&inst->position},
        {"referenced",              HAL_BIT,    HAL_OUT, (void**)&inst->referenced},
        {"rotor-angle",             HAL_FLOAT,  HAL_OUT, (void**)&inst->rotor_angle},
        {"angle",                   HAL_FLOAT,  HAL_OUT, (void**)&inst->angle},
        {"run",                     HAL_BIT,    HAL_IN,  (void**)&inst->run},
        {"u",                       HAL_BIT,    HAL_OUT, (void**)&inst->u},
        {"v",                       HAL_BIT,    HAL_OUT, (void**)&inst->v},
        {"velocity",                HAL_FLOAT,  HAL_OUT, (void**)&inst->velocity},
        {"w",                       HAL_BIT,    HAL_OUT, (void**)&inst->w},
        {"z",                       HAL_BIT,    HAL_OUT, (void**)&inst->z}
    };


    for(int index = 0; index < sizeof(pindefs)/sizeof(pin_creation_data_t); index++) {
       if(0 > defineHalPin(hm2, id, pindefs[index].name, pindefs[index].type, pindefs[index].dir, pindefs[index].addr)) {
            return -1;
        }
    }

    return 0;
}

int defineHalParams(hostmot2_t *hm2, hm2_sigma5enc_instance_t* inst, int id) {

    //params
    param_creation_data_t paramdefs[] = {
        {"ppr",        HAL_U32,   HAL_RW, (void*)&inst->ppr},
        {"fault-inc",  HAL_U32,   HAL_RW, (void*)&inst->fault_inc},
        {"fault-dec",  HAL_U32,   HAL_RW, (void*)&inst->fault_dec},
        {"fault-lim",  HAL_U32,   HAL_RW, (void*)&inst->fault_lim},
        {"pole-count", HAL_U32,   HAL_RW, (void*)&inst->pole_count},
        {"scale",      HAL_FLOAT,  HAL_RW,(void*)&inst->scale},

    };


    for(int index = 0; index < sizeof(paramdefs)/sizeof(param_creation_data_t); index++) {
        if(0 > defineHalParam(hm2, id, paramdefs[index].name, paramdefs[index].type, paramdefs[index].dir, paramdefs[index].addr)) {
            return -1;
        }
    }

 
    if(hm2->dpll_module_present) {
        if(defineHalParam(hm2, id, "dpll-timer", HAL_S32, HAL_RW, (void*)&(inst->dpll_timer)) < 0)
        { return -1;}
    }

    return 0;
}
	

        


int hm2_sigma5enc_configure_tram(hostmot2_t* hm2, hm2_module_descriptor_t *md) {
    int ret = 0;
    hm2_sigma5enc_t* sg5 = &hm2->sigma5enc;

    tram_creation_data_t td[] = {
        { "control",  false, 0,  &sg5->control_addr,  &sg5->control_reg },
        { "rx0",      true,  1,  &sg5->rx0_addr,      &sg5->rx0_reg },
        { "rx1",      true,  2,  &sg5->rx1_addr,      &sg5->rx1_reg },
        { "rx2",      true,  3,  &sg5->rx2_addr,      &sg5->rx2_reg },
        { "status",   true,  4,  &sg5->status_addr,   &sg5->status_reg }
    };


    for(int index = 0; index < sizeof(td)/sizeof(tram_creation_data_t); index++) {
        *td[index].addr = md->base_address + (td[index].reg_index * md->register_stride);
        
        *td[index].reg = (rtapi_u32*)rtapi_kmalloc(sg5->num_instances * sizeof(rtapi_u32), RTAPI_GFP_KERNEL);

        if(NULL == *td[index].reg) {
            HM2_ERR("sigma5enc: rtapi_kmalloc failed for %s.", td[index].name);
            return -1;
        }

        if(td[index].read) {
            ret = hm2_register_tram_read_region(hm2, *td[index].addr, sg5->num_instances * sizeof(rtapi_u32), td[index].reg);

            if (ret < 0) {
                HM2_ERR("sigma5enc: error registering tram read region for register %s (Code: %d)\n", td[index].name, ret);
                return -1;
            }
        } else {
            ret = hm2_register_tram_write_region(hm2, *td[index].addr, sg5->num_instances * sizeof(rtapi_u32), td[index].reg);
            if (ret < 0) {
                HM2_ERR("sigma5enc: error registering tram write region for register %s (Code: (%d)\n", td[index].name, ret);
                return -1;
            }
        }
    }

    return 0;
}


int hm2_sigma5enc_parse_md(hostmot2_t *hm2, int md_index) 
{
    int i, r = -EINVAL;
    hm2_module_descriptor_t *md = &hm2->md[md_index];
    static int last_gtag = -1;

    if (!hm2_md_is_consistent_or_complain(hm2, md_index, 0, 5, 4, 0x001F)) {
        HM2_ERR("inconsistent Module Descriptor!\n");
        return -EINVAL;
    }
    
    if (hm2->sigma5enc.num_instances > 1 && last_gtag == md->gtag) {
        HM2_ERR("found duplicate Module Descriptor for %s (inconsistent firmware), not loading driver %i %i\n",
                hm2_get_general_function_name(md->gtag), md->gtag, last_gtag);
        return -EINVAL;
    }

    last_gtag = md->gtag;

    if (hm2->config.num_sigma5enc > md->instances) {
        HM2_ERR("config defines %d sigma5enc, but only %d are available, not loading driver\n",
                hm2->config.num_sigma5enc,
                md->instances);
        return -EINVAL;
    }
    
    if (hm2->config.num_sigma5enc == 0) {
        return 0;
    }


    // 
    // looks good, start, or continue, initializing
    // 
    if (hm2->config.num_sigma5enc == -1) {
        hm2->sigma5enc.num_instances = md->instances;
    } else {
        hm2->sigma5enc.num_instances = hm2->config.num_sigma5enc;
    }
        
    hm2->sigma5enc.instances = (hm2_sigma5enc_instance_t *)hal_malloc(hm2->sigma5enc.num_instances * sizeof(hm2_sigma5enc_instance_t));
    if (NULL == hm2->sigma5enc.instances) {
        HM2_ERR("sigma5enc: out of hal memory!\n");
        return -ENOMEM;
    }

    hm2->sigma5enc.instance_stride = md->instance_stride;

    r = hm2_sigma5enc_configure_tram(hm2, md);
    if(r < 0) {
        return r;
    }

    for (i = 0 ; i < hm2->sigma5enc.num_instances ; i++){
        hm2_sigma5enc_instance_t *inst = &hm2->sigma5enc.instances[i];

        if(defineHalPins(hm2, inst, i) < 0) {
            return -1;
        }
 
       if(defineHalParams(hm2, inst, i) < 0) {
            return -1;
        }
   
  
        //private variables 
        inst->full_count = 0;
        inst->index_offset = 0;
        inst->prev_encoder_count = 0;
        *inst->reference_base = 0;
        inst->hall_filter_count = hall_filter_max;
        inst->startup = 1;
        inst->time = 0;

        //paramaters
        inst->dpll_timer = -1;
        inst->fault_dec = 1;
        inst->fault_inc = 10;
        inst->fault_lim = 200;
        inst->pole_count = 8;
        inst->ppr = 2097152; //2^21
        inst->scale = 1.0;

        //pins
        *inst->fault = 1;
        *inst->fault_count = inst->fault_lim;
        *inst->index_enable = 0;
        *inst->position = 0.0;
        *inst->referenced = 0;
        *inst->rotor_angle = 0;
        *inst->angle = 0.0;
        *inst->run = 0;
        *inst->u = 0;
        *inst->v = 0;
        *inst->velocity = 0;
        *inst->w = 0;
        *inst->z = 0;
 


        //debug pins
        *inst->any_data = 0;
        *inst->busy = 0;
        *inst->crc = 0;
        *inst->data_valid = 0;
        *inst->fastclock = 0;
        *inst->magic1 = 0;
        *inst->magic2 = 0;
        *inst->magic3 = 0;
        *inst->raw_count = 0;
        *inst->reference_angle = 0;
        *inst->reference_data = 0;
        *inst->rotor_offset = 0;
        *inst->rotor_offset_pos = 0;
        *inst->rotor_offset_neg = 0;
        *inst->rx0 = 0;
        *inst->rx1 = 0;
        *inst->rx2 = 0;
        *inst->slowclock = 0;
        *inst->status = 0;
        *inst->z_counter = 0;
    }
	
   
    return hm2->sigma5enc.num_instances;
}


void hm2_sigma5enc_prepare_tram_write(hostmot2_t* hm2) {
    hm2_sigma5enc_instance_t* inst;
    
    //control register
    //bit 0 - enable
    //bit 1 - transmit
    //bit 2 - dppl timer enable
    //bit 8-16 - dpll timer number

    for(int i = 0; i < hm2->sigma5enc.num_instances; i++) {
        inst = &hm2->sigma5enc.instances[i];
        int32_t dpll_timer_num = inst->dpll_timer;


        if(hm2->dpll_module_present && dpll_timer_num >= 0 && dpll_timer_num <= 4) {
            //enable timer control
            hm2->sigma5enc.control_reg[i] |= 0x4;
            //hm2->sigma5enc.timer_reg[i] = dpll_timer_num;
            hm2->sigma5enc.control_reg[i] = (dpll_timer_num << 8) | (hm2->sigma5enc.control_reg[i] & 0xFFFF00FF);
        } else {
            //disable timer control
            hm2->sigma5enc.control_reg[i] &= ~0x4;
            hm2->sigma5enc.control_reg[i] &= 0xFFFF00FF;

            //alternate transmit bit every cycle to trigger read cycle
            hm2->sigma5enc.control_reg[i] ^= 0x2;
        }

        if(*inst->run && (inst->startup || !*inst->fault)) {
            hm2->sigma5enc.control_reg[i] |= 0x1;
        } else {
            hm2->sigma5enc.control_reg[i] &= ~0x1;
        }
            
   }
}




void hm2_sigma5enc_process_rx(hostmot2_t* hm2, hm2_sigma5enc_instance_t* inst, int i, hal_float_t fPeriods, const char* prefix) {
    char register_data[20] = {0};  
  
    hal_bit_t prev_referenced = *inst->referenced;
    hal_bit_t prev_u = *inst->u;
    hal_bit_t prev_v = *inst->v;
    hal_bit_t prev_w = *inst->w;

    hal_u32_t prev_z_counter = *inst->z_counter;

    int counter_bits = 24; //total number of bits stored in encoder

    rtapi_s64 dCounts = 0;

    //commutation lead angle converted to counts

    hal_u32_t wrap_counts = (1U << counter_bits);
    hal_u32_t wrap_lower = (1U << (counter_bits-2));
    hal_u32_t wrap_upper = wrap_counts - wrap_lower;
    
    rtapi_s64 counter_rollover = 0;


    //always report status from fpga
    *inst->status = (hal_u32_t)hm2->sigma5enc.status_reg[i];


    if(!*inst->run) {
        inst->startup = 1;
        *inst->fault_count = inst->fault_lim;
        return;
    } else {
        if(inst->startup) {
            if(*inst->fault_count == 0) {
                inst->startup = 0;
                *inst->fault = 0;
            } else if(*inst->fault_count > inst->fault_lim) {
                *inst->fault_count = inst->fault_lim;
            }
        } else if(*inst->fault) {
            return;
        } else if(*inst->fault_count > inst->fault_lim) {
            *inst->fault = 1;
            HM2_ERR("%s Too many encoder communication faults.\n", prefix);
            return;
        }
    }    
    
    *inst->rx0 = (hal_u32_t)hm2->sigma5enc.rx0_reg[i];
    *inst->rx1 = (hal_u32_t)hm2->sigma5enc.rx1_reg[i];
    *inst->rx2 = (hal_u32_t)hm2->sigma5enc.rx2_reg[i];

    //copy registers into register_data buffer
    //register data needs to be converted into big endian
    *(hal_u32_t*)(register_data+(0*sizeof(hal_u32_t))) = htobe32(hm2->sigma5enc.rx0_reg[i]);
    *(hal_u32_t*)(register_data+(1*sizeof(hal_u32_t))) = htobe32(hm2->sigma5enc.rx1_reg[i]);
    *(hal_u32_t*)(register_data+(2*sizeof(hal_u32_t))) = htobe32(hm2->sigma5enc.rx2_reg[i]);


    *inst->crc = (*inst->status >> 16);
    *inst->data_valid = (*inst->status & 0x8) ? 1 : 0;
    *inst->busy = (*inst->status & 0x10) ? 1 : 0;
    *inst->any_data = (*inst->status & 0x20) ? 1 : 0;

    //check for start conditiona
    if(*inst->busy) {
        HM2_ERR("%s is still receiving at thread read time. Dpll timer settings are incorrect.\n", prefix);
        *inst->fault = 1;
        inst->startup = 0;
        return;
    }
 
    if(inst->pole_count == 0) {
        HM2_ERR("%s.pole_count = %d is invalid. Must be greater than 0.\n", prefix, inst->pole_count);
        *inst->fault = 1;
        inst->startup = 0;
        return;
    }

    if(inst->ppr == 0) {
        HM2_ERR("%s.ppr = %d is invalid. Must be greater than 0.\n", prefix, inst->ppr);
        *inst->fault = 1;
        inst->startup = 0;
        return;
    }

    //scaled position value for motion controller 
    if(inst->scale == 0.0) {
        HM2_ERR("%s.scale of 0.0 is invalid.\n", prefix);
        *inst->fault = 1;
        inst->startup = 0;
        return;
    }
    

   if(!*inst->data_valid) {
        *inst->fault_count += inst->fault_inc;
        return;
   }
    

    decode_encoder_packet(register_data,
                          inst->magic1,
                          inst->referenced,
                          inst->magic2,
                          inst->slowclock,
                          inst->fastclock,
                          inst->magic3,
                          inst->raw_count,
                          inst->z,    
                          inst->u,
                          inst->v,
                          inst->w,
                          inst->reference_data,
                          inst->z_counter);

 
    if(prev_referenced && !*inst->referenced) {
        *inst->reference_angle = 0;
        inst->prev_encoder_count = 0;
        inst->full_count = 0;
        inst->index_offset = 0;

        //this will force realignment of rotor after next start
        *inst->u = 0;
        *inst->v = 0;
        *inst->w = 0;

        *inst->fault = 1;
        HM2_ERR("%s reference was lost. (Encoder power failure?)  Rehoming necessary.\n", prefix);
        return;
    }



    
    ///////////////////////////////////////////////////////////////////////////
    //compute commutaion data
    ///////////////////////////////////////////////////////////////////////////

    *inst->raw_angle = (hal_float_t)(*inst->raw_count % inst->ppr) / (hal_float_t)inst->ppr;

    *inst->raw_rotor_angle = angle_to_rotor_angle(*inst->raw_angle, inst->pole_count);

    if(prev_u != *inst->u || prev_v != *inst->v || prev_w != *inst->w) {
        inst->hall_filter_count = hall_filter_max;
    } else if(inst->hall_filter_count > 0) {
        inst->hall_filter_count -= 1;
    }

    //otherwise pick the center of the rotor angle
    *inst->hall_angle = hallRotorAngle(1, *inst->u, *inst->v, *inst->w);
    
    //raw_rotor_angle + raw_rotor_offset = hall_angle
    *inst->raw_rotor_offset = angle_diff(*inst->raw_rotor_angle, *inst->hall_angle);

    if(!prev_u && !prev_v && !prev_w) {
        //for the first cycle initialize starting value to center of known hall position
        *inst->rotor_offset_neg = *inst->raw_rotor_offset;
        *inst->rotor_offset_pos = *inst->raw_rotor_offset;
    } else if(inst->hall_filter_count == 0) {
        //if current_rotor_offset is more positive than rotor_offset_pos
        //we update it
        if(angle_diff(*inst->rotor_offset_pos, *inst->raw_rotor_offset) > 0.0) {
            *inst->rotor_offset_pos = *inst->raw_rotor_offset;
        } 

        //if current rotor offset is more negative than rotor_offset_neg we update it
        if(angle_diff(*inst->rotor_offset_neg, *inst->raw_rotor_offset) < 0.0) {
            *inst->rotor_offset_neg = *inst->raw_rotor_offset;
        } 
    }

    if(angle_diff(*inst->rotor_offset_pos, *inst->rotor_offset_neg) > 0.125) {
        *inst->fault = 1;
        HM2_ERR("%s rotor offset appears to have jumped.\n", prefix);
        return;
    }

    //real rotor offset is centered between rotor offset pos and neg
    *inst->rotor_offset = angle_med(*inst->rotor_offset_pos, *inst->rotor_offset_neg);

    *inst->rotor_angle = fmod(1.0 + *inst->raw_rotor_angle + *inst->rotor_offset, 1.0);


    ///////////////////////////////////////////////////////////////////////////
    //compute position data
    ///////////////////////////////////////////////////////////////////////////


    //check for encoder rollover
    if(inst->prev_encoder_count > wrap_upper && *inst->raw_count < wrap_lower) {
        //positive rollover
        counter_rollover = (rtapi_s64)wrap_counts; 
    } else if(inst->prev_encoder_count < wrap_lower && *inst->raw_count > wrap_upper) {
        //negative_rollover
        counter_rollover = -(rtapi_s64)wrap_counts;
    } else {
        //no rollover occured
        counter_rollover = 0;
    }


    dCounts = (rtapi_s64)*inst->raw_count - (rtapi_s64)inst->prev_encoder_count + counter_rollover;
    inst->full_count += dCounts;
    inst->prev_encoder_count = *inst->raw_count;
    *inst->velocity = (dCounts / inst->scale) / (hm2->sigma5enc.time - inst->time);

    
    
    //  when the encoder first powers on it does not know where the index point is until the encoder
    //  crosses the Z point.
    //  The z point is always at rotor position low edge of UVW(101).
    if(!prev_referenced  && *inst->referenced) {
          *inst->reference_base = floor(*inst->raw_angle * (hal_float_t)(inst->pole_count/2)) / (hal_float_t)(inst->pole_count/2);
    }

    //    once we know reference_pole_pair we can compute reference_angle.
    //    The reference angle will change slightly as the real rotor_offset converges over time
    if(*inst->referenced) {

      *inst->reference_angle = fmod(1.0 
                                    + *inst->reference_base
                                    + rotor_angle_to_angle(fmod(1.0 + hallRotorAngle(0, 1, 0, 1)-*inst->rotor_offset, 1.0), inst->pole_count),
                                    1.0);

       *inst->angle = fmod(1.0 + *inst->raw_angle - *inst->reference_angle, 1.0);
    }

    
    //    More than a full turn past Z point must occur before the z counter changes. 
    //   This prevents erratic homing behavior when the Z hall sensor is close 
    //    to the home switch.
    if(*inst->index_enable && *inst->referenced && (prev_z_counter != *inst->z_counter)) {
        inst->index_offset = inst->full_count - (inst->ppr * angle_diff(*inst->reference_angle, *inst->raw_angle));
        *inst->index_enable = 0;
    }
    
    *inst->position = ((hal_float_t)(inst->full_count - inst->index_offset)) / inst->scale;

    if(*inst->fault_count > inst->fault_dec) {
        *inst->fault_count -= inst->fault_dec;
    } else {
        *inst->fault_count = 0;
    }

    //last time the cycle completed to keep velocity mesaurements
    //accurate if there were read faults
    inst->time = hm2->sigma5enc.time;
}



void hm2_sigma5enc_process_tram_read(hostmot2_t* hm2, long periodns) {
	hm2_sigma5enc_instance_t* inst;
    char prefix[HAL_NAME_LEN];
    
    hal_float_t fPeriod_s = (hal_float_t)(periodns * 1e-9);
    hm2->sigma5enc.time += fPeriod_s;
    

    for(int i = 0; i < hm2->sigma5enc.num_instances; i++) {
        inst = &hm2->sigma5enc.instances[i];
        rtapi_snprintf(prefix, sizeof(prefix), "%s.sigma5enc.%02d", hm2->llio->name, i);

        hm2_sigma5enc_process_rx(hm2, inst, i, fPeriod_s, prefix);
    }        
}


void hm2_sigma5enc_print_module(hostmot2_t* hm2) {
    if (hm2->sigma5enc.num_instances <= 0) return;
    HM2_PRINT("Sigma5ABS: %d\n", hm2->sigma5enc.num_instances);
    HM2_PRINT("    Control Addr: 0x%X\n", hm2->sigma5enc.control_addr);
    HM2_PRINT("    Rx0 Addr: 0x%X\n", hm2->sigma5enc.rx0_addr);
    HM2_PRINT("    Rx1 Addr: 0x%X\n", hm2->sigma5enc.rx1_addr);
    HM2_PRINT("    Rx2 Addr: 0x%X\n", hm2->sigma5enc.rx2_addr);
    HM2_PRINT("    Status Addr: 0x%X\n", hm2->sigma5enc.status_addr);
}


