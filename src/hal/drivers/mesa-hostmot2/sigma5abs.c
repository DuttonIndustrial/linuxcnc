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


//returns a - b (mod)
unsigned int modsub(unsigned int a, unsigned int b, unsigned int mod) {

    if(a >= b) {
        return a - b;
    } else {
        return (mod - b + a);
    }
}

unsigned int modmed(unsigned int a, unsigned int b, unsigned int mod) {
    unsigned int diff = modsub(a, b, mod)/2;

    return (b + diff) % mod;
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
                          hal_u32_t* z_counter)
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
    *ref_offset = extract_bits(data, 84, -9); //reference offset 
    *z_counter = extract_bits(data, 93, -3);  //8 bit z cross counter
}



//converts hall signals to rotor position
//pos can be 0 1 or 2 to compute low edge, middle or high edge
//UVW(100) center of hall sensor is electrical 0 hallRotorCount(rotor_pulses, 1, 1, 0, 0) = 0
hal_s32_t hallRotorCount(hal_u32_t rotor_pulses, hal_u32_t edge, hal_bit_t U, hal_bit_t V, hal_bit_t W) { 
    hal_u32_t base = 0;
    
    switch((U << 2) + (V << 1) + W) {
       case 0b100: //100 = 11/12 - 1/12
            base = 11;
            break;
       case 0b101: //101 = 1/12 - 3/12 
            base = 1;
            break;
        case 0b001: //001 = 3/12 - 5/12
            base = 3;
            break;
        case 0b011: //011 = 5/12 - 7/12
            base = 5;
            break;
        case 0b010: //010 = 7/12 - 9/12
            base = 7;
            break;
        case 0b110: //110 = 9/12 - 11/12
            base = 9;
            break;
    }
    
    

    return ((rotor_pulses * ((base  + edge) % 12)) / 12) % rotor_pulses;
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
   
    if(0 > defineHalPin(hm2, id, "debug.reference-data", HAL_U32, HAL_OUT, (void**)&(inst->reference_data)))
    { return -1; }
   
    if(0 > defineHalPin(hm2, id, "debug.reference-angle", HAL_U32, HAL_OUT, (void**)&(inst->reference_angle)))
    { return -1; }

    if(0 > defineHalPin(hm2, id, "debug.reference-turn", HAL_U32, HAL_OUT, (void**)&(inst->reference_turn)))
    { return -1; }
 
   
    if(0 > defineHalPin(hm2, id, "debug.rotor-count",  HAL_U32, HAL_OUT, (void**)&(inst->rotor_count)))
    { return -1; }
    
    if(0 > defineHalPin(hm2, id, "debug.comm-count",  HAL_U32, HAL_OUT, (void**)&(inst->comm_count)))
    { return -1; }
    
    if(0 > defineHalPin(hm2, id, "debug.raw-angle",  HAL_U32, HAL_OUT, (void**)&(inst->raw_angle)))
    { return -1; }
 
    if(0 > defineHalPin(hm2, id, "debug.raw_count",     HAL_U32, HAL_OUT, (void**)&(inst->raw_count)))
    { return -1; }
 
    if(0 > defineHalPin(hm2, id, "debug.status",   HAL_U32, HAL_OUT, (void**)&(inst->status)))
    { return -1; }
    
    if(0 > defineHalPin(hm2, id, "debug.rotor-u-min",   HAL_U32, HAL_OUT, (void**)&(inst->rotor_u_min)))
    { return -1; }
 
    if(0 > defineHalPin(hm2, id, "debug.rotor-u-max",   HAL_U32, HAL_OUT, (void**)&(inst->rotor_u_max)))
    { return -1; }

    if(0 > defineHalPin(hm2, id, "debug.rotor-u-offset",   HAL_U32, HAL_OUT, (void**)&(inst->rotor_u_offset)))
    { return -1; }
    
    if(0 > defineHalPin(hm2, id, "debug.raw-rotor-count",   HAL_U32, HAL_OUT, (void**)&(inst->raw_rotor_count)))
    { return -1; }
 
    if(0 > defineHalPin(hm2, id, "debug.rotor-u-count",   HAL_U32, HAL_OUT, (void**)&(inst->rotor_u_count)))
    { return -1; }
 
    if(0 > defineHalPin(hm2, id, "debug.busy",  HAL_BIT, HAL_OUT, (void**)&(inst->busy)))
    { return -1; }
 
    if(0 > defineHalPin(hm2, id, "debug.any-data",  HAL_BIT, HAL_OUT, (void**)&(inst->any_data)))
    { return -1; }

    if(0 > defineHalPin(hm2, id, "debug.data-valid",  HAL_BIT, HAL_OUT, (void**)&(inst->data_valid)))
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
 
    if(0 > defineHalPin(hm2, id, "comm.rotor-angle",  HAL_FLOAT, HAL_OUT, (void**)&(inst->rotor_angle)))
    { return -1;}
    
    if(0 > defineHalPin(hm2, id, "comm.comm-angle",  HAL_FLOAT, HAL_OUT, (void**)&(inst->comm_angle)))
    { return -1;}
 
    if(0 > defineHalPin(hm2, id, "comm.rotor-alignment",  HAL_FLOAT, HAL_OUT, (void**)&(inst->rotor_alignment)))
    { return -1;}
   
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
        { "timer",    false, 1,  &sg5->timer_addr,    &sg5->timer_reg },
        { "control",  false, 0,  &sg5->control_addr,  &sg5->control_reg },
        { "rx_count", true,  2,  &sg5->rx_count_addr, &sg5->rx_count_reg },
        { "rx0",      true,  3,  &sg5->rx0_addr,      &sg5->rx0_reg },
        { "rx1",      true,  4,  &sg5->rx1_addr,      &sg5->rx1_reg },
        { "rx2",      true,  5,  &sg5->rx2_addr,      &sg5->rx2_reg },
        { "rx3",      true,  6, &sg5->rx3_addr,      &sg5->rx3_reg },
        { "rx4",      true,  7, &sg5->rx4_addr,      &sg5->rx4_reg },
        { "status",   true,  8, &sg5->status_addr,   &sg5->status_reg }
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

    if (!hm2_md_is_consistent_or_complain(hm2, md_index, 0, 9, 4, 0x03FF)) {
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
        inst->full_count = 0;
        inst->index_offset = 0;
        inst->prev_encoder_count = 0;
        inst->startup = 1;

        *inst->reference_data = 0;
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
        *inst->reset = 0;
        *inst->fault = 1;
        *inst->fault_lim = 20;
        *inst->fault_count = *inst->fault_lim;
        *inst->fault_inc = 10;
        *inst->fault_dec = 1;
        *inst->enable = 0;
        *inst->status = 0;
        *inst->lead_angle = 90.0;
        *inst->pole_count = 8;
        *inst->rotor_angle = 0;
        *inst->rotor_alignment = 30.0;
        *inst->comm_angle = 0;
        *inst->rotor_count = 0;
        *inst->comm_count = 0;
        *inst->ppr = 2097152; //encoder pulses per turn
        *inst->raw_count = 0;
        *inst->rotor_u_min = 0;
        *inst->rotor_u_max = 0;
        *inst->rotor_u_offset = 0;
        *inst->reference_turn = 0;

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
    char register_data[20] = {0};  //raw data passed from manchester trx with an hdlc message encoded in it
  
 
    hal_bit_t prev_referenced = *inst->referenced;
    hal_u32_t prev_z_counter = *inst->z_counter;
    hal_bit_t prev_u = *inst->u;
    hal_bit_t prev_v = *inst->v;
    hal_bit_t prev_w = *inst->w;

    int counter_bits = 24; //total number of bits stored in encoder



    rtapi_s64 dCounts = 0;
    

    hal_u32_t encoder_count = 0;

    //commutation lead angle converted to counts
    hal_s32_t lead_angle_counts = 0;


    hal_u32_t wrap_counts = (1U << counter_bits);
    hal_u32_t wrap_lower = (1U << (counter_bits-2));
    hal_u32_t wrap_upper = wrap_counts - wrap_lower;
    
    rtapi_s64 counter_rollover = 0;
    rtapi_s64 direction = 0;

    //current number of counts in "rotor" space
    hal_u32_t rotor_pulses = 0; 



    //reset on rising edge of reset pin if fault exists
    if(!inst->prev_reset && *inst->reset && *inst->fault) {
        inst->startup = 1;
    }

    inst->prev_reset = *inst->reset;

    //during startup mode we ignore any read failures
    //until fault_count reaches 0
    if(inst->startup) {
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

    
    *inst->rx_count = (hal_u32_t)hm2->sigma5abs.rx_count_reg[i];
    *inst->rx0 = (hal_u32_t)hm2->sigma5abs.rx0_reg[i];
    *inst->rx1 = (hal_u32_t)hm2->sigma5abs.rx1_reg[i];
    *inst->rx2 = (hal_u32_t)hm2->sigma5abs.rx2_reg[i];
    *inst->rx3 = (hal_u32_t)hm2->sigma5abs.rx3_reg[i];
    *inst->rx4 = (hal_u32_t)hm2->sigma5abs.rx4_reg[i];

    //copy registers into register_data buffer
    //register data needs to be converted into big endian
    *(hal_u32_t*)(register_data+(0*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx0_reg[i]);
    *(hal_u32_t*)(register_data+(1*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx1_reg[i]);
    *(hal_u32_t*)(register_data+(2*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx2_reg[i]);
    *(hal_u32_t*)(register_data+(3*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx3_reg[i]);
    *(hal_u32_t*)(register_data+(4*sizeof(hal_u32_t))) = htobe32(hm2->sigma5abs.rx4_reg[i]);


    *inst->status = (hal_u32_t)hm2->sigma5abs.status_reg[i];

    *inst->data_valid = (*inst->status & 0x8) ? 1 : 0;
    *inst->busy = (*inst->status & 0x10) ? 1 : 0;
    *inst->any_data = (*inst->status & 0x20) ? 1 : 0;

    //check for start conditiona
    if(!*inst->fault) {
        if(*inst->busy) {
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
    
       //scaled position value for motion controller 
       if(*inst->scale == 0.0) {
            HM2_ERR("%s.scale of 0.0 is invalid.\n", prefix);
            *inst->fault = 1;
            inst->startup = 0;
            return;
        }

        if(*inst->rotor_alignment < 0.0|| *inst->rotor_alignment > 360.0) {
            HM2_ERR("%s.comm.rotor-alignment value must be >= 0.0 and < 360.0\n", prefix);
            *inst->fault = 1;
            inst->startup = 0;
            return;
        }
    }


    
    if(!*inst->data_valid) {
        *inst->fault_count += *inst->fault_inc;
        return;
    }
    

    decode_encoder_packet(register_data,
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
                          inst->reference_data,
                          inst->z_counter);
 
 
    if(prev_referenced && !*inst->referenced) {
        *inst->reference_angle = 0;
        *inst->fault = 1;
        HM2_ERR("%s reference was lost. Rehoming necessary.\n", prefix);
        return;
    }


    ///////////////////////////////////////////////////////////////////////////
    //compute position data
    ///////////////////////////////////////////////////////////////////////////

    //check for encoder rollover
    if(inst->prev_encoder_count > wrap_upper && encoder_count < wrap_lower) {
        //positive rollover
        counter_rollover = (rtapi_s64)wrap_counts; 
    } else if(inst->prev_encoder_count < wrap_lower && encoder_count > wrap_upper) {
        //negative_rollover
        counter_rollover = -(rtapi_s64)wrap_counts;
    } else {
        //no rollover occured
        counter_rollover = 0;
    }


    dCounts = (rtapi_s64)encoder_count - (rtapi_s64)inst->prev_encoder_count + counter_rollover;
    inst->full_count += dCounts;
    direction = dCounts > 0 ? 1 : -1;
    inst->prev_encoder_count = encoder_count;
    *inst->position = ((hal_float_t)(inst->full_count - inst->index_offset)) / *inst->scale;
    *inst->velocity = (dCounts / *inst->scale) / (hm2->sigma5abs.time - inst->time);


    *inst->raw_count = encoder_count;
    *inst->raw_angle = encoder_count % *inst->ppr;


    

    ///////////////////////////////////////////////////////////////////////////
    //compute commutaion data
    ///////////////////////////////////////////////////////////////////////////
              
    rotor_pulses = *inst->ppr / (*inst->pole_count / 2);

    lead_angle_counts = (*inst->lead_angle * ((hal_float_t)rotor_pulses)) / 360.0;

    *inst->raw_rotor_count = encoder_count % rotor_pulses;

    *inst->rotor_u_count = modsub(*inst->raw_rotor_count,
                                  hallRotorCount(rotor_pulses, 1, *inst->u, *inst->v, *inst->w),
                                  rotor_pulses);

    //adjust rotor alignment value continuously
    //this finds the highest and lowest edge of hall transitions 
    //and finds the center of the alignment value which will be at rotor 0 position (UVW=100)
    //this finds the center point of all hall transitions to find those values
    if(!prev_u && !prev_v && !prev_w) {
        //for the first cycle initialize starting value to center of known hall position
        *inst->rotor_u_min = *inst->rotor_u_count;
        *inst->rotor_u_max = *inst->rotor_u_count; 
    } else {
        if(modsub(*inst->rotor_u_count, *inst->rotor_u_max, rotor_pulses) < (rotor_pulses / 2)) {
            *inst->rotor_u_max = *inst->rotor_u_count;
        }

        if(modsub(*inst->rotor_u_min, *inst->rotor_u_count, rotor_pulses) < (rotor_pulses / 2)) {
            *inst->rotor_u_min = *inst->rotor_u_count;
        }
    }


    *inst->rotor_u_offset = modmed(*inst->rotor_u_max, *inst->rotor_u_min, rotor_pulses);

    //actual rotor position
    *inst->rotor_count = modsub(*inst->raw_rotor_count, *inst->rotor_u_offset, rotor_pulses);
    //actual rotor angle from 0 to 1 
    *inst->rotor_angle = ((hal_float_t)*inst->rotor_count) / ((hal_float_t)rotor_pulses);
    
    
    //rotor position with lead angle added in
    *inst->comm_count = ((rotor_pulses << 2) //avoid negative remainder
                          + *inst->rotor_count
                          + lead_angle_counts
                         ) % rotor_pulses;
    
    //commitation angle from 0 to 1                     
    *inst->comm_angle = ((hal_float_t)*inst->comm_count) / ((hal_float_t)rotor_pulses);                         



    
    /*
      when the encoder first powers on it does not know where the index point is.
      the first time that the index is passed a flag is recorded in the encoders memory.
      We know that the exact reference point is the center of the z hall sensor which is
      always at UVW(101) = 30 degrees
    */
    if(!prev_referenced  && *inst->referenced) {
        *inst->reference_turn = *inst->raw_angle / rotor_pulses;
    }

    if(*inst->referenced) {
       *inst->reference_angle = (*inst->reference_turn * rotor_pulses) + ((*inst->rotor_u_offset + hallRotorCount(rotor_pulses, 0, 1, 0, 1)) % rotor_pulses);
    }
 
        
    /*
        z_count changes only when a full rotation past the index point is made
        once z_count changes and index_enable and the encoder is referenced
        we set the index to the just crossed index point
    */  
    if(*inst->index_enable && *inst->referenced && (prev_z_counter != *inst->z_counter)) {
        //fix this we should be able to set index offset instantly based upon direction

        if(*inst->raw_angle < *inst->reference_angle) {
            inst->index_offset = inst->full_count + *inst->reference_angle - *inst->raw_angle;
        } else {
            inst->index_offset = inst->full_count + *inst->ppr - *inst->raw_angle + *inst->reference_angle;
        }
          
        *inst->index_enable = 0;
    }
    
    
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
    HM2_PRINT("    Timer Addr: 0x%X\n", hm2->sigma5abs.timer_addr);
    HM2_PRINT("    Rx Count Addr: 0x%X\n", hm2->sigma5abs.rx_count_addr);
    HM2_PRINT("    Rx0 Addr: 0x%X\n", hm2->sigma5abs.rx0_addr);
    HM2_PRINT("    Rx1 Addr: 0x%X\n", hm2->sigma5abs.rx1_addr);
    HM2_PRINT("    Rx2 Addr: 0x%X\n", hm2->sigma5abs.rx2_addr);
    HM2_PRINT("    Rx3 Addr: 0x%X\n", hm2->sigma5abs.rx3_addr);
    HM2_PRINT("    Rx4 Addr: 0x%X\n", hm2->sigma5abs.rx4_addr);
    HM2_PRINT("    Status Addr: 0x%X\n", hm2->sigma5abs.status_addr);
}


