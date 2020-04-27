from struct import pack


OPB_CONTROLLER = 'adc5g_controller'
OPB_DATA_FMT = '>H2B'


def inc_mmcm_phase(roach, zdok_n, inc=1):
    """
    This increments (or decrements) the MMCM clk-to-data phase relationship by 
    (1/56) * Pvco, where VCO is depends on the MMCM configuration.

    inc_mmcm_phase(roach, zdok_n)        # default increments
    inc_mmcm_phase(roach, zdok_n, inc=0) # set inc=0 to decrement
    """
    reg_val = pack(OPB_DATA_FMT, (1<<(zdok_n*4)) + (inc<<(1+zdok_n*4)), 0x0, 0x0)
    roach.blindwrite(OPB_CONTROLLER, reg_val, offset=0x0)

def set_io_delay(r,zdok,core,delay,bit='all',regname='adc5g_controller'):
    ADC_BITS = 8
    if bit == 'all':
        bit_range = list(range(ADC_BITS))
    else:
        bit_range = [bit]
    for i in bit_range:
        data_pin = (core<<3) + i
        reg_val = (delay<<24) + (data_pin<<16) + 0x01
        reg_val_str = pack('>L',reg_val)
        r.blindwrite(regname,reg_val_str,offset=((4+zdok)*4))
